#include "ColliderManager.h"
#include "GameObject.h"
#include "SceneManager.h"
#include "SphereCollider.h"
#include "BoxCollider.h"
#include "CapsuleCollider.h"
#include "Assert.h"
#include <algorithm>
#include <cmath>
#include <cfloat>

namespace {
	// ワールド空間AABB同士の重なり判定。
	// broad-phase の候補絞り込みに使う。
	inline bool IntersectAABBWorld(const AABB& a, const AABB& b) noexcept {
		return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
			(a.min.y <= b.max.y && a.max.y >= b.min.y) &&
			(a.min.z <= b.max.z && a.max.z >= b.min.z);
	}
	// 3次元ベクトルの内積。
	// 投影長、角度判定、SAT などの基礎計算に使う。
	inline float Dot3(const VECTOR& a, const VECTOR& b) noexcept { return a.x * b.x + a.y * b.y + a.z * b.z; }
	// ベクトル長の二乗。
	// sqrt を避けて距離比較を軽くするために使う。
	inline float LenSq(const VECTOR& v) noexcept { return Dot3(v, v); }
	// ベクトル長。
	inline float Len3(const VECTOR& v) noexcept { return std::sqrt((std::max)(LenSq(v),0.0f)); }
	// 安全正規化。
	// ゼロ長ベクトルに近い場合は fallback を返して NaN を防ぐ。
	inline VECTOR SafeNorm(const VECTOR& v, const VECTOR& fallback = VGet(1,0,0)) noexcept {
		const float l = Len3(v);
		if (l >1e-6f) return VScale(v,1.0f / l);
		return fallback;
	}
	// 内積の絶対値。
	// SAT で各軸への投影半径を求める時に使う。
	inline float AbsDot3(const VECTOR& a, const VECTOR& b) noexcept { return std::fabs(Dot3(a, b)); }

	inline VECTOR ToObbLocal(const VECTOR& worldPoint, const BoxCollider* box) noexcept {
		const VECTOR d = VSub(worldPoint, box->GetCenter());
		return VGet(
			Dot3(d, box->GetAxisX()),
			Dot3(d, box->GetAxisY()),
			Dot3(d, box->GetAxisZ())
		);
	}

	inline VECTOR FromObbLocalVector(const VECTOR& localVector, const BoxCollider* box) noexcept {
		VECTOR v = VGet(0, 0, 0);
		v = VAdd(v, VScale(box->GetAxisX(), localVector.x));
		v = VAdd(v, VScale(box->GetAxisY(), localVector.y));
		v = VAdd(v, VScale(box->GetAxisZ(), localVector.z));
		return v;
	}

	inline bool SweepSphereAgainstBox(
		const VECTOR& prevCenter,
		const VECTOR& currCenter,
		float radius,
		const BoxCollider* box,
		float* outHitT,
		VECTOR* outNormalWorld,
		VECTOR* outHitCenterWorld) noexcept {
		if (!box) return false;

		const VECTOR p0 = ToObbLocal(prevCenter, box);
		const VECTOR p1 = ToObbLocal(currCenter, box);
		const VECTOR d = VSub(p1, p0);
		const VECTOR e = VAdd(box->GetHalfExtents(), VGet(radius, radius, radius));

		float tMin = 0.0f;
		float tMax = 1.0f;
		VECTOR hitNormalLocal = VGet(0, 0, 0);
		const float p0Arr[3] = { p0.x, p0.y, p0.z };
		const float dArr[3] = { d.x, d.y, d.z };
		const float eArr[3] = { e.x, e.y, e.z };

		for (int axis = 0; axis < 3; ++axis) {
			const float origin = p0Arr[axis];
			const float dir = dArr[axis];
			const float extent = eArr[axis];

			if (std::fabs(dir) < 1e-6f) {
				if (origin < -extent || origin > extent) {
					return false;
				}
				continue;
			}

			float t1 = (-extent - origin) / dir;
			float t2 = (extent - origin) / dir;
			VECTOR nearNormal = VGet(0, 0, 0);
			if (axis == 0) nearNormal.x = (t1 <= t2) ? -1.0f : 1.0f;
			else if (axis == 1) nearNormal.y = (t1 <= t2) ? -1.0f : 1.0f;
			else nearNormal.z = (t1 <= t2) ? -1.0f : 1.0f;

			if (t1 > t2) std::swap(t1, t2);
			if (t1 > tMin) {
				tMin = t1;
				hitNormalLocal = nearNormal;
			}
			tMax = (std::min)(tMax, t2);
			if (tMin > tMax) {
				return false;
			}
		}

		if (tMin < 0.0f || tMin > 1.0f) {
			return false;
		}

		if (outHitT) *outHitT = tMin;
		if (outNormalWorld) *outNormalWorld = SafeNorm(FromObbLocalVector(hitNormalLocal, box), VGet(1, 0, 0));
		if (outHitCenterWorld) *outHitCenterWorld = VAdd(prevCenter, VScale(VSub(currCenter, prevCenter), tMin));
		return true;
	}

	// 子Colliderの owner から、その親Transformに紐づく GameObject を取り出す。
	// bubbleEventsToParentOwner が true の時だけ、owner に加えて親GameObject にもイベントを送る。
	inline GameObject* GetParentOwner(Collider* c) noexcept {
		if (!c || !c->bubbleEventsToParentOwner || !c->owner) return nullptr;
		Transform* parentTf = c->owner->transform.Parent();
		if (!parentTf) return nullptr;
		GameObject* parentOwner = parentTf->Owner();
		if (!parentOwner || parentOwner == c->owner) return nullptr;
		return parentOwner;
	}
}

// 明示終了。
// 終了中に Unregister が走っても安全なように内部コンテナを空にする。
void ColliderManager::Shutdown() noexcept {
	const bool wasShuttingDown = _shuttingDown.exchange(true, std::memory_order_relaxed);
	if (wasShuttingDown) {
		return;
	}

	_currPairs.clear();
	_prevPairs.clear();
	_colliders.clear();
	_prevAABBs.clear();
}

// 衝突開始イベントの配送。
// Trigger と通常衝突で呼ぶコールバックを分けている。
void ColliderManager::DispatchEnter(Collider* a, Collider* b) {
	if (!a || !b) return;
	if (a->isTrigger || b->isTrigger) {
		if (a->sendEventsToOwner && a->owner) a->owner->OnTriggerEnter(a, b);
		if (b->sendEventsToOwner && b->owner) b->owner->OnTriggerEnter(b, a);
		if (GameObject* p = GetParentOwner(a)) p->OnTriggerEnter(a, b);
		if (GameObject* p = GetParentOwner(b)) p->OnTriggerEnter(b, a);
		a->OnTriggerEnter(b);
		b->OnTriggerEnter(a);
	}
	else {
		if (a->sendEventsToOwner && a->owner) a->owner->OnCollisionEnter(a, b);
		if (b->sendEventsToOwner && b->owner) b->owner->OnCollisionEnter(b, a);
		if (GameObject* p = GetParentOwner(a)) p->OnCollisionEnter(a, b);
		if (GameObject* p = GetParentOwner(b)) p->OnCollisionEnter(b, a);
		a->OnCollisionEnter(b);
		b->OnCollisionEnter(a);
	}
}

// 衝突継続イベントの配送。
void ColliderManager::DispatchStay(Collider* a, Collider* b) {
	if (!a || !b) return;
	if (a->isTrigger || b->isTrigger) {
		if (a->sendEventsToOwner && a->owner) a->owner->OnTriggerStay(a, b);
		if (b->sendEventsToOwner && b->owner) b->owner->OnTriggerStay(b, a);
		if (GameObject* p = GetParentOwner(a)) p->OnTriggerStay(a, b);
		if (GameObject* p = GetParentOwner(b)) p->OnTriggerStay(b, a);
		a->OnTriggerStay(b);
		b->OnTriggerStay(a);
	}
	else {
		if (a->sendEventsToOwner && a->owner) a->owner->OnCollisionStay(a, b);
		if (b->sendEventsToOwner && b->owner) b->owner->OnCollisionStay(b, a);
		if (GameObject* p = GetParentOwner(a)) p->OnCollisionStay(a, b);
		if (GameObject* p = GetParentOwner(b)) p->OnCollisionStay(b, a);
		a->OnCollisionStay(b);
		b->OnCollisionStay(a);
	}
}

// 衝突終了イベントの配送。
void ColliderManager::DispatchExit(Collider* a, Collider* b) {
	if (!a || !b) return;
	if (a->isTrigger || b->isTrigger) {
		if (a->sendEventsToOwner && a->owner) a->owner->OnTriggerExit(a, b);
		if (b->sendEventsToOwner && b->owner) b->owner->OnTriggerExit(b, a);
		if (GameObject* p = GetParentOwner(a)) p->OnTriggerExit(a, b);
		if (GameObject* p = GetParentOwner(b)) p->OnTriggerExit(b, a);
		a->OnTriggerExit(b);
		b->OnTriggerExit(a);
	}
	else {
		if (a->sendEventsToOwner && a->owner) a->owner->OnCollisionExit(a, b);
		if (b->sendEventsToOwner && b->owner) b->owner->OnCollisionExit(b, a);
		if (GameObject* p = GetParentOwner(a)) p->OnCollisionExit(a, b);
		if (GameObject* p = GetParentOwner(b)) p->OnCollisionExit(b, a);
		a->OnCollisionExit(b);
		b->OnCollisionExit(a);
	}
}

// 判定関数選択用の種別ペア比較。
// a-b / b-a の順不同で同じ組み合わせとして扱う。
static bool IsPair(Collider::Kind a, Collider::Kind b, Collider::Kind x, Collider::Kind y) {
	return (a == x && b == y) || (a == y && b == x);
}

// 現フレームAABBと前フレームAABBを合成した swept AABB を返す。
// これは「前回位置から今回位置までに通過した可能性のある領域」を近似したもの。
// 完全な連続衝突判定ではないが、高速移動時の broad-phase 取りこぼしを減らせる。
AABB ColliderManager::GetSweptAABB(Collider* collider) const {
	const AABB curr = collider->GetAABB();
	auto it = _prevAABBs.find(collider);
	if (it == _prevAABBs.end()) {
		return curr;
	}

	bool useSweep = collider->enableCCD;
	if (!useSweep) {
		// enableCCD が明示されていない場合でも、
		// フレーム間速度が閾値を超えたら sweep を有効化する。
		// speed^2 = distance^2 / dt^2 で sqrt を避けて比較している。
		const VECTOR prevCenter = VGet((it->second.min.x + it->second.max.x) * 0.5f,
			(it->second.min.y + it->second.max.y) * 0.5f,
			(it->second.min.z + it->second.max.z) * 0.5f);
		const VECTOR currCenter = VGet((curr.min.x + curr.max.x) * 0.5f,
			(curr.min.y + curr.max.y) * 0.5f,
			(curr.min.z + curr.max.z) * 0.5f);
		const VECTOR d = VSub(currCenter, prevCenter);
		const float distSq = LenSq(d);
		float dt = _deltaTimeSec;
		if (dt < 1e-6f) dt = 1e-6f;
		const float speedSq = distSq / (dt * dt);
		const float thr = collider->ccdDistanceThreshold;
		useSweep = (speedSq > thr * thr);
	}

	if (!useSweep) {
		return curr;
	}

	// 前回AABBと今回AABBの union を取り、移動区間全体を含むAABBにする。
	AABB sweep = curr;
	sweep.min.x = (std::min)(sweep.min.x, it->second.min.x);
	sweep.min.y = (std::min)(sweep.min.y, it->second.min.y);
	sweep.min.z = (std::min)(sweep.min.z, it->second.min.z);
	sweep.max.x = (std::max)(sweep.max.x, it->second.max.x);
	sweep.max.y = (std::max)(sweep.max.y, it->second.max.y);
	sweep.max.z = (std::max)(sweep.max.z, it->second.max.z);
	sweep.center = VScale(VAdd(sweep.min, sweep.max), 0.5f);
	return sweep;
}

// 全コライダのワールド形状更新。
// Transform 変更後に narrow-phase へ入る前提をそろえる。
void ColliderManager::UpdateAllShapes() {
	for (auto* c : _colliders) {
		if (!c) continue;
		c->UpdateShape();
	}
}

// 今フレームの衝突候補を再構築。
void ColliderManager::BuildCurrentPairs() {
	_currPairs.clear();
	_contacts.clear();
	SpatialPartitioning();
}

// 空間分割による broad-phase。
// swept AABB をセルに登録し、同じセルにいるペアだけを詳細判定に回す。
void ColliderManager::SpatialPartitioning() {
	const int currentSceneId = SceneManager::Instance().CurrentSceneId();

	struct CellKey {
		int x{};
		int y{};
		int z{};
		bool operator==(const CellKey& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
	};
	struct CellHash {
		size_t operator()(const CellKey& k) const noexcept {
			size_t h =1469598103934665603ull;
			h ^= static_cast<size_t>(k.x) +0x9e3779b97f4a7c15ull + (h <<6) + (h >>2);
			h ^= static_cast<size_t>(k.y) +0x9e3779b97f4a7c15ull + (h <<6) + (h >>2);
			h ^= static_cast<size_t>(k.z) +0x9e3779b97f4a7c15ull + (h <<6) + (h >>2);
			return h;
		}
	};

	const float cellSize = (_cellSize >0.01f) ? _cellSize :0.01f;
	auto ToCell = [&](float v) {
		return static_cast<int>(std::floor(v / cellSize));
	};

	std::unordered_map<CellKey, std::vector<Collider*>, CellHash> grid;
	grid.reserve(_colliders.size());

	auto PassCommonFilters = [&](Collider* c) -> bool {
		if (!c) return false;
		if (!c->owner) { if (c->useSceneFilter) return false; }
		else { if (c->useSceneFilter && c->owner->_ownerSceneId != currentSceneId) return false; }
		if (!c->IsEnabled()) return false;
		if (c->owner && !c->owner->IsActive()) return false;
		return true;
	};

	for (auto* c : _colliders) {
		if (!PassCommonFilters(c)) continue;

		// 高速移動体は前回位置も含めた swept AABB でセル登録することで、
		// 「今フレームの終点では離れているが途中で近づいた」候補を拾いやすくする。
		const AABB sweep = GetSweptAABB(c);
		const int minX = ToCell(sweep.min.x);
		const int minY = ToCell(sweep.min.y);
		const int minZ = ToCell(sweep.min.z);
		const int maxX = ToCell(sweep.max.x);
		const int maxY = ToCell(sweep.max.y);
		const int maxZ = ToCell(sweep.max.z);

		for (int z = minZ; z <= maxZ; ++z) {
			for (int y = minY; y <= maxY; ++y) {
				for (int x = minX; x <= maxX; ++x) {
					grid[CellKey{ x,y,z }].push_back(c);
				}
			}
		}
	}

	for (auto& [cell, cellCols] : grid) {
		const size_t n = cellCols.size();
		for (size_t i =0; i < n; ++i) {
			Collider* a = cellCols[i];
			for (size_t j = i +1; j < n; ++j) {
				Collider* b = cellCols[j];

				const auto key = MakeKey(a, b);
				if (_currPairs.contains(key)) continue;
				if (CheckLayerMaskCollisions(a, b)) continue;
				// broad-phase のAABB判定にも swept AABB を使う。
				// これによりセル登録だけ sweep して、AABB判定で落ちる状況を減らす。
				if (CheckAABBCollisionsSwept(a, b)) continue;

				_narrowHit = false;
				const auto ka = a->GetKind();
				const auto kb = b->GetKind();
				if (IsPair(ka, kb, Collider::Kind::Sphere, Collider::Kind::Sphere)) CheckSphereSphere(a, b);
				else if (IsPair(ka, kb, Collider::Kind::Sphere, Collider::Kind::Box)) CheckSphereBox(a, b);
				else if (IsPair(ka, kb, Collider::Kind::Box, Collider::Kind::Box)) CheckBoxBox(a, b);
				else if (IsPair(ka, kb, Collider::Kind::Capsule, Collider::Kind::Capsule)) CheckCapsuleCapsule(a, b);
				else if (IsPair(ka, kb, Collider::Kind::Sphere, Collider::Kind::Capsule)) CheckSphereCapsule(a, b);
				else if (IsPair(ka, kb, Collider::Kind::Box, Collider::Kind::Capsule)) CheckBoxCapsule(a, b);
				else {
					ASSERT_MSG(false, "未定義のコライダー組み合わせ: kindA=%d kindB=%d", static_cast<int>(ka), static_cast<int>(kb));
					_narrowHit = false;
				}

				if (!_narrowHit) continue;

				_currPairs.insert(key);
				if (!(a->isTrigger || b->isTrigger)) {
					ResolvePushOut(a, b);
				}
			}
		}
	}
}

// Enter / Stay / Exit の差分計算。
// 前フレーム集合と今フレーム集合を比較してイベントを確定する。
void ColliderManager::ProcessPairEvents() {
	for (const auto& k : _currPairs) {
		if (_prevPairs.contains(k)) {
			DispatchStay(k.a, k.b);
		}
		else {
			DispatchEnter(k.a, k.b);
		}
	}

	for (const auto& k : _prevPairs) {
		if (!_currPairs.contains(k)) {
			DispatchExit(k.a, k.b);
		}
	}

	_prevPairs = _currPairs;
}

// 詳細判定一式。
// 最後に現在AABBを保存し、次フレームの swept AABB 計算に使う。
void ColliderManager::CheckDetailedCollisions() {
	BuildCurrentPairs();
	ProcessPairEvents();

	for (auto* c : _colliders) {
		if (!c) continue;
		_prevAABBs[c] = c->GetAABB();
	}
}

// 押し戻し。
// 両方可動なら形状ごとの押し戻しへ、片側固定なら簡易MTVで可動側のみ移動する。
void ColliderManager::ResolvePushOut(Collider* a, Collider* b) {
	if (!a || !b) return;
	if (a->isTrigger || b->isTrigger) return;

	GameObject* oa = a->owner;
	GameObject* ob = b->owner;
	if (!oa && !ob) return;

	const bool aFixed = (!oa) || (oa && oa->isStatic);
	const bool bFixed = (!ob) || (ob && ob->isStatic);

	if (aFixed && bFixed) {
		return;
	}

	const auto ka = a->GetKind();
	const auto kb = b->GetKind();
	if (IsPair(ka, kb, Collider::Kind::Sphere, Collider::Kind::Sphere)) {
		PushOutSphereSphere(a, b);
	}
	else if (IsPair(ka, kb, Collider::Kind::Sphere, Collider::Kind::Box)) {
		PushOutSphereBox(a, b);
	}
	else if (IsPair(ka, kb, Collider::Kind::Box, Collider::Kind::Box)) {
		PushOutBoxBox(a, b);
	}
	else if (IsPair(ka, kb, Collider::Kind::Capsule, Collider::Kind::Capsule)) {
		PushOutCapsuleCapsule(a, b);
	}
	else if (IsPair(ka, kb, Collider::Kind::Sphere, Collider::Kind::Capsule)) {
		PushOutSphereCapsule(a, b);
	}
	else if (IsPair(ka, kb, Collider::Kind::Box, Collider::Kind::Capsule)) {
		PushOutBoxCapsule(a, b);
	}
	else {
		ASSERT_MSG(false, "未定義のコライダー組み合わせ: kindA=%d kindB=%d", static_cast<int>(ka), static_cast<int>(kb));
	}
}

// Sphere-Sphere 押し戻し。
// 中心間ベクトルを法線とし、半径和との差分だけ分離する。
void ColliderManager::PushOutSphereSphere(Collider* a, Collider* b) {
	auto* sa = dynamic_cast<SphereCollider*>(a);
	auto* sb = dynamic_cast<SphereCollider*>(b);
	if (!sa || !sb) return;

	GameObject* oa = static_cast<Collider*>(sa)->owner;
	GameObject* ob = static_cast<Collider*>(sb)->owner;
	if (!oa && !ob) return;

	const VECTOR ca = sa->GetCenter();
	const VECTOR cb = sb->GetCenter();
	const VECTOR d = VSub(cb, ca);
	const float dist = std::sqrt((std::max)(LenSq(d), 1e-8f));
	const float r = sa->GetRadius() + sb->GetRadius();
	const float pen = r - dist;
	if (pen <= 0.0f) return;

	const VECTOR n = VScale(d, 1.0f / dist);

	Contact ct;
	ct.a = sa;
	ct.b = sb;
	ct.normal = n;
	ct.penetration = pen;
	_contacts.push_back(ct);

	const float wA = (oa && !oa->isStatic) ? 1.0f : 0.0f;
	const float wB = (ob && !ob->isStatic) ? 1.0f : 0.0f;
	const float wSum = wA + wB;
	if (wSum <= 0.0f) return;

	const float moveA = (wA / wSum) * pen;
	const float moveB = (wB / wSum) * pen;

	if (oa && !oa->isStatic) {
		VECTOR p = oa->transform.LocalPosition();
		p = VSub(p, VScale(n, moveA));
		oa->transform.SetLocalPosition(p);
	}
	if (ob && !ob->isStatic) {
		VECTOR p = ob->transform.LocalPosition();
		p = VAdd(p, VScale(n, moveB));
		ob->transform.SetLocalPosition(p);
	}

	sa->UpdateShape();
	sb->UpdateShape();
}

// Sphere-Box 押し戻し。
// 球中心からOBBへの最近点を求め、その差ベクトルを押し戻し法線にする。
void ColliderManager::PushOutSphereBox(Collider* a, Collider* b) {
	SphereCollider* s = dynamic_cast<SphereCollider*>(a);
	BoxCollider* box = dynamic_cast<BoxCollider*>(b);
	if (!s || !box) {
		s = dynamic_cast<SphereCollider*>(b);
		box = dynamic_cast<BoxCollider*>(a);
	}
	if (!s || !box) return;

	GameObject* os = static_cast<Collider*>(s)->owner;
	GameObject* obox = static_cast<Collider*>(box)->owner;
	if (!os || os->isStatic) return;
	if (obox && os == obox) return;

	const VECTOR c = s->GetCenter();
	const VECTOR d = VSub(c, box->GetCenter());
	float x = Dot3(d, box->GetAxisX());
	float y = Dot3(d, box->GetAxisY());
	float z = Dot3(d, box->GetAxisZ());
	x = std::clamp(x, -box->GetHalfExtents().x, box->GetHalfExtents().x);
	y = std::clamp(y, -box->GetHalfExtents().y, box->GetHalfExtents().y);
	z = std::clamp(z, -box->GetHalfExtents().z, box->GetHalfExtents().z);
	VECTOR closest = box->GetCenter();
	closest = VAdd(closest, VScale(box->GetAxisX(), x));
	closest = VAdd(closest, VScale(box->GetAxisY(), y));
	closest = VAdd(closest, VScale(box->GetAxisZ(), z));

	VECTOR diff = VSub(c, closest);
	float dist = std::sqrt((std::max)(LenSq(diff), 1e-8f));
	float pen = s->GetRadius() - dist;
	VECTOR n = VGet(0, 0, 0);
	bool ccdResolved = false;

	if (pen > 0.0f) {
		n = VScale(diff, 1.0f / dist);
	}
	else {
		auto prevIt = _prevAABBs.find(s);
		if (prevIt == _prevAABBs.end()) return;

		const VECTOR prevCenter = prevIt->second.center;
		const VECTOR move = VSub(c, prevCenter);
		const float distSq = LenSq(move);
		float dt = _deltaTimeSec;
		if (dt < 1e-6f) dt = 1e-6f;
		const float speedSq = distSq / (dt * dt);
		const float thr = s->ccdDistanceThreshold;
		const bool useSweep = s->enableCCD || box->enableCCD || speedSq > thr * thr;
		if (!useSweep) return;

		float hitT = 0.0f;
		VECTOR hitNormal = VGet(0, 0, 0);
		VECTOR hitCenter = VGet(0, 0, 0);
		if (!SweepSphereAgainstBox(prevCenter, c, s->GetRadius(), box, &hitT, &hitNormal, &hitCenter)) return;

		n = SafeNorm(hitNormal, VGet(1, 0, 0));
		const VECTOR targetCenter = VAdd(hitCenter, VScale(n, 1e-4f));
		const VECTOR centerDelta = VSub(targetCenter, c);
		pen = Len3(centerDelta);
		if (pen < 1e-4f) pen = 1e-4f;

		VECTOR p = os->transform.LocalPosition();
		p = VAdd(p, centerDelta);
		os->transform.SetLocalPosition(p);
		s->UpdateShape();
		box->UpdateShape();
		ccdResolved = true;
	}

	Contact ct;
	ct.a = box;
	ct.b = s;
	ct.normal = n;
	ct.penetration = pen;
	_contacts.push_back(ct);

	if (ccdResolved) {
		return;
	}

	VECTOR p = os->transform.LocalPosition();
	p = VAdd(p, VScale(n, pen));
	os->transform.SetLocalPosition(p);

	s->UpdateShape();
	box->UpdateShape();
}

// Box-Box 押し戻し。
// OBB同士は SAT(Separating Axis Theorem) ベースで最小貫通軸を求める。
// 分離軸候補は面法線6本 + 辺同士の外積9本。
void ColliderManager::PushOutBoxBox(Collider* a, Collider* b) {
	auto* ba = dynamic_cast<BoxCollider*>(a);
	auto* bb = dynamic_cast<BoxCollider*>(b);
	if (!ba || !bb) return;

	GameObject* oa = ba->owner;
	GameObject* ob = bb->owner;
	if (!oa && !ob) return;

	const VECTOR A0 = SafeNorm(ba->GetAxisX(), VGet(1,0,0));
	const VECTOR A1 = SafeNorm(ba->GetAxisY(), VGet(0,1,0));
	const VECTOR A2 = SafeNorm(ba->GetAxisZ(), VGet(0,0,1));
	const VECTOR B0 = SafeNorm(bb->GetAxisX(), VGet(1,0,0));
	const VECTOR B1 = SafeNorm(bb->GetAxisY(), VGet(0,1,0));
	const VECTOR B2 = SafeNorm(bb->GetAxisZ(), VGet(0,0,1));
	const float aExt[3] = { ba->GetHalfExtents().x, ba->GetHalfExtents().y, ba->GetHalfExtents().z };
	const float bExt[3] = { bb->GetHalfExtents().x, bb->GetHalfExtents().y, bb->GetHalfExtents().z };

	const VECTOR tV = VSub(bb->GetCenter(), ba->GetCenter());
	const float tA[3] = { Dot3(tV, A0), Dot3(tV, A1), Dot3(tV, A2) };

	float R[3][3] = {
		{ Dot3(A0, B0), Dot3(A0, B1), Dot3(A0, B2) },
		{ Dot3(A1, B0), Dot3(A1, B1), Dot3(A1, B2) },
		{ Dot3(A2, B0), Dot3(A2, B1), Dot3(A2, B2) },
	};
	const float eps =1e-6f;
	float AbsR[3][3];
	for (int i =0; i <3; ++i) {
		for (int j =0; j <3; ++j) {
			AbsR[i][j] = std::fabs(R[i][j]) + eps;
		}
	}

	float bestPen = FLT_MAX;
	VECTOR bestAxisW = VGet(1,0,0);
	bool bestAxisIsFace = false;

	auto ConsiderAxis = [&](const VECTOR& axisW, float dist, float ra, float rb, bool isFaceAxis) {
		// pen = (両者の投影半径の和) - (中心間投影距離)
		// 最小の貫通量を持つ軸が MTV になる。
		const float sep = (ra + rb) - dist;
		const float bias = (!isFaceAxis && bestAxisIsFace) ? 1e-4f : 0.0f;
		if (sep + bias < bestPen || (isFaceAxis && !bestAxisIsFace && sep <= bestPen + 1e-4f)) {
			bestPen = sep;
			bestAxisW = axisW;
			bestAxisIsFace = isFaceAxis;
		}
	};

	for (int i =0; i <3; ++i) {
		float ra = aExt[i];
		float rb = bExt[0] * AbsR[i][0] + bExt[1] * AbsR[i][1] + bExt[2] * AbsR[i][2];
		float dist = std::fabs(tA[i]);
		ConsiderAxis((i ==0) ? A0 : (i ==1) ? A1 : A2, dist, ra, rb, true);
	}

	for (int j =0; j <3; ++j) {
		float ra = aExt[0] * AbsR[0][j] + aExt[1] * AbsR[1][j] + aExt[2] * AbsR[2][j];
		float rb = bExt[j];
		float dist = std::fabs(tA[0] * R[0][j] + tA[1] * R[1][j] + tA[2] * R[2][j]);
		ConsiderAxis((j ==0) ? B0 : (j ==1) ? B1 : B2, dist, ra, rb, true);
	}

	auto CrossAxis = [&](const VECTOR& aAxis, const VECTOR& bAxis) {
		return VCross(aAxis, bAxis);
	};

	// i=0,j=0
	{
		VECTOR ax = CrossAxis(A0, B0);
		const float len = Len3(ax);
		if (len >1e-5f) {
			ax = VScale(ax,1.0f / len);
			const float ra = aExt[1] * AbsR[2][0] + aExt[2] * AbsR[1][0];
			const float rb = bExt[1] * AbsR[0][2] + bExt[2] * AbsR[0][1];
			const float dist = std::fabs(tA[2] * R[1][0] - tA[1] * R[2][0]);
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}
	// i=0,j=1
	{
		VECTOR ax = CrossAxis(A0, B1);
		const float len = Len3(ax);
		if (len >1e-5f) {
			ax = VScale(ax,1.0f / len);
			const float ra = aExt[1] * AbsR[2][1] + aExt[2] * AbsR[1][1];
			const float rb = bExt[0] * AbsR[0][2] + bExt[2] * AbsR[0][0];
			const float dist = std::fabs(tA[2] * R[1][1] - tA[1] * R[2][1]);
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}
	// i=0,j=2
	{
		VECTOR ax = CrossAxis(A0, B2);
		const float len = Len3(ax);
		if (len >1e-5f) {
			ax = VScale(ax,1.0f / len);
			const float ra = aExt[1] * AbsR[2][2] + aExt[2] * AbsR[1][2];
			const float rb = bExt[0] * AbsR[0][1] + bExt[1] * AbsR[0][0];
			const float dist = std::fabs(tA[2] * R[1][2] - tA[1] * R[2][2]);
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}
	// i=1,j=0
	{
		VECTOR ax = CrossAxis(A1, B0);
		const float len = Len3(ax);
		if (len >1e-5f) {
			ax = VScale(ax,1.0f / len);
			const float ra = aExt[0] * AbsR[2][0] + aExt[2] * AbsR[0][0];
			const float rb = bExt[1] * AbsR[1][2] + bExt[2] * AbsR[1][1];
			const float dist = std::fabs(tA[0] * R[2][0] - tA[2] * R[0][0]);
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}
	// i=1,j=1
	{
		VECTOR ax = CrossAxis(A1, B1);
		const float len = Len3(ax);
		if (len >1e-5f) {
			ax = VScale(ax,1.0f / len);
			const float ra = aExt[0] * AbsR[2][1] + aExt[2] * AbsR[0][1];
			const float rb = bExt[0] * AbsR[1][2] + bExt[2] * AbsR[1][0];
			const float dist = std::fabs(tA[0] * R[2][1] - tA[2] * R[0][1]);
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}
	// i=1,j=2
	{
		VECTOR ax = CrossAxis(A1, B2);
		const float len = Len3(ax);
		if (len >1e-5f) {
			ax = VScale(ax,1.0f / len);
			const float ra = aExt[0] * AbsR[2][2] + aExt[2] * AbsR[0][2];
			const float rb = bExt[0] * AbsR[1][1] + bExt[1] * AbsR[1][0];
			const float dist = std::fabs(tA[0] * R[2][2] - tA[2] * R[0][2]);
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}
	// i=2,j=0
	{
		VECTOR ax = CrossAxis(A2, B0);
		const float len = Len3(ax);
		if (len >1e-5f) {
			ax = VScale(ax,1.0f / len);
			const float ra = aExt[0] * AbsR[1][0] + aExt[1] * AbsR[0][0];
			const float rb = bExt[1] * AbsR[2][2] + bExt[2] * AbsR[2][1];
			const float dist = std::fabs(tA[1] * R[0][0] - tA[0] * R[1][0]);
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}
	// i=2,j=1
	{
		VECTOR ax = CrossAxis(A2, B1);
		const float len = Len3(ax);
		if (len >1e-5f) {
			ax = VScale(ax,1.0f / len);
			const float ra = aExt[0] * AbsR[1][1] + aExt[1] * AbsR[0][1];
			const float rb = bExt[0] * AbsR[2][2] + bExt[2] * AbsR[2][0];
			const float dist = std::fabs(tA[1] * R[0][1] - tA[0] * R[1][1]);
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}
	// i=2,j=2
	{
		VECTOR ax = CrossAxis(A2, B2);
		const float len = Len3(ax);
		if (len >1e-5f) {
			ax = VScale(ax,1.0f / len);
			const float ra = aExt[0] * AbsR[1][2] + aExt[1] * AbsR[0][2];
			const float rb = bExt[0] * AbsR[2][1] + bExt[1] * AbsR[2][0];
			const float dist = std::fabs(tA[1] * R[0][2] - tA[0] * R[1][2]);
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}

	if (bestPen == FLT_MAX || bestPen <=0.0f) return;

	VECTOR n = bestAxisW;
	if (Dot3(tV, n) <0.0f) {
		n = VScale(n, -1.0f);
	}
	const float pen = bestPen;

	Contact ct;
	ct.a = ba;
	ct.b = bb;
	ct.normal = n;
	ct.penetration = pen;
	_contacts.push_back(ct);

	const float wA = (oa && !oa->isStatic) ? 1.0f : 0.0f;
	const float wB = (ob && !ob->isStatic) ? 1.0f : 0.0f;
	const float wSum = wA + wB;
	if (wSum <=0.0f) return;

	const float moveA = (wA / wSum) * pen;
	const float moveB = (wB / wSum) * pen;

	if (oa && !oa->isStatic) {
		VECTOR p = oa->transform.LocalPosition();
		p = VSub(p, VScale(n, moveA));
		oa->transform.SetLocalPosition(p);
	}
	if (ob && !ob->isStatic) {
		VECTOR p = ob->transform.LocalPosition();
		p = VAdd(p, VScale(n, moveB));
		ob->transform.SetLocalPosition(p);
	}

	ba->UpdateShape();
	bb->UpdateShape();
}

// Capsule-Capsule 押し戻し。
// 2本の線分の最近点同士を求め、その法線方向へ半径和ぶん分離する。
void ColliderManager::PushOutCapsuleCapsule(Collider* a, Collider* b) {
	auto* ca = dynamic_cast<CapsuleCollider*>(a);
	auto* cb = dynamic_cast<CapsuleCollider*>(b);
	if (!ca || !cb) return;

	GameObject* oa = ca->owner;
	GameObject* ob = cb->owner;
	if (!oa && !ob) return;

	const VECTOR p1 = ca->GetBottom();
	const VECTOR q1 = ca->GetTop();
	const VECTOR p2 = cb->GetBottom();
	const VECTOR q2 = cb->GetTop();

	const VECTOR d1 = VSub(q1, p1);
	const VECTOR d2 = VSub(q2, p2);
	const VECTOR r0 = VSub(p1, p2);
	const float a11 = Dot3(d1, d1);
	const float a22 = Dot3(d2, d2);
	const float a12 = Dot3(d1, d2);
	const float b1 = Dot3(d1, r0);
	const float b2 = Dot3(d2, r0);

	float s =0.0f;
	float t =0.0f;

	const float denom = a11 * a22 - a12 * a12;
	if (denom >1e-6f) {
		s = (a12 * b2 - a22 * b1) / denom;
		s = std::clamp(s,0.0f,1.0f);
	}
	else {
		s =0.0f;
	}

	const float tNom = a12 * s + b2;
	if (a22 >1e-6f) {
		t = tNom / a22;
		t = std::clamp(t,0.0f,1.0f);
	}
	else {
		t =0.0f;
	}

	const float sNom = a12 * t - b1;
	if (a11 >1e-6f) {
		s = sNom / a11;
		s = std::clamp(s,0.0f,1.0f);
	}
	else {
		s =0.0f;
	}

	const VECTOR c1 = VAdd(p1, VScale(d1, s));
	const VECTOR c2 = VAdd(p2, VScale(d2, t));
	VECTOR diff = VSub(c1, c2);
	const float dist = std::sqrt((std::max)(LenSq(diff), 1e-8f));
	const float r = ca->GetRadius() + cb->GetRadius();
	const float pen = r - dist;
	if (pen <= 0.0f) return;

	const VECTOR n = VScale(diff, 1.0f / dist);

	Contact ct;
	ct.a = ca;
	ct.b = cb;
	ct.normal = n;
	ct.penetration = pen;
	_contacts.push_back(ct);

	const float wA = (oa && !oa->isStatic) ? 1.0f : 0.0f;
	const float wB = (ob && !ob->isStatic) ? 1.0f : 0.0f;
	const float wSum = wA + wB;
	if (wSum <= 0.0f) return;

	const float moveA = (wA / wSum) * pen;
	const float moveB = (wB / wSum) * pen;

	if (oa && !oa->isStatic) {
		VECTOR p = oa->transform.LocalPosition();
		p = VAdd(p, VScale(n, moveA));
		oa->transform.SetLocalPosition(p);
	}
	if (ob && !ob->isStatic) {
		VECTOR p = ob->transform.LocalPosition();
		p = VSub(p, VScale(n, moveB));
		ob->transform.SetLocalPosition(p);
	}

	ca->UpdateShape();
	cb->UpdateShape();
}

// Sphere-Capsule 押し戻し。
// カプセル軸線分上の最近点を求め、球中心との差で法線を作る。
void ColliderManager::PushOutSphereCapsule(Collider* a, Collider* b) {
	SphereCollider* s = dynamic_cast<SphereCollider*>(a);
	CapsuleCollider* c = dynamic_cast<CapsuleCollider*>(b);
	bool sphereIsA = true;
	if (!s || !c) {
		s = dynamic_cast<SphereCollider*>(b);
		c = dynamic_cast<CapsuleCollider*>(a);
		sphereIsA = false;
	}
	if (!s || !c) return;

	GameObject* os = s->owner;
	GameObject* oc = c->owner;
	if (!os && !oc) return;

	const VECTOR p = c->GetBottom();
	const VECTOR q = c->GetTop();
	const VECTOR seg = VSub(q, p);
	const VECTOR v = VSub(s->GetCenter(), p);

	const float segLenSq = Dot3(seg, seg);
	float t =0.0f;
	if (segLenSq >1e-6f) {
		t = Dot3(v, seg) / segLenSq;
		t = std::clamp(t,0.0f,1.0f);
	}

	const VECTOR closest = VAdd(p, VScale(seg, t));
	VECTOR diff = VSub(s->GetCenter(), closest);
	const float dist = std::sqrt((std::max)(LenSq(diff), 1e-8f));
	const float r = s->GetRadius() + c->GetRadius();
	const float pen = r - dist;
	if (pen <= 0.0f) return;

	VECTOR n = VScale(diff, 1.0f / dist);

	Contact ct;
	ct.a = c;
	ct.b = s;
	ct.normal = n;
	ct.penetration = pen;
	_contacts.push_back(ct);

	const float wS = (os && !os->isStatic) ? 1.0f : 0.0f;
	const float wC = (oc && !oc->isStatic) ? 1.0f : 0.0f;
	const float wSum = wS + wC;
	if (wSum <= 0.0f) return;

	const float moveS = (wS / wSum) * pen;
	const float moveC = (wC / wSum) * pen;

	if (os && !os->isStatic) {
		VECTOR p0 = os->transform.LocalPosition();
		p0 = VAdd(p0, VScale(n, moveS));
		os->transform.SetLocalPosition(p0);
	}
	if (oc && !oc->isStatic) {
		VECTOR p0 = oc->transform.LocalPosition();
		p0 = VSub(p0, VScale(n, moveC));
		oc->transform.SetLocalPosition(p0);
	}

	s->UpdateShape();
	c->UpdateShape();
}

// Box-Capsule 押し戻し。
// カプセル端点をOBBローカル空間へ移し、AABBとの最近点問題に落として近似する。
void ColliderManager::PushOutBoxCapsule(Collider* a, Collider* b) {
	BoxCollider* box = dynamic_cast<BoxCollider*>(a);
	CapsuleCollider* cap = dynamic_cast<CapsuleCollider*>(b);
	if (!box || !cap) {
		box = dynamic_cast<BoxCollider*>(b);
		cap = dynamic_cast<CapsuleCollider*>(a);
	}
	if (!box || !cap) return;

	GameObject* obox = box->owner;
	GameObject* ocap = cap->owner;
	if (!obox && !ocap) return;

	auto ToLocal = [&](const VECTOR& w) {
		const VECTOR d = VSub(w, box->GetCenter());
		return VGet(Dot3(d, box->GetAxisX()), Dot3(d, box->GetAxisY()), Dot3(d, box->GetAxisZ()));
	};

	const VECTOR pL = ToLocal(cap->GetBottom());
	const VECTOR qL = ToLocal(cap->GetTop());
	const VECTOR dL = VSub(qL, pL);

	const float hx = box->GetHalfExtents().x;
	const float hy = box->GetHalfExtents().y;
	const float hz = box->GetHalfExtents().z;

	auto ClampPointToAABB = [&](const VECTOR& p) {
		return VGet(
			std::clamp(p.x, -hx, hx),
			std::clamp(p.y, -hy, hy),
			std::clamp(p.z, -hz, hz)
		);
	};

	float bestDistSq = FLT_MAX;
	VECTOR bestSegPointL = pL;
	VECTOR bestBoxPointL = ClampPointToAABB(pL);

	auto DistSq = [&](const VECTOR& u, const VECTOR& v) {
		return LenSq(VSub(u, v));
	};

	auto ConsiderPoint = [&](const VECTOR& segPointL) {
		const VECTOR boxPointL = ClampPointToAABB(segPointL);
		const float distSq = DistSq(segPointL, boxPointL);
		if (distSq < bestDistSq) {
			bestDistSq = distSq;
			bestSegPointL = segPointL;
			bestBoxPointL = boxPointL;
		}
	};

	ConsiderPoint(pL);
	ConsiderPoint(qL);

	auto ConsiderT = [&](float t) {
		if (t < 0.0f || t > 1.0f) return;
		ConsiderPoint(VAdd(pL, VScale(dL, t)));
	};

	if (std::fabs(dL.x) > 1e-6f) {
		ConsiderT((-hx - pL.x) / dL.x);
		ConsiderT((hx - pL.x) / dL.x);
	}
	if (std::fabs(dL.y) > 1e-6f) {
		ConsiderT((-hy - pL.y) / dL.y);
		ConsiderT((hy - pL.y) / dL.y);
	}
	if (std::fabs(dL.z) > 1e-6f) {
		ConsiderT((-hz - pL.z) / dL.z);
		ConsiderT((hz - pL.z) / dL.z);
	}

	const float r = cap->GetRadius();
	if (bestDistSq > r * r) return;

	VECTOR diffL = VSub(bestSegPointL, bestBoxPointL);
	float dist = std::sqrt((std::max)(bestDistSq, 1e-8f));
	float pen = r - dist;
	if (pen <= 0.0f) return;

	VECTOR n;
	if (dist > 1e-6f) {
		const VECTOR diffW = VAdd(
			VAdd(VScale(box->GetAxisX(), diffL.x), VScale(box->GetAxisY(), diffL.y)),
			VScale(box->GetAxisZ(), diffL.z)
		);
		n = SafeNorm(diffW, box->GetAxisY());
	}
	else {
		const float dx = hx - std::fabs(bestSegPointL.x);
		const float dy = hy - std::fabs(bestSegPointL.y);
		const float dz = hz - std::fabs(bestSegPointL.z);
		if (dx <= dy && dx <= dz) n = VScale(box->GetAxisX(), (bestSegPointL.x >= 0.0f) ? 1.0f : -1.0f);
		else if (dy <= dz) n = VScale(box->GetAxisY(), (bestSegPointL.y >= 0.0f) ? 1.0f : -1.0f);
		else n = VScale(box->GetAxisZ(), (bestSegPointL.z >= 0.0f) ? 1.0f : -1.0f);
		pen = r + (std::min)({ dx, dy, dz });
	}

	Contact ct;
	ct.a = box;
	ct.b = cap;
	ct.normal = n;
	ct.penetration = pen;
	_contacts.push_back(ct);

	const float wBox = (obox && !obox->isStatic) ? 1.0f : 0.0f;
	const float wCap = (ocap && !ocap->isStatic) ? 1.0f : 0.0f;
	const float wSum = wBox + wCap;
	if (wSum <= 0.0f) return;

	const float moveBox = (wBox / wSum) * pen;
	const float moveCap = (wCap / wSum) * pen;

	if (obox && !obox->isStatic) {
		VECTOR p = obox->transform.LocalPosition();
		p = VSub(p, VScale(n, moveBox));
		obox->transform.SetLocalPosition(p);
	}
	if (ocap && !ocap->isStatic) {
		VECTOR p = ocap->transform.LocalPosition();
		p = VAdd(p, VScale(n, moveCap));
		ocap->transform.SetLocalPosition(p);
	}

	box->UpdateShape();
	cap->UpdateShape();
}

// 1フレームぶんのコライダ更新入口。
void ColliderManager::Update(float dtSec) {
	if (IsShuttingDown()) {
		return;
	}
	_deltaTimeSec = (dtSec > 1e-6f) ? dtSec : 1e-6f;
	UpdateAllShapes();
	CheckDetailedCollisions();
}

// Sphere-Sphere 詳細判定。
// 距離二乗と半径和二乗の比較で sqrt を避ける。
void ColliderManager::CheckSphereSphere(Collider* a, Collider* b) {
	auto* sa = dynamic_cast<SphereCollider*>(a);
	auto* sb = dynamic_cast<SphereCollider*>(b);
	if (!sa || !sb) {
		_narrowHit = false;
		return;
	}
	const VECTOR ca = sa->GetCenter();
	const VECTOR cb = sb->GetCenter();
	const VECTOR d = VSub(cb, ca);
	const float r = sa->GetRadius() + sb->GetRadius();
	_narrowHit = (LenSq(d) <= r * r);
}

// Sphere-Box 詳細判定。
// 球中心から OBB 上の最近点を求め、その距離が半径以内かを判定する。
void ColliderManager::CheckSphereBox(Collider* a, Collider* b) {
	SphereCollider* s = dynamic_cast<SphereCollider*>(a);
	BoxCollider* box = dynamic_cast<BoxCollider*>(b);
	if (!s || !box) {
		s = dynamic_cast<SphereCollider*>(b);
		box = dynamic_cast<BoxCollider*>(a);
	}
	if (!s || !box) {
		_narrowHit = false;
		return;
	}

	const VECTOR c = s->GetCenter();
	const float r = s->GetRadius();
	const VECTOR d = VSub(c, box->GetCenter());
	float x = Dot3(d, box->GetAxisX());
	float y = Dot3(d, box->GetAxisY());
	float z = Dot3(d, box->GetAxisZ());

	x = std::clamp(x, -box->GetHalfExtents().x, box->GetHalfExtents().x);
	y = std::clamp(y, -box->GetHalfExtents().y, box->GetHalfExtents().y);
	z = std::clamp(z, -box->GetHalfExtents().z, box->GetHalfExtents().z);

	VECTOR closest = box->GetCenter();
	closest = VAdd(closest, VScale(box->GetAxisX(), x));
	closest = VAdd(closest, VScale(box->GetAxisY(), y));
	closest = VAdd(closest, VScale(box->GetAxisZ(), z));

	const VECTOR diff = VSub(c, closest);
	_narrowHit = (LenSq(diff) <= r * r);
	if (_narrowHit) return;

	auto prevIt = _prevAABBs.find(s);
	if (prevIt == _prevAABBs.end()) return;

	const VECTOR prevCenter = prevIt->second.center;
	const VECTOR move = VSub(c, prevCenter);
	const float distSq = LenSq(move);
	float dt = _deltaTimeSec;
	if (dt < 1e-6f) dt = 1e-6f;
	const float speedSq = distSq / (dt * dt);
	const float thr = s->ccdDistanceThreshold;
	const bool useSweep = s->enableCCD || box->enableCCD || speedSq > thr * thr;
	if (!useSweep) return;

	_narrowHit = SweepSphereAgainstBox(prevCenter, c, r, box, nullptr, nullptr, nullptr);
}

// Box-Box 詳細判定。
// SAT により「分離軸が1本でもあれば非衝突」と判定する。
void ColliderManager::CheckBoxBox(Collider* a, Collider* b) {
	BoxCollider* ba = dynamic_cast<BoxCollider*>(a);
	BoxCollider* bb = dynamic_cast<BoxCollider*>(b);
	if (!ba || !bb) {
		_narrowHit = false;
		return;
	}

	const VECTOR A0 = ba->GetAxisX();
	const VECTOR A1 = ba->GetAxisY();
	const VECTOR A2 = ba->GetAxisZ();
	const VECTOR B0 = bb->GetAxisX();
	const VECTOR B1 = bb->GetAxisY();
	const VECTOR B2 = bb->GetAxisZ();

	const VECTOR tV = VSub(bb->GetCenter(), ba->GetCenter());
	const float t[3] = { Dot3(tV, A0), Dot3(tV, A1), Dot3(tV, A2) };

	float R[3][3] = {
		{ Dot3(A0, B0), Dot3(A0, B1), Dot3(A0, B2) },
		{ Dot3(A1, B0), Dot3(A1, B1), Dot3(A1, B2) },
		{ Dot3(A2, B0), Dot3(A2, B1), Dot3(A2, B2) },
	};

	const float eps =1e-6f;
	float AbsR[3][3];
	for (int i =0; i <3; ++i) {
		for (int j =0; j <3; ++j) {
			AbsR[i][j] = std::fabs(R[i][j]) + eps;
		}
	}

	const float aExt[3] = { ba->GetHalfExtents().x, ba->GetHalfExtents().y, ba->GetHalfExtents().z };
	const float bExt[3] = { bb->GetHalfExtents().x, bb->GetHalfExtents().y, bb->GetHalfExtents().z };

	float ra, rb;
	float tval;

	for (int i =0; i <3; ++i) {
		ra = aExt[i];
		rb = bExt[0] * AbsR[i][0] + bExt[1] * AbsR[i][1] + bExt[2] * AbsR[i][2];
		if (std::fabs(t[i]) > ra + rb) { _narrowHit = false; return; }
	}

	for (int j =0; j <3; ++j) {
		ra = aExt[0] * AbsR[0][j] + aExt[1] * AbsR[1][j] + aExt[2] * AbsR[2][j];
		rb = bExt[j];
		tval = std::fabs(t[0] * R[0][j] + t[1] * R[1][j] + t[2] * R[2][j]);
		if (tval > ra + rb) { _narrowHit = false; return; }
	}

	ra = aExt[1] * AbsR[2][0] + aExt[2] * AbsR[1][0];
	rb = bExt[1] * AbsR[0][2] + bExt[2] * AbsR[0][1];
	tval = std::fabs(t[2] * R[1][0] - t[1] * R[2][0]);
	if (tval > ra + rb) { _narrowHit = false; return; }

	ra = aExt[1] * AbsR[2][1] + aExt[2] * AbsR[1][1];
	rb = bExt[0] * AbsR[0][2] + bExt[2] * AbsR[0][0];
	tval = std::fabs(t[2] * R[1][1] - t[1] * R[2][1]);
	if (tval > ra + rb) { _narrowHit = false; return; }

	ra = aExt[1] * AbsR[2][2] + aExt[2] * AbsR[1][2];
	rb = bExt[0] * AbsR[0][1] + bExt[1] * AbsR[0][0];
	tval = std::fabs(t[2] * R[1][2] - t[1] * R[2][2]);
	if (tval > ra + rb) { _narrowHit = false; return; }

	ra = aExt[0] * AbsR[2][0] + aExt[2] * AbsR[0][0];
	rb = bExt[1] * AbsR[1][2] + bExt[2] * AbsR[1][1];
	tval = std::fabs(t[0] * R[2][0] - t[2] * R[0][0]);
	if (tval > ra + rb) { _narrowHit = false; return; }

	ra = aExt[0] * AbsR[2][1] + aExt[2] * AbsR[0][1];
	rb = bExt[0] * AbsR[1][2] + bExt[2] * AbsR[1][0];
	tval = std::fabs(t[0] * R[2][1] - t[2] * R[0][1]);
	if (tval > ra + rb) { _narrowHit = false; return; }

	ra = aExt[0] * AbsR[2][2] + aExt[2] * AbsR[0][2];
	rb = bExt[0] * AbsR[1][1] + bExt[1] * AbsR[1][0];
	tval = std::fabs(t[0] * R[2][2] - t[2] * R[0][2]);
	if (tval > ra + rb) { _narrowHit = false; return; }

	ra = aExt[0] * AbsR[1][0] + aExt[1] * AbsR[0][0];
	rb = bExt[1] * AbsR[2][2] + bExt[2] * AbsR[2][1];
	tval = std::fabs(t[1] * R[0][0] - t[0] * R[1][0]);
	if (tval > ra + rb) { _narrowHit = false; return; }

	ra = aExt[0] * AbsR[1][1] + aExt[1] * AbsR[0][1];
	rb = bExt[0] * AbsR[2][2] + bExt[2] * AbsR[2][0];
	tval = std::fabs(t[1] * R[0][1] - t[0] * R[1][1]);
	if (tval > ra + rb) { _narrowHit = false; return; }

	ra = aExt[0] * AbsR[1][2] + aExt[1] * AbsR[0][2];
	rb = bExt[0] * AbsR[2][1] + bExt[1] * AbsR[2][0];
	tval = std::fabs(t[1] * R[0][2] - t[0] * R[1][2]);
	if (tval > ra + rb) { _narrowHit = false; return; }

	_narrowHit = true;
}

// Capsule-Capsule 詳細判定。
// 線分間最近点距離と半径和の比較。
void ColliderManager::CheckCapsuleCapsule(Collider* a, Collider* b) {
	auto* ca = dynamic_cast<CapsuleCollider*>(a);
	auto* cb = dynamic_cast<CapsuleCollider*>(b);
	if (!ca || !cb) {
		_narrowHit = false;
		return;
	}

	const VECTOR p1 = ca->GetBottom();
	const VECTOR q1 = ca->GetTop();
	const VECTOR p2 = cb->GetBottom();
	const VECTOR q2 = cb->GetTop();
	const float r = ca->GetRadius() + cb->GetRadius();

	const VECTOR d1 = VSub(q1, p1);
	const VECTOR d2 = VSub(q2, p2);
	const VECTOR r0 = VSub(p1, p2);
	const float a11 = Dot3(d1, d1);
	const float a22 = Dot3(d2, d2);
	const float a12 = Dot3(d1, d2);
	const float b1 = Dot3(d1, r0);
	const float b2 = Dot3(d2, r0);

	float s =0.0f;
	float t =0.0f;

	const float denom = a11 * a22 - a12 * a12;
	if (denom >1e-6f) {
		s = (a12 * b2 - a22 * b1) / denom;
		s = std::clamp(s,0.0f,1.0f);
	}
	else {
		s =0.0f;
	}

	const float tNom = a12 * s + b2;
	if (a22 >1e-6f) {
		t = tNom / a22;
		t = std::clamp(t,0.0f,1.0f);
	}
	else {
		t =0.0f;
	}

	const float sNom = a12 * t - b1;
	if (a11 >1e-6f) {
		s = sNom / a11;
		s = std::clamp(s,0.0f,1.0f);
	}
	else {
		s =0.0f;
	}

	const VECTOR c1 = VAdd(p1, VScale(d1, s));
	const VECTOR c2 = VAdd(p2, VScale(d2, t));
	const VECTOR diff = VSub(c1, c2);

	_narrowHit = (LenSq(diff) <= r * r);
}

// Sphere-Capsule 詳細判定。
// カプセル軸線分上の最近点を求め、球中心との差で法線を作る。
void ColliderManager::CheckSphereCapsule(Collider* a, Collider* b) {
	SphereCollider* s = dynamic_cast<SphereCollider*>(a);
	CapsuleCollider* c = dynamic_cast<CapsuleCollider*>(b);
	if (!s || !c) {
		s = dynamic_cast<SphereCollider*>(b);
		c = dynamic_cast<CapsuleCollider*>(a);
	}
	if (!s || !c) {
		_narrowHit = false;
		return;
	}

	const VECTOR p = c->GetBottom();
	const VECTOR q = c->GetTop();
	const VECTOR seg = VSub(q, p);
	const VECTOR v = VSub(s->GetCenter(), p);

	const float segLenSq = Dot3(seg, seg);
	float t =0.0f;
	if (segLenSq >1e-6f) {
		t = Dot3(v, seg) / segLenSq;
		t = std::clamp(t,0.0f,1.0f);
	}

	const VECTOR closest = VAdd(p, VScale(seg, t));
	const VECTOR diff = VSub(s->GetCenter(), closest);
	const float r = s->GetRadius() + c->GetRadius();
	_narrowHit = (LenSq(diff) <= r * r);
}

// Box-Capsule 詳細判定。
// カプセル線分を OBB ローカルに変換し、AABB との最近距離を近似評価する。
void ColliderManager::CheckBoxCapsule(Collider* a, Collider* b) {
	BoxCollider* box = dynamic_cast<BoxCollider*>(a);
	CapsuleCollider* cap = dynamic_cast<CapsuleCollider*>(b);
	if (!box || !cap) {
		box = dynamic_cast<BoxCollider*>(b);
		cap = dynamic_cast<CapsuleCollider*>(a);
	}
	if (!box || !cap) {
		_narrowHit = false;
		return;
	}

	auto ToLocal = [&](const VECTOR& w) {
		const VECTOR d = VSub(w, box->GetCenter());
		return VGet(Dot3(d, box->GetAxisX()), Dot3(d, box->GetAxisY()), Dot3(d, box->GetAxisZ()));
	};

	const VECTOR pL = ToLocal(cap->GetBottom());
	const VECTOR qL = ToLocal(cap->GetTop());
	const VECTOR dL = VSub(qL, pL);

	const float hx = box->GetHalfExtents().x;
	const float hy = box->GetHalfExtents().y;
	const float hz = box->GetHalfExtents().z;

	float best = FLT_MAX;

	auto ClampPointToAABB = [&](const VECTOR& p) {
		return VGet(
			std::clamp(p.x, -hx, hx),
			std::clamp(p.y, -hy, hy),
			std::clamp(p.z, -hz, hz)
		);
	};

	auto DistSq = [&](const VECTOR& u, const VECTOR& v) {
		return LenSq(VSub(u, v));
	};

	{
		const VECTOR cp = ClampPointToAABB(pL);
		best = (std::min)(best, DistSq(pL, cp));
		const VECTOR cq = ClampPointToAABB(qL);
		best = (std::min)(best, DistSq(qL, cq));
	}

	auto ConsiderT = [&](float t) {
		if (t <0.0f || t >1.0f) return;
		const VECTOR s = VAdd(pL, VScale(dL, t));
		const VECTOR cs = ClampPointToAABB(s);
		best = (std::min)(best, DistSq(s, cs));
	};

	if (std::fabs(dL.x) >1e-6f) {
		ConsiderT((-hx - pL.x) / dL.x);
		ConsiderT((hx - pL.x) / dL.x);
	}
	if (std::fabs(dL.y) >1e-6f) {
		ConsiderT((-hy - pL.y) / dL.y);
		ConsiderT((hy - pL.y) / dL.y);
	}
	if (std::fabs(dL.z) >1e-6f) {
		ConsiderT((-hz - pL.z) / dL.z);
		ConsiderT((hz - pL.z) / dL.z);
	}

	const float r = cap->GetRadius();
	_narrowHit = (best <= r * r);
}

// デバッグ描画。
void ColliderManager::DrawDebugAll() {
	for (auto* collider : _colliders) {
		if (!collider) continue;
		collider->DrawDebug();
	}
}

// broad-phase 用 AABB のデバッグ描画。
void ColliderManager::DrawDebugAABBAll() {
	for (auto* collider : _colliders) {
		if (!collider) continue;
		collider->DrawDebugAABB();
	}
}

// 管理対象への登録。
void ColliderManager::RegisterCollider(Collider* collider) {
	if (IsShuttingDown()) {
		return;
	}
	if (!collider) return;
	if (std::find(_colliders.begin(), _colliders.end(), collider) != _colliders.end()) return;
	_colliders.push_back(collider);
}

// 管理対象からの解除。
// ペア集合と前回AABBも一緒に掃除して dangling を防ぐ。
void ColliderManager::UnregisterCollider(Collider* collider) {
	if (IsShuttingDown()) {
		return;
	}
	if (!collider) return;

	{
		auto it = std::find(_colliders.begin(), _colliders.end(), collider);
		if (it != _colliders.end()) {
			_colliders.erase(it);
		}
	}

	_prevAABBs.erase(collider);

	for (auto it = _prevPairs.begin(); it != _prevPairs.end();) {
		if (it->a == collider || it->b == collider) {
			it = _prevPairs.erase(it);
			continue;
		}
		++it;
	}
	for (auto it = _currPairs.begin(); it != _currPairs.end();) {
		if (it->a == collider || it->b == collider) {
			it = _currPairs.erase(it);
			continue;
		}
		++it;
	}
}

// Layer / Mask フィルタ。
// false ではなく true を返した時に「衝突させない」設計になっている点に注意。
bool ColliderManager::CheckLayerMaskCollisions(Collider* a, Collider* b) {
	if ((a->layer & b->mask) ==0) return true;
	if ((b->layer & a->mask) ==0) return true;
	return false;
}

// 現在AABB同士の通常判定。
bool ColliderManager::CheckAABBCollisions(Collider* a, Collider* b) {
	return !IntersectAABBWorld(a->GetAABB(), b->GetAABB());
}

// swept AABB 同士の判定。
// broad-phase での取りこぼし抑制用であり、これ自体は連続衝突判定ではない。
bool ColliderManager::CheckAABBCollisionsSwept(Collider* a, Collider* b) {
	return !IntersectAABBWorld(GetSweptAABB(a), GetSweptAABB(b));
}



