#include "ColliderManager.h"
#include "GameObject.h"
#include "SceneManager.h"
#include "SphereCollider.h"
#include "BoxCollider.h"
#include "CapsuleCollider.h"
#include "Assert.h"
#include "Time.h"
#include <algorithm> // clamp, min/max
#include <cmath>
#include <cfloat>

// ユーティリティ関数群
namespace {
	// AABB同士の交差判定（ワールド座標系）
	inline bool IntersectAABBWorld(const AABB& a, const AABB& b) noexcept {
		return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
			(a.min.y <= b.max.y && a.max.y >= b.min.y) &&
			(a.min.z <= b.max.z && a.max.z >= b.min.z);
	}
	// ベクトル内積
	inline float Dot3(const VECTOR& a, const VECTOR& b) noexcept { return a.x * b.x + a.y * b.y + a.z * b.z; }
	// ベクトル長^2
	inline float LenSq(const VECTOR& v) noexcept { return Dot3(v, v); }
	// ベクトル長
	inline float Len3(const VECTOR& v) noexcept { return std::sqrt((std::max)(LenSq(v),0.0f)); }
	// 安全正規化
	inline VECTOR SafeNorm(const VECTOR& v, const VECTOR& fallback = VGet(1,0,0)) noexcept {
		const float l = Len3(v);
		if (l >1e-6f) return VScale(v,1.0f / l);
		return fallback;
	}
	inline float AbsDot3(const VECTOR& a, const VECTOR& b) noexcept { return std::fabs(Dot3(a, b)); }
}

// 明示的終了処理
void ColliderManager::Shutdown() noexcept {
	// 多重呼び出し安全
	const bool wasShuttingDown = _shuttingDown.exchange(true, std::memory_order_relaxed);
	if (wasShuttingDown) {
		return;
	}

	// 終了時にデストラクタなどから UnregisterCollider が呼ばれても
	// unordered_set / vector に触らないように、ここで空にしておく。
	_currPairs.clear();
	_prevPairs.clear();
	_colliders.clear();
	_prevAABBs.clear();
	// _narrowHit はローカル状態なので放置でOK
}

// イベントディスパッチ
void ColliderManager::DispatchEnter(Collider* a, Collider* b) {
	if (!a || !b) return;
	if (a->isTrigger || b->isTrigger) {
		if (a->sendEventsToOwner && a->owner) a->owner->OnTriggerEnter(a, b);
		if (b->sendEventsToOwner && b->owner) b->owner->OnTriggerEnter(b, a);
		// Collider側
		a->OnTriggerEnter(b);
		b->OnTriggerEnter(a);
	}
	else {
		if (a->sendEventsToOwner && a->owner) a->owner->OnCollisionEnter(a, b);
		if (b->sendEventsToOwner && b->owner) b->owner->OnCollisionEnter(b, a);
		a->OnCollisionEnter(b);
		b->OnCollisionEnter(a);
	}
}

void ColliderManager::DispatchStay(Collider* a, Collider* b) {
	if (!a || !b) return;
	if (a->isTrigger || b->isTrigger) {
		if (a->sendEventsToOwner && a->owner) a->owner->OnTriggerStay(a, b);
		if (b->sendEventsToOwner && b->owner) b->owner->OnTriggerStay(b, a);
		a->OnTriggerStay(b);
		b->OnTriggerStay(a);
	}
	else {
		if (a->sendEventsToOwner && a->owner) a->owner->OnCollisionStay(a, b);
		if (b->sendEventsToOwner && b->owner) b->owner->OnCollisionStay(b, a);
		a->OnCollisionStay(b);
		b->OnCollisionStay(a);
	}
}

void ColliderManager::DispatchExit(Collider* a, Collider* b) {
	if (!a || !b) return;
	if (a->isTrigger || b->isTrigger) {
		if (a->sendEventsToOwner && a->owner) a->owner->OnTriggerExit(a, b);
		if (b->sendEventsToOwner && b->owner) b->owner->OnTriggerExit(b, a);
		a->OnTriggerExit(b);
		b->OnTriggerExit(a);
	}
	else {
		if (a->sendEventsToOwner && a->owner) a->owner->OnCollisionExit(a, b);
		if (b->sendEventsToOwner && b->owner) b->owner->OnCollisionExit(b, a);
		a->OnCollisionExit(b);
		b->OnCollisionExit(a);
	}
}

// コライダー種別ペアチェック
static bool IsPair(Collider::Kind a, Collider::Kind b, Collider::Kind x, Collider::Kind y) {
	return (a == x && b == y) || (a == y && b == x);
}

//形状更新
void ColliderManager::UpdateAllShapes() {
	for (auto* c : _colliders) {
		if (!c) continue;
		c->UpdateShape();
	}
}

// ペア構築
void ColliderManager::BuildCurrentPairs() {
	_currPairs.clear();
	_contacts.clear();
	SpatialPartitioning();
}

// Broad-phase（空間分割）
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

	//1) AABB範囲（スイープ：前フレームAABBと共に）をセルに登録
	for (auto* c : _colliders) {
		if (!PassCommonFilters(c)) continue;

		const AABB curr = c->GetAABB();
		AABB sweep = curr;

		// Decide whether to use swept AABB:
		// - if collider explicitly enableCCD
		// - or if per-frame center displacement (speed) exceeds Collider::ccdDistanceThreshold
		auto it = _prevAABBs.find(c);
		bool useSweep = false;
		if (c->enableCCD) {
			useSweep = true;
		}
		else if (it != _prevAABBs.end()) {
			// previous and current centers
			const VECTOR prevCenter = VGet((it->second.min.x + it->second.max.x) * 0.5f,
								(it->second.min.y + it->second.max.y) * 0.5f,
								(it->second.min.z + it->second.max.z) * 0.5f);
			const VECTOR currCenter = VGet((curr.min.x + curr.max.x) * 0.5f,
								(curr.min.y + curr.max.y) * 0.5f,
								(curr.min.z + curr.max.z) * 0.5f);
			const VECTOR d = VSub(currCenter, prevCenter);
			const float distSq = LenSq(d);
			const float thr = c->ccdDistanceThreshold; // interpreted as speed threshold (units/sec)

			// compute per-frame speed (units/sec) using delta time
			double dt_d = Time::Instance().GetDeltaTime();
			float dt = static_cast<float>(dt_d);
			const float minDt = 1e-6f;
			if (dt < minDt) dt = minDt;
			const float speedSq = distSq / (dt * dt);
			if (speedSq > thr * thr) useSweep = true;
		}

		if (useSweep && it != _prevAABBs.end()) {
			// union previous and current to form swept AABB
			sweep.min.x = (std::min)(sweep.min.x, it->second.min.x);
			sweep.min.y = (std::min)(sweep.min.y, it->second.min.y);
			sweep.min.z = (std::min)(sweep.min.z, it->second.min.z);
			sweep.max.x = (std::max)(sweep.max.x, it->second.max.x);
			sweep.max.y = (std::max)(sweep.max.y, it->second.max.y);
			sweep.max.z = (std::max)(sweep.max.z, it->second.max.z);
		}

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

	//2) セル内のペア生成（重複は_currPairsでガード）
	for (auto& [cell, cellCols] : grid) {
		const size_t n = cellCols.size();
		for (size_t i =0; i < n; ++i) {
			Collider* a = cellCols[i];
			for (size_t j = i +1; j < n; ++j) {
				Collider* b = cellCols[j];

				const auto key = MakeKey(a, b);
				if (_currPairs.contains(key)) continue;

				// Layer/Mask
				if (CheckLayerMaskCollisions(a, b)) continue;

				// AABB
				if (CheckAABBCollisions(a, b)) continue;

				// 詳細判定
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

				// 押し戻し（Triggerは除外）
				if (!(a->isTrigger || b->isTrigger)) {
					ResolvePushOut(a, b);
				}
			}
		}
	}
}

// イベント処理
void ColliderManager::ProcessPairEvents() {
	// Enter/Stay
	for (const auto& k : _currPairs) {
		if (_prevPairs.contains(k)) {
			DispatchStay(k.a, k.b);
		}
		else {
			DispatchEnter(k.a, k.b);
		}
	}

	// Exit
	for (const auto& k : _prevPairs) {
		if (!_currPairs.contains(k)) {
			DispatchExit(k.a, k.b);
		}
	}

	_prevPairs = _currPairs;
}

// 詳細判定
void ColliderManager::CheckDetailedCollisions() {
	BuildCurrentPairs();
	ProcessPairEvents();

	// 更新が終わったら次フレーム用に現在のAABBを保存しておく（CCD用）
	for (auto* c : _colliders) {
		if (!c) continue;
		_prevAABBs[c] = c->GetAABB();
	}
}

// 押し戻し
void ColliderManager::ResolvePushOut(Collider* a, Collider* b) {
	if (!a || !b) return;
	if (a->isTrigger || b->isTrigger) return;

	GameObject* oa = a->owner;
	GameObject* ob = b->owner;

	//どちらも owner 無しなら何もしない
	if (!oa && !ob) return;

	// --- Extensible fixed rule ---
	const bool aFixed = (!oa) || (oa && oa->isStatic);
	const bool bFixed = (!ob) || (ob && ob->isStatic);

	if (aFixed && bFixed) {
		return;
	}

	// 両方可動なら従来（重み付きで両方動かす）
	const auto ka = a->GetKind();
	const auto kb = b->GetKind();
	if (!aFixed && !bFixed) {
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
		return;
	}

	// ----片側固定: 可動側だけ動かす ----
	GameObject* movable = nullptr;
	Collider* movableCol = nullptr;
	Collider* fixedCol = nullptr;
	const bool movableIsB = aFixed; // aが固定ならbが可動

	if (aFixed) {
		movable = ob;
		movableCol = b;
		fixedCol = a;
	}
	else if (bFixed) {
		movable = oa;
		movableCol = a;
		fixedCol = b;
	}
	if (!movable || !movableCol || !fixedCol) return;

	// AABB の重なりから最小押し戻し軸(MTV)を選ぶ
	const AABB& A = movableCol->GetAABB();
	const AABB& B = fixedCol->GetAABB();
	const float ox = (std::min)(A.max.x, B.max.x) - (std::max)(A.min.x, B.min.x);
	const float oy = (std::min)(A.max.y, B.max.y) - (std::max)(A.min.y, B.min.y);
	const float oz = (std::min)(A.max.z, B.max.z) - (std::max)(A.min.z, B.min.z);
	if (ox <=0.0f || oy <=0.0f || oz <=0.0f) return;

	float pen = ox;
	VECTOR n = VGet((movableCol->GetCenter().x >= fixedCol->GetCenter().x) ?1.0f : -1.0f,0,0);
	if (oy < pen) {
		pen = oy;
		n = VGet(0, (movableCol->GetCenter().y >= fixedCol->GetCenter().y) ?1.0f : -1.0f,0);
	}
	if (oz < pen) {
		pen = oz;
		n = VGet(0,0, (movableCol->GetCenter().z >= fixedCol->GetCenter().z) ?1.0f : -1.0f);
	}

	// record contact: fixedCol -> movableCol (normal points from fixed to movable)
	Contact ct;
	ct.a = fixedCol;
	ct.b = movableCol;
	ct.normal = n;
	ct.penetration = pen;
	_contacts.push_back(ct);

	VECTOR p = movable->transform.LocalPosition();
	p = VAdd(p, VScale(n, pen));
	movable->transform.SetLocalPosition(p);

	movableCol->UpdateShape();
	fixedCol->UpdateShape();
}

// 各種押し戻し処理関数群

// Sphere-Sphere 押し戻し
void ColliderManager::PushOutSphereSphere(Collider* a, Collider* b) {
	auto* sa = dynamic_cast<SphereCollider*>(a);
	auto* sb = dynamic_cast<SphereCollider*>(b);
	if (!sa || !sb) return;

	GameObject* oa = static_cast<Collider*>(sa)->owner;
	GameObject* ob = static_cast<Collider*>(sb)->owner;
	if (!oa && !ob) return;

	const VECTOR ca = sa->_sphere.center;
	const VECTOR cb = sb->_sphere.center;
	const VECTOR d = VSub(cb, ca);
	const float dist = std::sqrt((std::max)(LenSq(d), 1e-8f));
	const float r = sa->_sphere.radius + sb->_sphere.radius;
	const float pen = r - dist;
	if (pen <= 0.0f) return;

	const VECTOR n = VScale(d, 1.0f / dist);

	Contact ct;
	ct.a = sa;
	ct.b = sb;
	ct.normal = n;
	ct.penetration = pen;
	_contacts.push_back(ct);

	const float wA = oa ? 1.0f : 0.0f;
	const float wB = ob ? 1.0f : 0.0f;
	const float wSum = wA + wB;
	if (wSum <= 0.0f) return;

	const float moveA = (wA / wSum) * pen;
	const float moveB = (wB / wSum) * pen;

	if (oa) {
		VECTOR p = oa->transform.LocalPosition();
		p = VSub(p, VScale(n, moveA));
		oa->transform.SetLocalPosition(p);
	}
	if (ob) {
		VECTOR p = ob->transform.LocalPosition();
		p = VAdd(p, VScale(n, moveB));
		ob->transform.SetLocalPosition(p);
	}

	sa->UpdateShape();
	sb->UpdateShape();
}
// Sphere-Box 押し戻し
void ColliderManager::PushOutSphereBox(Collider* a, Collider* b) {
	SphereCollider* s = dynamic_cast<SphereCollider*>(a);
	BoxCollider* box = dynamic_cast<BoxCollider*>(b);
	if (!s || !box) {
		s = dynamic_cast<SphereCollider*>(b);
		box = dynamic_cast<BoxCollider*>(a);
	}
	if (!s || !box) return;

	GameObject* os = static_cast<Collider*>(s)->owner;
	if (!os) return;

	// 最近点
	const VECTOR c = s->GetCenter();
	const VECTOR d = VSub(c, box->_box.center);
	float x = Dot3(d, box->_box.axisX);
	float y = Dot3(d, box->_box.axisY);
	float z = Dot3(d, box->_box.axisZ);
	x = std::clamp(x, -box->_box.halfExtents.x, box->_box.halfExtents.x);
	y = std::clamp(y, -box->_box.halfExtents.y, box->_box.halfExtents.y);
	z = std::clamp(z, -box->_box.halfExtents.z, box->_box.halfExtents.z);
	VECTOR closest = box->_box.center;
	closest = VAdd(closest, VScale(box->_box.axisX, x));
	closest = VAdd(closest, VScale(box->_box.axisY, y));
	closest = VAdd(closest, VScale(box->_box.axisZ, z));

	VECTOR diff = VSub(c, closest);
	float dist = std::sqrt((std::max)(LenSq(diff), 1e-8f));
	float pen = s->_sphere.radius - dist;
	if (pen <= 0.0f) return;

	VECTOR n = VScale(diff, 1.0f / dist);

	// record contact (sphere -> box)
	Contact ct;
	ct.a = s;
	ct.b = box;
	ct.normal = n;
	ct.penetration = pen;
	_contacts.push_back(ct);

	VECTOR p = os->transform.LocalPosition();
	p = VAdd(p, VScale(n, pen));
	os->transform.SetLocalPosition(p);

	s->UpdateShape();
}
// Box-Box 押し戻し
void ColliderManager::PushOutBoxBox(Collider* a, Collider* b) {
	auto* ba = dynamic_cast<BoxCollider*>(a);
	auto* bb = dynamic_cast<BoxCollider*>(b);
	if (!ba || !bb) return;

	GameObject* oa = ba->owner;
	GameObject* ob = bb->owner;
	if (!oa && !ob) return;

	// OBB vs OBB: SAT の最小貫通軸(MTV)を使って押し戻し（AABB近似を廃止）
	const VECTOR A0 = SafeNorm(ba->_box.axisX, VGet(1,0,0));
	const VECTOR A1 = SafeNorm(ba->_box.axisY, VGet(0,1,0));
	const VECTOR A2 = SafeNorm(ba->_box.axisZ, VGet(0,0,1));
	const VECTOR B0 = SafeNorm(bb->_box.axisX, VGet(1,0,0));
	const VECTOR B1 = SafeNorm(bb->_box.axisY, VGet(0,1,0));
	const VECTOR B2 = SafeNorm(bb->_box.axisZ, VGet(0,0,1));
	const float aExt[3] = { ba->_box.halfExtents.x, ba->_box.halfExtents.y, ba->_box.halfExtents.z };
	const float bExt[3] = { bb->_box.halfExtents.x, bb->_box.halfExtents.y, bb->_box.halfExtents.z };

	const VECTOR tV = VSub(bb->_box.center, ba->_box.center);
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

	auto ConsiderAxis = [&](const VECTOR& axisW, float dist, float ra, float rb) {
		// dist = |t・axis|, pen = (ra+rb) - dist
		const float sep = (ra + rb) - dist;
		if (sep < bestPen) {
			bestPen = sep;
			bestAxisW = axisW;
		}
	};

	// A0,A1,A2
	for (int i =0; i <3; ++i) {
		float ra = aExt[i];
		float rb = bExt[0] * AbsR[i][0] + bExt[1] * AbsR[i][1] + bExt[2] * AbsR[i][2];
		float dist = std::fabs(tA[i]);
		ConsiderAxis((i ==0) ? A0 : (i ==1) ? A1 : A2, dist, ra, rb);
	}

	// B0,B1,B2
	for (int j =0; j <3; ++j) {
		float ra = aExt[0] * AbsR[0][j] + aExt[1] * AbsR[1][j] + aExt[2] * AbsR[2][j];
		float rb = bExt[j];
		float dist = std::fabs(tA[0] * R[0][j] + tA[1] * R[1][j] + tA[2] * R[2][j]);
		ConsiderAxis((j ==0) ? B0 : (j ==1) ? B1 : B2, dist, ra, rb);
	}

	// A_i x B_j
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
			ConsiderAxis(ax, dist, ra, rb);
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
			ConsiderAxis(ax, dist, ra, rb);
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
			ConsiderAxis(ax, dist, ra, rb);
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
			ConsiderAxis(ax, dist, ra, rb);
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
			ConsiderAxis(ax, dist, ra, rb);
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
			ConsiderAxis(ax, dist, ra, rb);
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
			ConsiderAxis(ax, dist, ra, rb);
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
			ConsiderAxis(ax, dist, ra, rb);
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
			ConsiderAxis(ax, dist, ra, rb);
		}
	}

	if (bestPen == FLT_MAX || bestPen <=0.0f) return;

	// 押し出し方向の符号を揃える（A -> B方向に押し出す）
	VECTOR n = bestAxisW;
	if (Dot3(tV, n) <0.0f) {
		n = VScale(n, -1.0f);
	}
	const float pen = bestPen;

	// record contact (ba -> bb)
	Contact ct;
	ct.a = ba;
	ct.b = bb;
	ct.normal = n;
	ct.penetration = pen;
	_contacts.push_back(ct);

	const float wA = oa ?1.0f :0.0f;
	const float wB = ob ?1.0f :0.0f;
	const float wSum = wA + wB;
	if (wSum <=0.0f) return;

	const float moveA = (wA / wSum) * pen;
	const float moveB = (wB / wSum) * pen;

	if (oa) {
		VECTOR p = oa->transform.LocalPosition();
		p = VSub(p, VScale(n, moveA));
		oa->transform.SetLocalPosition(p);
	}
	if (ob) {
		VECTOR p = ob->transform.LocalPosition();
		p = VAdd(p, VScale(n, moveB));
		ob->transform.SetLocalPosition(p);
	}

	ba->UpdateShape();
	bb->UpdateShape();
}
// Capsule-Capsule 押し戻し
void ColliderManager::PushOutCapsuleCapsule(Collider* a, Collider* b) {
	auto* ca = dynamic_cast<CapsuleCollider*>(a);
	auto* cb = dynamic_cast<CapsuleCollider*>(b);
	if (!ca || !cb) return;

	GameObject* oa = ca->owner;
	GameObject* ob = cb->owner;
	if (!oa && !ob) return;

	const VECTOR p1 = ca->_cap.bottom;
	const VECTOR q1 = ca->_cap.top;
	const VECTOR p2 = cb->_cap.bottom;
	const VECTOR q2 = cb->_cap.top;

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
	const float r = ca->_cap.radius + cb->_cap.radius;
	const float pen = r - dist;
	if (pen <= 0.0f) return;

	const VECTOR n = VScale(diff, 1.0f / dist);

	Contact ct;
	ct.a = ca;
	ct.b = cb;
	ct.normal = n;
	ct.penetration = pen;
	_contacts.push_back(ct);

	const float wA = oa ? 1.0f : 0.0f;
	const float wB = ob ? 1.0f : 0.0f;
	const float wSum = wA + wB;
	if (wSum <= 0.0f) return;

	const float moveA = (wA / wSum) * pen;
	const float moveB = (wB / wSum) * pen;

	if (oa) {
		VECTOR p = oa->transform.LocalPosition();
		p = VAdd(p, VScale(n, moveA));
		oa->transform.SetLocalPosition(p);
	}
	if (ob) {
		VECTOR p = ob->transform.LocalPosition();
		p = VSub(p, VScale(n, moveB));
		ob->transform.SetLocalPosition(p);
	}

	ca->UpdateShape();
	cb->UpdateShape();
}
// Sphere-Capsule 押し戻し
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

	const VECTOR p = c->_cap.bottom;
	const VECTOR q = c->_cap.top;
	const VECTOR seg = VSub(q, p);
	const VECTOR v = VSub(s->_sphere.center, p);

	const float segLenSq = Dot3(seg, seg);
	float t =0.0f;
	if (segLenSq >1e-6f) {
		t = Dot3(v, seg) / segLenSq;
		t = std::clamp(t,0.0f,1.0f);
	}

	const VECTOR closest = VAdd(p, VScale(seg, t));
	VECTOR diff = VSub(s->_sphere.center, closest);
	const float dist = std::sqrt((std::max)(LenSq(diff), 1e-8f));
	const float r = s->_sphere.radius + c->_cap.radius;
	const float pen = r - dist;
	if (pen <= 0.0f) return;

	VECTOR n = VScale(diff, 1.0f / dist);

	Contact ct;
	ct.a = s;
	ct.b = c;
	ct.normal = n;
	ct.penetration = pen;
	_contacts.push_back(ct);

	const float wS = os ? 1.0f : 0.0f;
	const float wC = oc ? 1.0f : 0.0f;
	const float wSum = wS + wC;
	if (wSum <= 0.0f) return;

	const float moveS = (wS / wSum) * pen;
	const float moveC = (wC / wSum) * pen;

	if (os) {
		VECTOR p0 = os->transform.LocalPosition();
		p0 = VAdd(p0, VScale(n, moveS));
		os->transform.SetLocalPosition(p0);
	}
	if (oc) {
		VECTOR p0 = oc->transform.LocalPosition();
		p0 = VSub(p0, VScale(n, moveC));
		oc->transform.SetLocalPosition(p0);
	}

	s->UpdateShape();
	c->UpdateShape();
}
// Box-Capsule 押し戻し
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

	// OBBローカルへ: sphere-box の最近点計算を流用するため、カプセル端点をローカル化
	auto ToLocal = [&](const VECTOR& w) {
		const VECTOR d = VSub(w, box->_box.center);
		return VGet(Dot3(d, box->_box.axisX), Dot3(d, box->_box.axisY), Dot3(d, box->_box.axisZ));
	 };
	auto ToWorld = [&](const VECTOR& l) {
		VECTOR w = box->_box.center;
		w = VAdd(w, VScale(box->_box.axisX, l.x));
		w = VAdd(w, VScale(box->_box.axisY, l.y));
		w = VAdd(w, VScale(box->_box.axisZ, l.z));
		return w;
	 };

	const VECTOR pL = ToLocal(cap->_cap.bottom);
	const VECTOR qL = ToLocal(cap->_cap.top);
	const VECTOR dL = VSub(qL, pL);

	const float hx = box->_box.halfExtents.x;
	const float hy = box->_box.halfExtents.y;
	const float hz = box->_box.halfExtents.z;

	auto ClampPointToAABB = [&](const VECTOR& p) {
		return VGet(
			std::clamp(p.x, -hx, hx),
			std::clamp(p.y, -hy, hy),
			std::clamp(p.z, -hz, hz)
		);
	};

	// 候補点（端点 + スラブ境界）から、最近点ペアを得る
	float best = FLT_MAX;
	VECTOR bestSegPointL = pL;
	VECTOR bestBoxPointL = ClampPointToAABB(pL);

	auto ConsiderT = [&](float t) {
		if (t < 0.0f || t >1.0f) return;
		const VECTOR sL = VAdd(pL, VScale(dL, t));
		const VECTOR cL = ClampPointToAABB(sL);
		const float dsq = LenSq(VSub(sL, cL));
		if (dsq < best) {
			best = dsq;
			bestSegPointL = sL;
			bestBoxPointL = cL;
		}
	};

	//端点
	ConsiderT(0.0f);
	ConsiderT(1.0f);

	// スラブ境界
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

	const float dist = std::sqrt((std::max)(best, 1e-8f));
	const float pen = cap->_cap.radius - dist;
	if (pen <= 0.0f) return;

	// ローカルでの法線（箱の最近点 -> 線分最近点）をワールドへ
	VECTOR nL = VSub(bestSegPointL, bestBoxPointL);
	const float nLen = std::sqrt((std::max)(LenSq(nL), 1e-8f));
	nL = VScale(nL, 1.0f / nLen);

	// ローカル法線をワールドへ（OBB軸で合成）
	VECTOR nW = VGet(0, 0, 0);
	nW = VAdd(nW, VScale(box->_box.axisX, nL.x));
	nW = VAdd(nW, VScale(box->_box.axisY, nL.y));
	nW = VAdd(nW, VScale(box->_box.axisZ, nL.z));
	const float nWLen = std::sqrt((std::max)(LenSq(nW), 1e-8f));
	nW = VScale(nW, 1.0f / nWLen);

	const float wBox = obox ? 1.0f : 0.0f;
	const float wCap = ocap ? 1.0f : 0.0f;
	const float wSum = wBox + wCap;
	if (wSum <= 0.0f) return;

	const float moveBox = (wBox / wSum) * pen;
	const float moveCap = (wCap / wSum) * pen;

	// cap を +nW, box を -nW に動かす
	if (ocap) {
		VECTOR p0 = ocap->transform.LocalPosition();
		p0 = VAdd(p0, VScale(nW, moveCap));
		ocap->transform.SetLocalPosition(p0);
	}
	if (obox) {
		VECTOR p0 = obox->transform.LocalPosition();
		p0 = VSub(p0, VScale(nW, moveBox));
		obox->transform.SetLocalPosition(p0);
	}

	box->UpdateShape();
	cap->UpdateShape();
}

// 更新
void ColliderManager::Update() {
	if (IsShuttingDown()) {
		return;
	}
	UpdateAllShapes();
	CheckDetailedCollisions();
}

// Sphere-Sphere 当たり判定
void ColliderManager::CheckSphereSphere(Collider* a, Collider* b) {
	auto* sa = dynamic_cast<SphereCollider*>(a);
	auto* sb = dynamic_cast<SphereCollider*>(b);
	if (!sa || !sb) {
		_narrowHit = false;
		return;
	}
	const VECTOR ca = sa->_sphere.center;
	const VECTOR cb = sb->_sphere.center;
	const VECTOR d = VSub(cb, ca);
	const float r = sa->_sphere.radius + sb->_sphere.radius;
	_narrowHit = (LenSq(d) <= r * r);
}

// Sphere-Box(OBB) 当たり判定
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

	const VECTOR c = s->_sphere.center;
	const float r = s->_sphere.radius;
	const VECTOR d = VSub(c, box->_box.center);
	float x = Dot3(d, box->_box.axisX);
	float y = Dot3(d, box->_box.axisY);
	float z = Dot3(d, box->_box.axisZ);

	x = std::clamp(x, -box->_box.halfExtents.x, box->_box.halfExtents.x);
	y = std::clamp(y, -box->_box.halfExtents.y, box->_box.halfExtents.y);
	z = std::clamp(z, -box->_box.halfExtents.z, box->_box.halfExtents.z);

	VECTOR closest = box->_box.center;
	closest = VAdd(closest, VScale(box->_box.axisX, x));
	closest = VAdd(closest, VScale(box->_box.axisY, y));
	closest = VAdd(closest, VScale(box->_box.axisZ, z));

	const VECTOR diff = VSub(c, closest);
	_narrowHit = (LenSq(diff) <= r * r);
}

// Box-Box(OBB) 当たり判定
void ColliderManager::CheckBoxBox(Collider* a, Collider* b) {
	BoxCollider* ba = dynamic_cast<BoxCollider*>(a);
	BoxCollider* bb = dynamic_cast<BoxCollider*>(b);
	if (!ba || !bb) {
		_narrowHit = false;
		return;
	}

	// SAT: OBB vs OBB
	const VECTOR A0 = ba->_box.axisX;
	const VECTOR A1 = ba->_box.axisY;
	const VECTOR A2 = ba->_box.axisZ;
	const VECTOR B0 = bb->_box.axisX;
	const VECTOR B1 = bb->_box.axisY;
	const VECTOR B2 = bb->_box.axisZ;

	const VECTOR tV = VSub(bb->_box.center, ba->_box.center);
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

	const float aExt[3] = { ba->_box.halfExtents.x, ba->_box.halfExtents.y, ba->_box.halfExtents.z };
	const float bExt[3] = { bb->_box.halfExtents.x, bb->_box.halfExtents.y, bb->_box.halfExtents.z };

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

	// A_i x B_j
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

// Capsule-Capsule 当たり判定
void ColliderManager::CheckCapsuleCapsule(Collider* a, Collider* b) {
	auto* ca = dynamic_cast<CapsuleCollider*>(a);
	auto* cb = dynamic_cast<CapsuleCollider*>(b);
	if (!ca || !cb) {
		_narrowHit = false;
		return;
	}

	const VECTOR p1 = ca->_cap.bottom;
	const VECTOR q1 = ca->_cap.top;
	const VECTOR p2 = cb->_cap.bottom;
	const VECTOR q2 = cb->_cap.top;
	const float r = ca->_cap.radius + cb->_cap.radius;

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

// Sphere-Capsule 当たり判定
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

	const VECTOR p = c->_cap.bottom;
	const VECTOR q = c->_cap.top;
	const VECTOR seg = VSub(q, p);
	const VECTOR v = VSub(s->_sphere.center, p);

	const float segLenSq = Dot3(seg, seg);
	float t =0.0f;
	if (segLenSq >1e-6f) {
		t = Dot3(v, seg) / segLenSq;
		t = std::clamp(t,0.0f,1.0f);
	}

	const VECTOR closest = VAdd(p, VScale(seg, t));
	const VECTOR diff = VSub(s->_sphere.center, closest);
	const float r = s->_sphere.radius + c->_cap.radius;
	_narrowHit = (LenSq(diff) <= r * r);
}

// Box(OBB)-Capsule 当たり判定
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

	// OBBローカルへ: sphere-box の最近点計算を流用するため、カプセル端点をローカル化
	auto ToLocal = [&](const VECTOR& w) {
		const VECTOR d = VSub(w, box->_box.center);
		return VGet(Dot3(d, box->_box.axisX), Dot3(d, box->_box.axisY), Dot3(d, box->_box.axisZ));
	 };
	auto ToWorld = [&](const VECTOR& l) {
		VECTOR w = box->_box.center;
		w = VAdd(w, VScale(box->_box.axisX, l.x));
		w = VAdd(w, VScale(box->_box.axisY, l.y));
		w = VAdd(w, VScale(box->_box.axisZ, l.z));
		return w;
	 };

	const VECTOR pL = ToLocal(cap->_cap.bottom);
	const VECTOR qL = ToLocal(cap->_cap.top);
	const VECTOR dL = VSub(qL, pL);

	const float hx = box->_box.halfExtents.x;
	const float hy = box->_box.halfExtents.y;
	const float hz = box->_box.halfExtents.z;

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

	const float r = cap->_cap.radius;
	_narrowHit = (best <= r * r);
}

// デバッグ描画
void ColliderManager::DrawDebugAll() {
	for (auto* collider : _colliders) {
		if (!collider) continue;
		collider->DrawDebug();
	}
}

void ColliderManager::DrawDebugAABBAll() {
	for (auto* collider : _colliders) {
		if (!collider) continue;
		collider->DrawDebugAABB();
	}
}

// Collider登録
void ColliderManager::RegisterCollider(Collider* collider) {
	if (IsShuttingDown()) {
		return;
	}
	if (!collider) return;
	if (std::find(_colliders.begin(), _colliders.end(), collider) != _colliders.end()) return;
	_colliders.push_back(collider);
}

// Collider解除
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

// Layer/Mask判定
bool ColliderManager::CheckLayerMaskCollisions(Collider* a, Collider* b) {
	if ((a->layer & b->mask) ==0) return true;
	if ((b->layer & a->mask) ==0) return true;
	return false;
}

// AABB判定
bool ColliderManager::CheckAABBCollisions(Collider* a, Collider* b) {
	return !IntersectAABBWorld(a->GetAABB(), b->GetAABB());
}


