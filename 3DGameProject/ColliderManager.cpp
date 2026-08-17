#include "ColliderManager.h"
#include "GameObject.h"
#include "SceneManager.h"
#include "SphereCollider.h"
#include "BoxCollider.h"
#include "CapsuleCollider.h"
#include "HalfPlaneCollider.h"
#include "CompoundCollider.h"
#include "MeshCollider.h"
#include "PhysicsDebugClass.h"
#include "Assert.h"
#include "ThreadPool.h"
#include "PerformanceMonitor.h"
#include "BitOperation.h"
#include <algorithm>
#include <cmath>
#include <cfloat>

// --- 静的メンバ定義 ---
thread_local bool                  ColliderManager::_tlNarrowHit   = false;
thread_local std::vector<ColliderManager::Contact>* ColliderManager::_tlContactOut = nullptr;

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

	// OBBローカル空間のベクトルをワールド空間に変換。
	inline VECTOR FromObbLocalVector(const VECTOR& localVector, const BoxCollider* box) noexcept {
		VECTOR v = VGet(0, 0, 0);
		v = VAdd(v, VScale(box->GetAxisX(), localVector.x));
		v = VAdd(v, VScale(box->GetAxisY(), localVector.y));
		v = VAdd(v, VScale(box->GetAxisZ(), localVector.z));
		return v;
	}

	// 線分（球の中心の移動）と OBB のスイープテスト。
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

	// 線分（Aの中心の移動）と OBB-OBB のスイープテスト。
	inline bool SweepBoxAgainstBox(
		const VECTOR& prevCenterA, const VECTOR& currCenterA,
		const BoxCollider* ba,
		const VECTOR& prevCenterB, const VECTOR& currCenterB,
		const BoxCollider* bb,
		float* outHitT, VECTOR* outNormalWorld) noexcept
	{
		if (!ba || !bb) return false;
		// 相対速度を求める
		const VECTOR moveA = VSub(currCenterA, prevCenterA);
		const VECTOR moveB = VSub(currCenterB, prevCenterB);
		const VECTOR relVel = VSub(moveB, moveA);

		const VECTOR A0 = ba->GetAxisX(), A1 = ba->GetAxisY(), A2 = ba->GetAxisZ();
		const VECTOR B0 = bb->GetAxisX(), B1 = bb->GetAxisY(), B2 = bb->GetAxisZ();
		const float aExt[3] = { ba->GetHalfExtents().x, ba->GetHalfExtents().y, ba->GetHalfExtents().z };
		const float bExt[3] = { bb->GetHalfExtents().x, bb->GetHalfExtents().y, bb->GetHalfExtents().z };
		const VECTOR Axes_A[3] = { A0, A1, A2 };
		const VECTOR Axes_B[3] = { B0, B1, B2 };

		// 初期分離ベクトル（t=0の中心間ベクトル）
		const VECTOR t0Sep = VSub(prevCenterB, prevCenterA);

		float tFirst = 0.0f, tLast = 1.0f;
		VECTOR bestNormal = VGet(0, 1, 0);
		bool bestNormalSet = false;

		auto TestAxis = [&](const VECTOR& axis) -> bool {
			const float axisLen = Len3(axis);
			if (axisLen < 1e-5f) return true; // ほぼゼロベクトルは無視（平行な軸同士のクロスプロダクトがこれに該当する）
			const VECTOR n = VScale(axis, 1.0f / axisLen);

			// 各ボックスの半幅を軸に投影して、分離距離を求める
			float rA = 0.0f, rB = 0.0f;
			for (int i = 0; i < 3; ++i) {
				rA += std::fabs(Dot3(Axes_A[i], n)) * aExt[i];
				rB += std::fabs(Dot3(Axes_B[i], n)) * bExt[i];
			}
			const float d0 = Dot3(t0Sep, n); // 初期分離距離（t=0の中心間ベクトルの投影）
			const float v = Dot3(relVel, n);  // 軸方向の相対速度

			const float s = rA + rB; // 結合された半幅
			// 重なり範囲: d0 + v*t in [-s, s]
			float tEnter, tExit;
			if (std::fabs(v) < 1e-8f) {
				if (d0 < -s || d0 > s) return false; // 分離, この軸上で相対運動なし
				tEnter = 0.0f;
				tExit = 1.0f;
			} else {
				tEnter = (-s - d0) / v;
				tExit = (s - d0) / v;
				if (tEnter > tExit) std::swap(tEnter, tExit);
			}

			if (tEnter > tFirst) {
				tFirst = tEnter;
				bestNormal = (d0 > 0.0f) ? n : VScale(n, -1.0f);
				bestNormalSet = true;
			}
			tLast = (std::min)(tLast, tExit);

			return tFirst <= tLast;
		};

		// 6つの面法線をテスト
		for (int i = 0; i < 3; ++i) { if (!TestAxis(Axes_A[i])) return false; }
		for (int i = 0; i < 3; ++i) { if (!TestAxis(Axes_B[i])) return false; }
		// 9つのエッジ-エッジのクロス積をテスト
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				if (!TestAxis(VCross(Axes_A[i], Axes_B[j]))) return false;
			}
		}

		if (tFirst < 0.0f || tFirst > 1.0f) return false;
		if (!bestNormalSet) return false;

		if (outHitT) *outHitT = tFirst;
		if (outNormalWorld) *outNormalWorld = bestNormal;
		return true;
	}

	// 球の回転による見かけの拡大を計算する。
	inline float ComputeAngularExpansion(const VECTOR& angVel, float halfExtent, float dt) noexcept {
		const float angSpeed = Len3(angVel);
		// 角速度が速いほど、半径方向の拡大が大きくなる。dt を掛けることで、フレーム時間に応じた拡大量になる。
		return angSpeed * halfExtent * dt;
	}

	// 線分（球の中心の移動）と OBB のスイープテスト。角速度による見かけの拡大を考慮して、より正確な衝突時間と法線を求める。
	inline bool SweepSphereAgainstBoxRefined(
		const VECTOR& prevCenter,
		const VECTOR& currCenter,
		float radius,
		const BoxCollider* box,
		float* outHitT,
		VECTOR* outNormalWorld,
		VECTOR* outHitCenterWorld) noexcept
	{
		if (!box) return false;

		// 第一パス: 面ベースのスイープテストで大まかな衝突時間と法線を求める
		const VECTOR p0 = ToObbLocal(prevCenter, box);
		const VECTOR p1 = ToObbLocal(currCenter, box);
		const VECTOR d = VSub(p1, p0);
		const VECTOR he = box->GetHalfExtents();
		const VECTOR e = VAdd(he, VGet(radius, radius, radius));

		float tMin = 0.0f;
		float tMax = 1.0f;
		VECTOR hitNormalLocal = VGet(0, 0, 0);
		int hitAxis = -1;
		const float p0Arr[3] = { p0.x, p0.y, p0.z };
		const float dArr[3] = { d.x, d.y, d.z };
		const float eArr[3] = { e.x, e.y, e.z };

		for (int axis = 0; axis < 3; ++axis) {
			const float origin = p0Arr[axis];
			const float dir = dArr[axis];
			const float extent = eArr[axis];

			if (std::fabs(dir) < 1e-6f) {
				if (origin < -extent || origin > extent) return false;
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
				hitAxis = axis;
			}
			tMax = (std::min)(tMax, t2);
			if (tMin > tMax) return false;
		}

		if (tMin < 0.0f || tMin > 1.0f) return false;

		// 第二パス: コーナー/エッジの補正
		// 衝突時刻での球の中心を計算
		const VECTOR hitLocal = VAdd(p0, VScale(d, tMin));
		const float heArr[3] = { he.x, he.y, he.z };
		const float hitArr[3] = { hitLocal.x, hitLocal.y, hitLocal.z };

		// 衝突点をOBBの中心から見たときの座標を、OBBの半サイズでクランプして、最も近いOBB表面上の点を求める。
		float closest[3];
		int outsideAxes = 0;
		for (int i = 0; i < 3; ++i) {
			closest[i] = std::clamp(hitArr[i], -heArr[i], heArr[i]);
			if (std::fabs(hitArr[i]) > heArr[i] + 1e-6f) ++outsideAxes;
		}

		if (outsideAxes >= 2) {
			// 2軸以上でOBBの外に出ている場合は、コーナー/エッジの法線を採用する。
			const VECTOR closestPt = VGet(closest[0], closest[1], closest[2]);
			const VECTOR diff = VSub(hitLocal, closestPt);
			const float diffLen = Len3(diff);
			if (diffLen > 1e-6f) {
				// 面ベースの法線が、コーナー/エッジの法線に比べて大きく外れている場合は、コーナー/エッジの法線を採用する。
				if (diffLen <= radius + 0.1f) {
					hitNormalLocal = VScale(diff, 1.0f / diffLen);
				}
			}
		}

		if (outHitT) *outHitT = tMin;
		if (outNormalWorld) *outNormalWorld = SafeNorm(FromObbLocalVector(hitNormalLocal, box), VGet(1, 0, 0));
		if (outHitCenterWorld) *outHitCenterWorld = VAdd(prevCenter, VScale(VSub(currCenter, prevCenter), tMin));
		return true;
	}
}

// 明示終了。
// 終了中に Unregister が走っても安全なように内部コンテナを空にする。
void ColliderManager::Shutdown() noexcept {
	const bool wasShuttingDown = _shuttingDown.exchange(true, std::memory_order_relaxed);
	if (wasShuttingDown) {
		return;
	}

	std::lock_guard lk(_mtx);
	_currPairs.clear();
	_prevPairs.clear();
	_colliders.clear();
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
	if (!collider->hasPrevAABB) {
		return curr;
	}
	const AABB& prev = collider->prevAABB;

	bool useSweep = collider->enableCCD;
	if (!useSweep) {
		// enableCCD が明示されていない場合でも、
		// フレーム間速度が閾値を超えたら sweep を有効化する。
		// speed^2 = distance^2 / dt^2 で sqrt を避けて比較している。
		const VECTOR prevCenter = VGet((prev.min.x + prev.max.x) * 0.5f,
			(prev.min.y + prev.max.y) * 0.5f,
			(prev.min.z + prev.max.z) * 0.5f);
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
	sweep.min.x = (std::min)(sweep.min.x, prev.min.x);
	sweep.min.y = (std::min)(sweep.min.y, prev.min.y);
	sweep.min.z = (std::min)(sweep.min.z, prev.min.z);
	sweep.max.x = (std::max)(sweep.max.x, prev.max.x);
	sweep.max.y = (std::max)(sweep.max.y, prev.max.y);
	sweep.max.z = (std::max)(sweep.max.z, prev.max.z);

	// 回転による見かけの拡大を考慮するため、もしコライダにオーナーがいて Transform が回転しているなら、
	// AABBの対角線の半分を最大拡大量の目安として、前回AABBと今回AABBのサイズ変化から見かけの拡大を推測し、sweep AABBをさらに拡大する。
	if (collider->owner) {
		// AABBの対角線の半分を最大拡大量の目安とする。
		const float ex = (curr.max.x - curr.min.x) * 0.5f;
		const float ey = (curr.max.y - curr.min.y) * 0.5f;
		const float ez = (curr.max.z - curr.min.z) * 0.5f;
		const float currExtentSq = ex * ex + ey * ey + ez * ez;
		// 前回AABBと今回AABBのサイズ変化から見かけの拡大を推測する。
		const float prevEx = (prev.max.x - prev.min.x) * 0.5f;
		const float prevEy = (prev.max.y - prev.min.y) * 0.5f;
		const float prevEz = (prev.max.z - prev.min.z) * 0.5f;
		const float prevExtentSq = prevEx*prevEx + prevEy*prevEy + prevEz*prevEz;
		const float extentGrowth = std::fabs(std::sqrt(currExtentSq) - std::sqrt(prevExtentSq));
		if (extentGrowth > 0.01f) {
			const float angExpansion = extentGrowth * 1.5f;
			sweep.min.x -= angExpansion;
			sweep.min.y -= angExpansion;
			sweep.min.z -= angExpansion;
			sweep.max.x += angExpansion;
			sweep.max.y += angExpansion;
			sweep.max.z += angExpansion;
		}
	}

	sweep.center = VScale(VAdd(sweep.min, sweep.max), 0.5f);
	return sweep;
}

// 全コライダのワールド形状更新。
// Transform 変更後に narrow-phase へ入る前提をそろえる。
void ColliderManager::UpdateAllShapes() {
   #ifdef _DEBUG
	auto _s = PerformanceMonitor::Instance().Scope("Collider.UpdateAllShapes");
	#endif
	// _colliders のスナップショットをロック下で取得し、
	// ParallelFor はロック外で実行する。
	// Register/Unregister が並行しても _colliders を破壊しない。
	auto& snapshot = _snapshotBuf;
	{
		std::lock_guard lk(_mtx);
		snapshot.assign(_colliders.begin(), _colliders.end());
	}
	const size_t count = snapshot.size();
	if (count == 0) return;

	ThreadPool::Instance().ParallelForBarrier(0, count, [&](size_t i) {
		Collider* c = snapshot[i];
		if (!c) return;
		c->UpdateShape();
	}, 8);
}

// 今フレームの衝突候補を再構築。
void ColliderManager::BuildCurrentPairs() {
 #ifdef _DEBUG
	auto _s = PerformanceMonitor::Instance().Scope("Collider.BuildCurrentPairs");
	#endif
	_currPairs.clear();
	_contacts.clear();
	SpatialPartitioning();
}

// 空間分割による broad-phase。
// swept AABB をセルに登録し、同じセルにいるペアだけを詳細判定に回す。
void ColliderManager::SpatialPartitioning() {
   #ifdef _DEBUG
	auto _sTotal = PerformanceMonitor::Instance().Scope("Collider.SpatialPartitioning");
	#endif
	const int currentSceneId = SceneManager::Instance().CurrentSceneId();

	const float cellSize = (_cellSize > 0.01f) ? _cellSize : 0.01f;
	constexpr int kCellLimit = 100000;
	auto ToCell = [&](float v) -> int {
		if (!std::isfinite(v)) return 0;
		const float c = std::floor(v / cellSize);
		if (c < -static_cast<float>(kCellLimit)) return -kCellLimit;
		if (c >  static_cast<float>(kCellLimit)) return  kCellLimit;
		return static_cast<int>(c);
	};

	// --- フィルタ済みコライダーの収集（ロック下でスナップショット） ---
	auto& active = _activeBuf;
	{
		#ifdef _DEBUG
		auto _s = PerformanceMonitor::Instance().Scope("Collider.ActiveCollect");
		#endif
		std::lock_guard lk(_mtx);
		active.clear();
		active.reserve(_colliders.size());
		for (auto* c : _colliders) {
			if (!c) continue;
			if (!c->owner) { if (c->useSceneFilter) continue; }
			else { if (c->useSceneFilter && c->owner->_ownerSceneId != currentSceneId) continue; }
			if (!c->IsEnabled()) continue;
			if (c->owner && !c->owner->IsActive()) continue;
			active.push_back(c);
		}
	}

 // --- swept AABB のプリコンピュート（並列） ---
	auto& sweptAABBs = _sweptAABBsBuf;
	sweptAABBs.resize(active.size());
	{
		#ifdef _DEBUG
		auto _s = PerformanceMonitor::Instance().Scope("Collider.SweptAABB");
		#endif
		ThreadPool::Instance().ParallelForBarrier(0, active.size(), [&](size_t i) {
			sweptAABBs[i] = GetSweptAABB(active[i]);
		}, 8);
	}

   // --- グリッド構築 ---
	// Phase 1: 各コライダの swept AABB をセルに分解してリスト化（並列で高速化）
	// 大きすぎるAABBは中心点のみで登録して、極端なセル爆発を防止する。
	auto& perColliderCells = _perColliderCellsBuf;
	if (perColliderCells.size() < active.size()) perColliderCells.resize(active.size());
	for (size_t i = 0; i < active.size(); ++i) perColliderCells[i].clear();
	{
		#ifdef _DEBUG
		auto _s = PerformanceMonitor::Instance().Scope("Collider.GridCellCompute");
		#endif
		ThreadPool::Instance().ParallelForBarrier(0, active.size(), [&](size_t i) {
			const AABB& sweep = sweptAABBs[i];
			const int idx = static_cast<int>(i);

			if (!std::isfinite(sweep.min.x) || !std::isfinite(sweep.max.x) ||
				!std::isfinite(sweep.min.y) || !std::isfinite(sweep.max.y) ||
				!std::isfinite(sweep.min.z) || !std::isfinite(sweep.max.z)) {
				perColliderCells[i].push_back({CellKey{0, 0, 0}, idx});
				return;
			}

			const int minX = ToCell(sweep.min.x);
			const int minY = ToCell(sweep.min.y);
			const int minZ = ToCell(sweep.min.z);
			const int maxX = ToCell(sweep.max.x);
			const int maxY = ToCell(sweep.max.y);
			const int maxZ = ToCell(sweep.max.z);

			const int64_t dx = static_cast<int64_t>(maxX) - minX + 1;
			const int64_t dy = static_cast<int64_t>(maxY) - minY + 1;
			const int64_t dz = static_cast<int64_t>(maxZ) - minZ + 1;
			if (dx <= 0 || dy <= 0 || dz <= 0 || dx * dy * dz > 512) {
				const int cx = ToCell(sweep.center.x);
				const int cy = ToCell(sweep.center.y);
				const int cz = ToCell(sweep.center.z);
				perColliderCells[i].push_back({CellKey{cx, cy, cz}, idx});
				return;
			}

			const size_t cellCount = static_cast<size_t>(dx * dy * dz);
			if (perColliderCells[i].capacity() < cellCount)
				perColliderCells[i].reserve(cellCount);
			for (int z = minZ; z <= maxZ; ++z) {
				for (int y = minY; y <= maxY; ++y) {
					for (int x = minX; x <= maxX; ++x) {
						perColliderCells[i].push_back({CellKey{x, y, z}, idx});
					}
				}
			}
		}, 8);
	}

	// Phase 2: コライダインデックスをセルに集約してグリッドを構築（並列化）
	auto& grid = _gridBuf;
	grid.clear();
	{
		#ifdef _DEBUG
		auto _s = PerformanceMonitor::Instance().Scope("Collider.GridMerge");
		#endif
		size_t totalEntries = 0;
		for (size_t i = 0; i < active.size(); ++i) totalEntries += perColliderCells[i].size();
		if (totalEntries > 0 && grid.bucket_count() < totalEntries * 2)
			grid.reserve(totalEntries);

		// グローバル mutex で直列化されるくらいなら、シリアルマージのほうが
		// 結局速い (totalEntries 数千程度ならハッシュ挿入はキャッシュに乗る)。
		// 並列化は perColliderCells / sweptAABBs の事前計算側で十分活かしているため、
		// ここはあえてシングルスレッドで実行する。
		for (size_t i = 0; i < active.size(); ++i) {
			for (const auto& entry : perColliderCells[i]) {
				grid[entry.key].push_back(entry.colliderIdx);
			}
		}
	}

 // --- Step 1: 候補ペア列挙 (broad-phase) ---
	auto& candidates = _candidatesBuf;
	candidates.clear();
	{
		#ifdef _DEBUG
		auto _s = PerformanceMonitor::Instance().Scope("Collider.BroadPhase");
		#endif
		// 同じセルにいるコライダペアを列挙し、AABBの重なりをチェックして候補ペアに追加。
		// 同じペアが複数のセルにまたがって存在する可能性があるため、ペアIDを作って重複を後で削除する。
		// makePairId は lo=min(ai,bi) を上位 32bit、hi=max(ai,bi) を下位 32bit に詰めるので、
		// uint64 を sort+unique するだけで重複排除 + 順序付き ai/bi 復元ができる。
		// これにより、別途 candidates / indices / unique といった一時バッファを持たずに済む。
		auto& seenPairs = _seenPairsBuf;
		seenPairs.clear();
		seenPairs.reserve(active.size() * 2);
		auto makePairId = [](int ai, int bi) -> uint64_t {
			const auto lo = static_cast<uint32_t>((std::min)(ai, bi));
			const auto hi = static_cast<uint32_t>((std::max)(ai, bi));
			return (static_cast<uint64_t>(lo) << 32) | hi;
		};
		// grid (unordered_map) のセルごとに同居コライダのペアを列挙する。
		// 並列化を試したが、(a) セルあたりのペア数が少ない (b) thread_local + mutex の
		// マージコストが勝つ、ため直列のほうが速い。BroadPhase の主コストは
		// CheckLayerMaskCollisions の呼び出しと AABB チェックそのものなので
		// シリアルキャッシュ局所性を活かす。
		for (auto& [cell, cellIndices] : grid) {
			const size_t cn = cellIndices.size();
			for (size_t i = 0; i < cn; ++i) {
				for (size_t j = i + 1; j < cn; ++j) {
					const int ai = cellIndices[i];
					const int bi = cellIndices[j];

					const AABB& sa = sweptAABBs[ai];
					const AABB& sb = sweptAABBs[bi];
					if (sa.min.x > sb.max.x || sa.max.x < sb.min.x) continue;
					if (sa.min.y > sb.max.y || sa.max.y < sb.min.y) continue;
					if (sa.min.z > sb.max.z || sa.max.z < sb.min.z) continue;

					if (CheckLayerMaskCollisions(active[ai], active[bi])) continue;

					seenPairs.push_back(makePairId(ai, bi));
				}
			}
		}
		// sort + unique で重複排除 (一時 vector 不要)。
		if (seenPairs.size() > 1) {
			std::sort(seenPairs.begin(), seenPairs.end());
			seenPairs.erase(std::unique(seenPairs.begin(), seenPairs.end()), seenPairs.end());
		}
		// 重複排除後の pairId から ai/bi を復元。
		candidates.reserve(seenPairs.size());
		for (uint64_t pid : seenPairs) {
			const int ai = static_cast<int>(pid >> 32);
			const int bi = static_cast<int>(pid & 0xFFFFFFFFu);
			candidates.push_back({ai, bi});
		}
	}

	// --- Step 2: ナロウフェーズ判定 (Week 1-2: 並列化) ---

	//  並列化戦略:
	//    Phase A (Parallel): 各ペアの当たり判定のみ実行。
	//                        結果を perPairContacts[ci] / perPairHit[ci] に格納。
	//                        CheckXxx は thread_local の _tlNarrowHit / _tlContactOut
	//                        を使うため共有状態への書き込みなし。
	//    Phase B (Serial):   ヒットしたペアを _currPairs に登録し Contact をマージ。
	//                        ResolvePushOut は Transform を書き換えるのでシリアル実行。

	{
		const size_t numCandidates = candidates.size();
#ifdef _DEBUG
		auto _s = PerformanceMonitor::Instance().Scope("Collider.NarrowPhase");
#endif

		// --- Phase A: 並列 narrow-phase ---
		// 各 candidate のヒット結果と接触点を格納するバッファを確保
		auto& perPairHit = _perPairHitBuf;
		perPairHit.assign(numCandidates, 0);
		auto& perPairContacts = _perPairContactsBuf;
		if (perPairContacts.size() < numCandidates) perPairContacts.resize(numCandidates);
		for (size_t i = 0; i < numCandidates; ++i) perPairContacts[i].clear();

		ThreadPool::Instance().ParallelForBarrier(0, numCandidates, [&](size_t ci) {
			const int ai = candidates[ci].a;
			const int bi = candidates[ci].b;
			Collider* a = active[ai];
			Collider* b = active[bi];

			// thread_local バッファをこの候補専用に設定
			_tlNarrowHit  = false;
			_tlContactOut = &perPairContacts[ci];

			const auto ka = a->GetKind();
			const auto kb = b->GetKind();
			if (IsPair(ka, kb, Collider::Kind::Sphere, Collider::Kind::Sphere))         CheckSphereSphere(a, b);
			else if (IsPair(ka, kb, Collider::Kind::Sphere, Collider::Kind::Box))       CheckSphereBox(a, b);
			else if (IsPair(ka, kb, Collider::Kind::Box, Collider::Kind::Box))          CheckBoxBox(a, b);
			else if (IsPair(ka, kb, Collider::Kind::Capsule, Collider::Kind::Capsule))  CheckCapsuleCapsule(a, b);
			else if (IsPair(ka, kb, Collider::Kind::Sphere, Collider::Kind::Capsule))   CheckSphereCapsule(a, b);
			else if (IsPair(ka, kb, Collider::Kind::Box, Collider::Kind::Capsule))      CheckBoxCapsule(a, b);
			else if (IsPair(ka, kb, Collider::Kind::Sphere, Collider::Kind::HalfPlane)) {
				Collider* sp = (ka == Collider::Kind::Sphere) ? a : b;
				Collider* hp = (ka == Collider::Kind::HalfPlane) ? a : b;
				CheckSphereHalfPlane(sp, hp);
			}
			else if (IsPair(ka, kb, Collider::Kind::Box, Collider::Kind::HalfPlane)) {
				Collider* bx = (ka == Collider::Kind::Box) ? a : b;
				Collider* hp = (ka == Collider::Kind::HalfPlane) ? a : b;
				CheckBoxHalfPlane(bx, hp);
			}
			else if (IsPair(ka, kb, Collider::Kind::Capsule, Collider::Kind::HalfPlane)) {
				Collider* cp = (ka == Collider::Kind::Capsule) ? a : b;
				Collider* hp = (ka == Collider::Kind::HalfPlane) ? a : b;
				CheckCapsuleHalfPlane(cp, hp);
			}
			else if (IsPair(ka, kb, Collider::Kind::Sphere, Collider::Kind::Mesh)) {
				Collider* sp = (ka == Collider::Kind::Sphere) ? a : b;
				Collider* ms = (ka == Collider::Kind::Mesh) ? a : b;
				CheckSphereMesh(sp, ms);
			}
			else if (IsPair(ka, kb, Collider::Kind::Capsule, Collider::Kind::Mesh)) {
				Collider* cp = (ka == Collider::Kind::Capsule) ? a : b;
				Collider* ms = (ka == Collider::Kind::Mesh) ? a : b;
				CheckCapsuleMesh(cp, ms);
			}
			else if (IsPair(ka, kb, Collider::Kind::Box, Collider::Kind::Mesh)) {
				Collider* bx = (ka == Collider::Kind::Box) ? a : b;
				Collider* ms = (ka == Collider::Kind::Mesh) ? a : b;
				CheckBoxMesh(bx, ms);
			}
			else if (ka == Collider::Kind::Compound || kb == Collider::Kind::Compound) {
				Collider* comp  = (ka == Collider::Kind::Compound) ? a : b;
				Collider* other = (comp == a) ? b : a;
				CheckCompoundVsAny(comp, other);
			}

					perPairHit[ci] = _tlNarrowHit;
					_tlContactOut  = nullptr; // スレッドローカルをリセット
				}, 4); // grainSize=4: 候補ペア数が少ない場合でも複数ワーカーに分散させる

		// --- Phase B: シリアルマージ + push-out ---
		// ヒットしたペアの接触点を _contacts に統合し、ペア登録と押し戻しを実行
		for (size_t ci = 0; ci < numCandidates; ++ci) {
			if (!perPairHit[ci]) continue;

			Collider* a = active[candidates[ci].a];
			Collider* b = active[candidates[ci].b];

			const auto key = MakeKey(a, b);
			if (_currPairs.contains(key)) continue;
			_currPairs.insert(key);

			// 接触点をメインバッファへ移動
			for (auto& ct : perPairContacts[ci]) {
				_contacts.push_back(std::move(ct));
			}

			// 押し戻し (Transform 書き換えのためシリアル)
			if (!(a->isTrigger || b->isTrigger)) {
				ResolvePushOut(a, b);
			}
		}
	}
}


// Enter / Stay / Exit の差分計算。
// 前フレーム集合と今フレーム集合を比較してイベントを確定する。
void ColliderManager::ProcessPairEvents() {
  #ifdef _DEBUG
	auto _s = PerformanceMonitor::Instance().Scope("Collider.ProcessPairEvents");
	#endif
	// コールバック内で GameObject が破棄されると UnregisterCollider() から
	// _currPairs / _prevPairs が変更され、走査中のイテレータが無効化される。
	// そのため、配送内容を先に確定させてからイベントを送る。
	std::vector<PairKey> enterPairs, stayPairs, exitPairs;
	enterPairs.reserve(_currPairs.size());
	stayPairs.reserve(_currPairs.size());
	exitPairs.reserve(_prevPairs.size());

	for (const auto& k : _currPairs) {
		if (_prevPairs.contains(k)) stayPairs.push_back(k);
		else                        enterPairs.push_back(k);
	}

	for (const auto& k : _prevPairs) {
		if (!_currPairs.contains(k)) exitPairs.push_back(k);
	}

	_prevPairs = _currPairs;

	// 配送中に解除されたコライダーは dangling の可能性があるためスキップする。
	auto dispatchAll = [this](const std::vector<PairKey>& pairs,
		void (ColliderManager::* fn)(Collider*, Collider*)) {
		for (const auto& k : pairs) {
			if (IsShuttingDown()) return;
			if (!IsColliderRegistered(k.a) || !IsColliderRegistered(k.b)) continue;
			(this->*fn)(k.a, k.b);
		}
	};

	dispatchAll(stayPairs, &ColliderManager::DispatchStay);
	dispatchAll(enterPairs, &ColliderManager::DispatchEnter);
	dispatchAll(exitPairs, &ColliderManager::DispatchExit);
}

// コライダーが現在も管理対象に含まれているか。
bool ColliderManager::IsColliderRegistered(Collider* c) const noexcept {
	if (!c) return false;
	std::lock_guard lk(_mtx);
	return std::find(_colliders.begin(), _colliders.end(), c) != _colliders.end();
}

// 詳細判定一式。
// 最後に現在AABBを保存し、次フレームの swept AABB 計算に使う。
void ColliderManager::CheckDetailedCollisions() {
	#ifdef _DEBUG
	auto _s = PerformanceMonitor::Instance().Scope("Collider.CheckDetailedCollisions");
	#endif
	BuildCurrentPairs();
	ProcessPairEvents();

	auto& snapshot = _snapshotBuf;
	{
		std::lock_guard lk(_mtx);
		snapshot.assign(_colliders.begin(), _colliders.end());
	}
	for (auto* c : snapshot) {
		if (!c) continue;
		c->prevAABB = c->GetAABB();
		c->hasPrevAABB = true;
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
	else if (IsPair(ka, kb, Collider::Kind::Sphere, Collider::Kind::HalfPlane)) {
		Collider* sp = (ka == Collider::Kind::Sphere) ? a : b;
		Collider* hp = (ka == Collider::Kind::HalfPlane) ? a : b;
		PushOutSphereHalfPlane(sp, hp);
	}
	else if (IsPair(ka, kb, Collider::Kind::Box, Collider::Kind::HalfPlane)) {
		Collider* bx = (ka == Collider::Kind::Box) ? a : b;
		Collider* hp = (ka == Collider::Kind::HalfPlane) ? a : b;
		PushOutBoxHalfPlane(bx, hp);
	}
	else if (IsPair(ka, kb, Collider::Kind::Capsule, Collider::Kind::HalfPlane)) {
		Collider* cp = (ka == Collider::Kind::Capsule) ? a : b;
		Collider* hp = (ka == Collider::Kind::HalfPlane) ? a : b;
		PushOutCapsuleHalfPlane(cp, hp);
	}
	else if (IsPair(ka, kb, Collider::Kind::Sphere, Collider::Kind::Mesh)) {
		Collider* sp = (ka == Collider::Kind::Sphere) ? a : b;
		Collider* ms = (ka == Collider::Kind::Mesh) ? a : b;
		PushOutSphereMesh(sp, ms);
	}
	else if (IsPair(ka, kb, Collider::Kind::Capsule, Collider::Kind::Mesh)) {
		Collider* cp = (ka == Collider::Kind::Capsule) ? a : b;
		Collider* ms = (ka == Collider::Kind::Mesh) ? a : b;
		PushOutCapsuleMesh(cp, ms);
	}
	else if (IsPair(ka, kb, Collider::Kind::Box, Collider::Kind::Mesh)) {
		Collider* bx = (ka == Collider::Kind::Box) ? a : b;
		Collider* ms = (ka == Collider::Kind::Mesh) ? a : b;
		PushOutBoxMesh(bx, ms);
	}
	// Compound: push-out handled by child dispatch (no explicit compound push-out)
}

// Sphere-Sphere 押し戻し。
// 中心間ベクトルを法線とし、半径和との差分だけ分離する。
// pen <= 0（すり抜け済み）の場合は CCD スイープで衝突時刻を求め巻き戻す。
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
	float pen = r - dist;

	VECTOR n;
	bool ccdResolved = false;

	if (pen > 0.0f) {
		// 通常の重なりケース
		// 中心間距離がほぼゼロ（完全重なり）の場合は法線が不定になるため
		// Y軸方向をフォールバックとして使う。
		if (LenSq(d) > 1e-6f) {
			n = VScale(d, 1.0f / dist);
		}
		else {
			n = VGet(0, 1, 0);
		}
	}
	else {
		// CCD: 前フレーム位置からのスイープで衝突時刻を求める
		if (!sa->hasPrevAABB || !sb->hasPrevAABB) return;

		const VECTOR prevA = sa->prevAABB.center;
		const VECTOR prevB = sb->prevAABB.center;

		// 相対移動: A固定で B の相対軌道
		const VECTOR relPrev = VSub(prevB, prevA);
		const VECTOR relCurr = VSub(cb, ca);
		const VECTOR relDir = VSub(relCurr, relPrev);
		const float a2 = LenSq(relDir);
		if (a2 < 1e-8f) return;
		const float b2 = 2.0f * Dot3(relPrev, relDir);
		const float c2 = LenSq(relPrev) - r * r;
		const float disc = b2 * b2 - 4.0f * a2 * c2;
		if (disc < 0.0f) return;
		const float sqrtDisc = std::sqrt(disc);
		const float hitT = (-b2 - sqrtDisc) / (2.0f * a2);
		if (hitT < 0.0f || hitT > 1.0f) return;

		// 衝突時刻の位置に巻き戻す
		const VECTOR hitA = VAdd(prevA, VScale(VSub(ca, prevA), hitT));
		const VECTOR hitB = VAdd(prevB, VScale(VSub(cb, prevB), hitT));
		const VECTOR hitD = VSub(hitB, hitA);
		const float hitDist = std::sqrt((std::max)(LenSq(hitD), 1e-8f));
		n = VScale(hitD, 1.0f / hitDist);

		// 衝突面にちょうど接する位置 + 微小マージン
		const VECTOR targetA = VSub(hitA, VScale(n, 1e-4f));
		const VECTOR targetB = VAdd(hitB, VScale(n, 1e-4f));

		const float wA = (oa && !oa->isStatic) ? 1.0f : 0.0f;
		const float wB = (ob && !ob->isStatic) ? 1.0f : 0.0f;

		if (oa && !oa->isStatic) {
			const VECTOR deltaA = VSub(targetA, ca);
			VECTOR p = oa->transform.LocalPosition();
			p = VAdd(p, deltaA);
			oa->transform.SetLocalPosition(p);
		}
		if (ob && !ob->isStatic) {
			const VECTOR deltaB = VSub(targetB, cb);
			VECTOR p = ob->transform.LocalPosition();
			p = VAdd(p, deltaB);
			ob->transform.SetLocalPosition(p);
		}

		pen = 1e-4f;
		ccdResolved = true;
		sa->UpdateShape();
		sb->UpdateShape();
	}

	// 接触点は2球の中心を結ぶ線分上、半径比で内分した位置
	const VECTOR contactPoint = VAdd(sa->GetCenter(), VScale(n, sa->GetRadius()));

	Contact ct;
	ct.a = sa;
	ct.b = sb;
	ct.normal = n;
	ct.point = contactPoint;
	ct.penetration = pen;
	EmitContact(ct);

	if (ccdResolved) return;

	// ペネトレーション解消は後段の SplitImpulseCorrection /
	// PositionalCorrection に一任する (Box-Box / Sphere-Box と同方針)。
	//
	// 旧実装は EmitContact の後にここで pen * 1.05f の直接位置補正を行って
	// いたが、これは以下の問題を引き起こしていた:
	//   1. オーバーシュート (105%) で 2 球が必要以上に反発する。
	//   2. EmitContact に渡る penetration は補正前の値なので、ソルバが
	//      その penetration を再度解消しようとし、二重補正になる。
	//   3. 位置だけ動かして速度を更新しないため、複数の球が連なって
	//      接触しているシーン (壁沿いに 6 個並ぶ等) で連鎖的に押し合い、
	//      速度ゼロのまま球が空中に張り付く現象が発生する。
	// → 接触検出だけ行い、解消はソルバの impulse + position correction に任せる。
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
	if (!os && !obox) return;
	if (obox && os == obox) return;

	const bool sFixed = (!os) || os->isStatic;
	const bool boxFixed = (!obox) || obox->isStatic;
	if (sFixed && boxFixed) return;

	const VECTOR c = s->GetCenter();
	const VECTOR d = VSub(c, box->GetCenter());
	const VECTOR he = box->GetHalfExtents();
	// 球中心をOBBローカル座標に投影
	float lx = Dot3(d, box->GetAxisX());
	float ly = Dot3(d, box->GetAxisY());
	float lz = Dot3(d, box->GetAxisZ());
	// OBB上の最近点（クランプ）
	float cx = std::clamp(lx, -he.x, he.x);
	float cy = std::clamp(ly, -he.y, he.y);
	float cz = std::clamp(lz, -he.z, he.z);

	// 球中心がBox内部にあるかどうか判定
	// 内部にある場合、最近点 == 球中心 となり法線が求まらないため
	// 最小貫通軸を使って正しい分離方向を計算する。
	const bool centerInside =
		(std::fabs(lx) <= he.x) &&
		(std::fabs(ly) <= he.y) &&
		(std::fabs(lz) <= he.z);

	VECTOR n = VGet(0, 0, 0);
	float pen = 0.0f;
	bool ccdResolved = false;

	if (centerInside) {
		// --- 球中心がBox内部にある場合 ---
		// 各軸で「面までの距離」を求め、最も浅い軸を分離方向とする。
		const float axes[3] = { lx, ly, lz };
		const float exts[3] = { he.x, he.y, he.z };
		const VECTOR axisVecs[3] = { box->GetAxisX(), box->GetAxisY(), box->GetAxisZ() };

		float minPen = FLT_MAX;
		int bestAxis = 0;
		float bestSign = 1.0f;
		for (int i = 0; i < 3; ++i) {
			// 正方向の面までの距離
			const float penPos = exts[i] - axes[i];
			// 負方向の面までの距離
			const float penNeg = exts[i] + axes[i];
			if (penPos < minPen) { minPen = penPos; bestAxis = i; bestSign =  1.0f; }
			if (penNeg < minPen) { minPen = penNeg; bestAxis = i; bestSign = -1.0f; }
		}
		n = VScale(axisVecs[bestAxis], bestSign);
		// penetration = 面までの距離 + 球の半径
		pen = minPen + s->GetRadius();
	}
	else {
		// --- 球中心がBox外部にある場合（通常ケース） ---
		VECTOR closest = box->GetCenter();
		closest = VAdd(closest, VScale(box->GetAxisX(), cx));
		closest = VAdd(closest, VScale(box->GetAxisY(), cy));
		closest = VAdd(closest, VScale(box->GetAxisZ(), cz));

		const VECTOR diff = VSub(c, closest);
		const float dist = std::sqrt((std::max)(LenSq(diff), 1e-8f));
		pen = s->GetRadius() - dist;
		if (pen > 0.0f) {
			n = VScale(diff, 1.0f / dist);
		}
	}

	if (pen <= 0.0f && !centerInside) {
		if (!s->hasPrevAABB) return;

		const VECTOR prevCenter = s->prevAABB.center;
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
		if (!SweepSphereAgainstBoxRefined(prevCenter, c, s->GetRadius(), box, &hitT, &hitNormal, &hitCenter)) return;

		n = SafeNorm(hitNormal, VGet(1, 0, 0));
		const VECTOR targetCenter = VAdd(hitCenter, VScale(n, 1e-4f));
		const VECTOR centerDelta = VSub(targetCenter, c);
		pen = Len3(centerDelta);
		if (pen < 1e-4f) pen = 1e-4f;

		if (os && !os->isStatic) {
			VECTOR p = os->transform.LocalPosition();
			p = VAdd(p, centerDelta);
			os->transform.SetLocalPosition(p);
		}
		s->UpdateShape();
		box->UpdateShape();
		ccdResolved = true;
	}

	// 接触点は Box 表面上の最近点 (closest) を採用する。
	// 球中心から法線方向に「球半径分」入った位置に等しく、
	// rA = contactPoint - sphereCenter のレバーアームが正しい球半径方向
	// ベクトルになるため、摩擦インパルスから生じるトルクが正しくスケールする。
	// 中点を使うと rA が短くなり、エッジ接触で球が回転加速して飛ぶ現象が
	// 起きるためここでは使わない。
	const VECTOR contactPoint = VSub(s->GetCenter(), VScale(n, s->GetRadius()));

	Contact ct;
	ct.a = box;
	ct.b = s;
	ct.normal = n;
	ct.point = contactPoint;
	ct.penetration = pen;
	EmitContact(ct);

	// ペネトレーション解消は後段の PositionalCorrection /
	// SplitImpulseCorrection に一任する。
	// ここで直接位置を動かすと、
	//   1) コライダー位置補正 (n*β*(pen-slop))
	//   2) PositionalCorrection (kBiasFactor*(pen-slop)/invSum)
	//   3) SplitImpulseCorrection (splitBias)
	// と三重補正になり、低速で転がる球は接触法線 n の微小な揺らぎが
	// そのまま位置ジッタになる。位置補正は速度/角速度を更新しないため、
	// 球が瞬間的にワープし、ソルバの摩擦解法が静摩擦/動摩擦境界で振動して
	// 角速度に積もる。閾値を越えると動摩擦に切り替わり急発進して飛ぶ。
	// → 直接位置補正をやめ、ソルバ側に統一する（Box-Box と同方針）。
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
	int bestFaceOwner = -1; // 0 = Aの面, 1 = Bの面
	int bestFaceIndex = -1; // 面インデックス (0,1,2)
	bool separated = false; // 分離軸が見つかったかどうか。見つかった時点で以降の軸は無視して早期リターンするためのフラグ。

	// 面軸（face axis）をエッジ軸（edge axis）より優先するバイアス。
	// Box2D / Bullet と同様に、面接触の方が安定なマニフォールドを得やすいため、
	// エッジ軸は面軸より「明確に小さい」場合にのみ採用する。
	// スタッキング時のペネトレーション同程度では面軸が常に選ばれ、
	// 押し戻し方向が横に流れて下のオブジェクトが床に潜り込む挙動を防ぐ。
	constexpr float kFaceBiasAbs = 0.002f; // 絶対値バイアス (m)
	constexpr float kFaceBiasRel = 0.95f;  // 相対的バイアス (面軸計上)
	auto ConsiderAxis = [&](const VECTOR& axisW, float dist, float ra, float rb, bool isFaceAxis, int faceOwner = -1, int faceIdx = -1) {
		if (separated) return;
		const float sep = (ra + rb) - dist;
		if (sep <= 0.0f) { separated = true; return; }

		if (isFaceAxis) {
			// 面軸は常に更新を試みて確認。面軸同士の比較では単純に最小の sep を選ぶ。
			if (!bestAxisIsFace || sep < bestPen) {
				bestPen = sep;
				bestAxisW = axisW;
				bestAxisIsFace = true;
				bestFaceOwner = faceOwner;
				bestFaceIndex = faceIdx;
			}
		} else {
			// エッジ軸は面軸と比較し、面軸より「明らかに小さい」場合にのみ採用。
			if (bestAxisIsFace) {
				if (sep < bestPen * kFaceBiasRel - kFaceBiasAbs) {
					bestPen = sep;
					bestAxisW = axisW;
					bestAxisIsFace = false;
				}
			} else if (sep < bestPen) {
				bestPen = sep;
				bestAxisW = axisW;
				bestAxisIsFace = false;
			}
		}
	};

	for (int i =0; i <3; ++i) {
		float ra = aExt[i];
		float rb = bExt[0] * AbsR[i][0] + bExt[1] * AbsR[i][1] + bExt[2] * AbsR[i][2];
		float dist = std::fabs(tA[i]);
		ConsiderAxis((i ==0) ? A0 : (i ==1) ? A1 : A2, dist, ra, rb, true, 0, i);
	}

	for (int j =0; j <3; ++j) {
		float ra = aExt[0] * AbsR[0][j] + aExt[1] * AbsR[1][j] + aExt[2] * AbsR[2][j];
		float rb = bExt[j];
		float dist = std::fabs(tA[0] * R[0][j] + tA[1] * R[1][j] + tA[2] * R[2][j]);
		ConsiderAxis((j ==0) ? B0 : (j ==1) ? B1 : B2, dist, ra, rb, true, 1, j);
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
			const float invLen = 1.0f / len;
			const float ra = (aExt[1] * AbsR[2][0] + aExt[2] * AbsR[1][0]) * invLen;
			const float rb = (bExt[1] * AbsR[0][2] + bExt[2] * AbsR[0][1]) * invLen;
			const float dist = std::fabs(tA[2] * R[1][0] - tA[1] * R[2][0]) * invLen;
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}
	// i=0,j=1
	{
		VECTOR ax = CrossAxis(A0, B1);
		const float len = Len3(ax);
		if (len >1e-5f) {
			ax = VScale(ax,1.0f / len);
			const float invLen = 1.0f / len;
			const float ra = (aExt[1] * AbsR[2][1] + aExt[2] * AbsR[1][1]) * invLen;
			const float rb = (bExt[0] * AbsR[0][2] + bExt[2] * AbsR[0][0]) * invLen;
			const float dist = std::fabs(tA[2] * R[1][1] - tA[1] * R[2][1]) * invLen;
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}
	// i=0,j=2
	{
		VECTOR ax = CrossAxis(A0, B2);
		const float len = Len3(ax);
		if (len >1e-5f) {
			ax = VScale(ax,1.0f / len);
			const float invLen = 1.0f / len;
			const float ra = (aExt[1] * AbsR[2][2] + aExt[2] * AbsR[1][2]) * invLen;
			const float rb = (bExt[0] * AbsR[0][1] + bExt[1] * AbsR[0][0]) * invLen;
			const float dist = std::fabs(tA[2] * R[1][2] - tA[1] * R[2][2]) * invLen;
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}
	// i=1,j=0
	{
		VECTOR ax = CrossAxis(A1, B0);
		const float len = Len3(ax);
		if (len >1e-5f) {
			ax = VScale(ax,1.0f / len);
			const float invLen = 1.0f / len;
			const float ra = (aExt[0] * AbsR[2][0] + aExt[2] * AbsR[0][0]) * invLen;
			const float rb = (bExt[1] * AbsR[1][2] + bExt[2] * AbsR[1][1]) * invLen;
			const float dist = std::fabs(tA[0] * R[2][0] - tA[2] * R[0][0]) * invLen;
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}
	// i=1,j=1
	{
		VECTOR ax = CrossAxis(A1, B1);
		const float len = Len3(ax);
		if (len >1e-5f) {
			ax = VScale(ax,1.0f / len);
			const float invLen = 1.0f / len;
			const float ra = (aExt[0] * AbsR[2][1] + aExt[2] * AbsR[0][1]) * invLen;
			const float rb = (bExt[0] * AbsR[1][2] + bExt[2] * AbsR[1][0]) * invLen;
			const float dist = std::fabs(tA[0] * R[2][1] - tA[2] * R[0][1]) * invLen;
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}
	// i=1,j=2
	{
		VECTOR ax = CrossAxis(A1, B2);
		const float len = Len3(ax);
		if (len >1e-5f) {
			ax = VScale(ax,1.0f / len);
			const float invLen = 1.0f / len;
			const float ra = (aExt[0] * AbsR[2][2] + aExt[2] * AbsR[0][2]) * invLen;
			const float rb = (bExt[0] * AbsR[1][1] + bExt[1] * AbsR[1][0]) * invLen;
			const float dist = std::fabs(tA[0] * R[2][2] - tA[2] * R[0][2]) * invLen;
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}
	// i=2,j=0
	{
		VECTOR ax = CrossAxis(A2, B0);
		const float len = Len3(ax);
		if (len >1e-5f) {
			ax = VScale(ax,1.0f / len);
			const float invLen = 1.0f / len;
			const float ra = (aExt[0] * AbsR[1][0] + aExt[1] * AbsR[0][0]) * invLen;
			const float rb = (bExt[1] * AbsR[2][2] + bExt[2] * AbsR[2][1]) * invLen;
			const float dist = std::fabs(tA[1] * R[0][0] - tA[0] * R[1][0]) * invLen;
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}
	// i=2,j=1
	{
		VECTOR ax = CrossAxis(A2, B1);
		const float len = Len3(ax);
		if (len >1e-5f) {
			ax = VScale(ax,1.0f / len);
			const float invLen = 1.0f / len;
			const float ra = (aExt[0] * AbsR[1][1] + aExt[1] * AbsR[0][1]) * invLen;
			const float rb = (bExt[0] * AbsR[2][2] + bExt[2] * AbsR[2][0]) * invLen;
			const float dist = std::fabs(tA[1] * R[0][1] - tA[0] * R[1][1]) * invLen;
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}
	// i=2,j=2
	{
		VECTOR ax = CrossAxis(A2, B2);
		const float len = Len3(ax);
		if (len >1e-5f) {
			ax = VScale(ax,1.0f / len);
			const float invLen = 1.0f / len;
			const float ra = (aExt[0] * AbsR[1][2] + aExt[1] * AbsR[0][2]) * invLen;
			const float rb = (bExt[0] * AbsR[2][1] + bExt[1] * AbsR[2][0]) * invLen;
			const float dist = std::fabs(tA[1] * R[0][2] - tA[0] * R[1][2]) * invLen;
			ConsiderAxis(ax, dist, ra, rb, false);
		}
	}

	if (separated || bestPen == FLT_MAX || bestPen <=0.0f) {
		// 分離軸が見つかった場合、または貫通深さがゼロ以下の場合は、CCDスイープで衝突時刻を求めて巻き戻す。
		if (!ba->hasPrevAABB || !bb->hasPrevAABB) return;
		float dt = _deltaTimeSec;
		if (dt < 1e-6f) dt = 1e-6f;
		const VECTOR prevCenterA = ba->prevAABB.center;
		const VECTOR prevCenterB = bb->prevAABB.center;
		const VECTOR moveA = VSub(ba->GetCenter(), prevCenterA);
		const VECTOR moveB = VSub(bb->GetCenter(), prevCenterB);
		const float speedSqA = LenSq(moveA) / (dt * dt);
		const float speedSqB = LenSq(moveB) / (dt * dt);
		const float thrA = static_cast<Collider*>(ba)->ccdDistanceThreshold;
		const float thrB = static_cast<Collider*>(bb)->ccdDistanceThreshold;
		const bool useSweep = static_cast<Collider*>(ba)->enableCCD || static_cast<Collider*>(bb)->enableCCD
			|| speedSqA > thrA * thrA || speedSqB > thrB * thrB;
		if (!useSweep) return;

		float hitT = 0.0f;
		VECTOR hitNormal = VGet(0, 1, 0);
		if (!SweepBoxAgainstBox(prevCenterA, ba->GetCenter(), ba, prevCenterB, bb->GetCenter(), bb, &hitT, &hitNormal)) return;

		// 衝突時刻の位置に巻き戻す + 微小マージン分だけ押し戻す
		const VECTOR hitCenterA = VAdd(prevCenterA, VScale(moveA, hitT));
		const VECTOR hitCenterB = VAdd(prevCenterB, VScale(moveB, hitT));
		// 衝突面にちょうど接する位置 + 微小マージン
		if (oa && !oa->isStatic) {
			VECTOR p = oa->transform.LocalPosition();
			p = VAdd(p, VSub(hitCenterA, ba->GetCenter()));
			p = VSub(p, VScale(hitNormal, 1e-4f));
			oa->transform.SetLocalPosition(p);
		}
		if (ob && !ob->isStatic) {
			VECTOR p = ob->transform.LocalPosition();
			p = VAdd(p, VSub(hitCenterB, bb->GetCenter()));
			p = VAdd(p, VScale(hitNormal, 1e-4f));
			ob->transform.SetLocalPosition(p);
		}
		ba->UpdateShape();
		bb->UpdateShape();

		// 接触点は2Boxの中心を結ぶ線分上、微小マージン分だけ離れた位置
		const VECTOR contactPt = VScale(VAdd(hitCenterA, hitCenterB), 0.5f);
		Contact ct;
		ct.a = ba;
		ct.b = bb;
		ct.normal = hitNormal;
		ct.point = contactPt;
		ct.penetration = 1e-4f;
		EmitContact(ct);
		return;
	}

	VECTOR n = bestAxisW;
	if (Dot3(tV, n) <0.0f) {
		n = VScale(n, -1.0f);
	}
	const float pen = bestPen;

	// 最小貫通軸が面軸の場合は、接触面の交差部分が接触領域になる。
	struct ManifoldPoint { VECTOR pos; float depth; };
	ManifoldPoint manifold[8];
	int manifoldCount = 0;

	if (bestAxisIsFace) {
		// 最小貫通軸が面軸の場合、接触面の交差部分が接触領域になる。
		// まず、基準面（reference face）と被参照面（incident face）を決める。
		// 基準面は、衝突法線が面法線と同じ向きの面。被参照面は、もう一方のBoxの面。
		const VECTOR* refAxes;
		const float*  refExt;
		VECTOR        refCenter;
		const VECTOR* incAxes;
		const float*  incExt;
		VECTOR        incCenter;
		const VECTOR  refAxesArr[2][3] = { {A0,A1,A2}, {B0,B1,B2} };
		const float   refExtArr[2][3]  = { {aExt[0],aExt[1],aExt[2]}, {bExt[0],bExt[1],bExt[2]} };

		if (bestFaceOwner == 0) {
			refAxes = refAxesArr[0]; refExt = refExtArr[0]; refCenter = ba->GetCenter();
			incAxes = refAxesArr[1]; incExt = refExtArr[1]; incCenter = bb->GetCenter();
		} else {
			refAxes = refAxesArr[1]; refExt = refExtArr[1]; refCenter = bb->GetCenter();
			incAxes = refAxesArr[0]; incExt = refExtArr[0]; incCenter = ba->GetCenter();
		}

		const int refFaceIdx = bestFaceIndex;
		// 基準面の2つの接線軸
		const int refU = (refFaceIdx + 1) % 3;
		const int refV = (refFaceIdx + 2) % 3;

		// 被参照面を見つける: incident boxの面で、法線がnに最も反平行な面
		// 外向き法線がnと最も負のドット積を持つ面を選ぶ
		int incFaceIdx = 0;
		float minDot = FLT_MAX;
		for (int i = 0; i < 3; ++i) {
			float d = Dot3(n, incAxes[i]);
			if (d < minDot) { minDot = d; incFaceIdx = i; }
			float nd = -d;
			if (nd < minDot) { minDot = nd; incFaceIdx = i; }
		}
		// incFaceIdxが、被参照面の法線がnと最も反平行な面のインデックス。
		// この面の法線は incAxes[incFaceIdx] または -incAxes[incFaceIdx] のどちらかで、nと反平行な方を選ぶ。
		const float incSign = (Dot3(n, incAxes[incFaceIdx]) < 0.0f) ? 1.0f : -1.0f;
		const int incU = (incFaceIdx + 1) % 3;
		const int incV = (incFaceIdx + 2) % 3;

		// 被参照面の4頂点を求める
		VECTOR incPoly[4];
		const float signs[4][2] = { {-1,-1}, {1,-1}, {1,1}, {-1,1} };
		for (int i = 0; i < 4; ++i) {
			incPoly[i] = incCenter;
			incPoly[i] = VAdd(incPoly[i], VScale(incAxes[incFaceIdx], incSign * incExt[incFaceIdx]));
			incPoly[i] = VAdd(incPoly[i], VScale(incAxes[incU], signs[i][0] * incExt[incU]));
			incPoly[i] = VAdd(incPoly[i], VScale(incAxes[incV], signs[i][1] * incExt[incV]));
		}

		// 基準面の4辺で被参照面のポリゴンをクリップして、接触領域を求める。
		// Sutherland-Hodgmanアルゴリズムを使用。
		// 各辺の平面は、接線軸と±extentで定義される。
		VECTOR clipIn[8], clipOut[8];
		int clipInCount = 4;
		for (int i = 0; i < 4; ++i) clipIn[i] = incPoly[i];

		auto ClipPolygonByPlane = [&](VECTOR* in, int inCount, VECTOR* out, const VECTOR& planeNormal, float planeDist) -> int {
			if (inCount < 1) return 0;
			int outCount = 0;
			for (int i = 0; i < inCount; ++i) {
				const VECTOR& cur = in[i];
				const VECTOR& nxt = in[(i + 1) % inCount];
				const float dCur = Dot3(cur, planeNormal) - planeDist;
				const float dNxt = Dot3(nxt, planeNormal) - planeDist;
				if (dCur <= 0.0f) {
					if (outCount < 8) out[outCount++] = cur;
					if (dNxt > 0.0f) {
						float t = dCur / (dCur - dNxt);
						if (outCount < 8) out[outCount++] = VAdd(cur, VScale(VSub(nxt, cur), t));
					}
				} else if (dNxt <= 0.0f) {
					float t = dCur / (dCur - dNxt);
					if (outCount < 8) out[outCount++] = VAdd(cur, VScale(VSub(nxt, cur), t));
				}
			}
			return outCount;
		};

		// Clip against +refU, -refU, +refV, -refV
		// Reference face center: the center of the reference face in world space.
		// The face is on the side of refBox that faces toward the incident box.
		// n always points from A toward B.
		// For faceOwner==0 (ref=A), the reference face normal aligns with +n.
		// For faceOwner==1 (ref=B), the reference face normal aligns with -n.
		const VECTOR refFaceNormal = (bestFaceOwner == 0) ? n : VScale(n, -1.0f);
		const float refFaceSign = (Dot3(refFaceNormal, refAxes[refFaceIdx]) >= 0.0f) ? 1.0f : -1.0f;
		const VECTOR refFaceCenter = VAdd(refCenter, VScale(refAxes[refFaceIdx], refFaceSign * refExt[refFaceIdx]));

		// Plane normals in world space; distance = dot(refFaceCenter, planeNormal) ± extent
		const float refFaceCenterU = Dot3(refFaceCenter, refAxes[refU]);
		const float refFaceCenterV = Dot3(refFaceCenter, refAxes[refV]);

		// +U side: dot(p, refAxes[refU]) <= refFaceCenterU + refExt[refU]
		int cnt = ClipPolygonByPlane(clipIn, clipInCount, clipOut, refAxes[refU], refFaceCenterU + refExt[refU]);
		// -U side: dot(p, -refAxes[refU]) <= -(refFaceCenterU - refExt[refU])
		VECTOR negU = VScale(refAxes[refU], -1.0f);
		cnt = ClipPolygonByPlane(clipOut, cnt, clipIn, negU, -(refFaceCenterU - refExt[refU]));
		// +V side
		cnt = ClipPolygonByPlane(clipIn, cnt, clipOut, refAxes[refV], refFaceCenterV + refExt[refV]);
		// -V side
		VECTOR negV = VScale(refAxes[refV], -1.0f);
		cnt = ClipPolygonByPlane(clipOut, cnt, clipIn, negV, -(refFaceCenterV - refExt[refV]));

		// Project clipped points onto reference face plane and keep those behind it.
		// Each point gets its own penetration depth for accurate per-point correction.
		// depth > 0  → point is in front of (outside) the reference face
		// depth <= 0 → point is behind (inside) the reference face → penetrating
		const float refFaceD = Dot3(refFaceCenter, n);
		for (int i = 0; i < cnt && manifoldCount < 8; ++i) {
			const float depth = Dot3(clipIn[i], n) - refFaceD;
			// Reject points that are clearly in front of the reference face.
			// Use a generous threshold so edge-case border points are not discarded.
			if (depth > pen * 0.5f + 0.01f) continue;
			// Per-point penetration: the SAT pen minus how far in front of the face.
			// For points behind the face (depth <= 0), this adds to the base pen.
			// Clamp to [0, pen] with a small floor to avoid zero-depth contacts.
			const float rawPointPen = pen - depth;
			const float pointPen = (std::max)((std::min)(rawPointPen, pen), 0.001f);
			// Project onto reference face plane
			manifold[manifoldCount].pos = VSub(clipIn[i], VScale(n, depth));
			manifold[manifoldCount].depth = pointPen;
			manifoldCount++;
		}

		// If clipping produced no points, fall back to the closest point
		// between the two face centers projected onto the reference face.
		if (manifoldCount == 0) {
			// Use the incident face center projected onto the reference face
			const float incDepth = Dot3(incCenter, n) - refFaceD;
			manifold[0].pos = VSub(incCenter, VScale(n, incDepth));
			// Clamp to reference face extents
			{
				const VECTOR rel = VSub(manifold[0].pos, refFaceCenter);
				float u = Dot3(rel, refAxes[refU]);
				float v = Dot3(rel, refAxes[refV]);
				u = std::clamp(u, -refExt[refU], refExt[refU]);
				v = std::clamp(v, -refExt[refV], refExt[refV]);
				manifold[0].pos = VAdd(refFaceCenter, VAdd(
					VScale(refAxes[refU], u), VScale(refAxes[refV], v)));
			}
			manifold[0].depth = pen;
			manifoldCount = 1;
		}
	}
	else {
		// Edge-edge: single contact at midpoint
		manifold[0].pos = VScale(VAdd(ba->GetCenter(), bb->GetCenter()), 0.5f);
		manifold[0].depth = pen;
		manifoldCount = 1;
	}

	// Emit contacts with per-point penetration depth
	for (int i = 0; i < manifoldCount; ++i) {
		Contact ct;
		ct.a = ba;
		ct.b = bb;
		ct.normal = n;
		ct.point = manifold[i].pos;
		ct.penetration = manifold[i].depth;
		EmitContact(ct);
	}

	const float wA = (oa && !oa->isStatic) ? 1.0f : 0.0f;
	const float wB = (ob && !ob->isStatic) ? 1.0f : 0.0f;
	const float wSum = wA + wB;
	if (wSum <=0.0f) return;

	// 後段の SplitImpulseCorrection / PositionalCorrection と二重に効くと
	// スタッキング時にオーバーシュート振動を起こすため、ここでは
	// slop を残しつつ控えめに補正する（Box2D 流の baumgarte 風）。
	constexpr float kBoxBoxSlop   = 0.005f;
	constexpr float kBoxBoxBeta   = 0.4f;
	const float correctedPen = (std::max)(pen - kBoxBoxSlop, 0.0f) * kBoxBoxBeta;
	if (correctedPen <= 0.0f) {
		ba->UpdateShape();
		bb->UpdateShape();
		return;
	}

	float moveA = (wA / wSum) * correctedPen;
	float moveB = (wB / wSum) * correctedPen;

	auto* bodyA = dynamic_cast<PhysicsDebugClass*>(oa);
	auto* bodyB = dynamic_cast<PhysicsDebugClass*>(ob);

	if (bodyA && bodyB && bodyA->GetPhysicsBody()->IsDynamic() && bodyB->GetPhysicsBody()->IsDynamic()) {
		const float invMassA = bodyA->GetPhysicsBody()->InverseMass();
		const float invMassB = bodyB->GetPhysicsBody()->InverseMass();
		const float invMassSum = invMassA + invMassB;
		if (invMassSum > 1e-8f) {
			moveA = (invMassA / invMassSum) * correctedPen;
			moveB = (invMassB / invMassSum) * correctedPen;
		}
	}

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
	if (pen <= 0.0f) {
		// CCD: sweep via relative sphere-sphere model on segment closest points
		if (!ca->hasPrevAABB || !cb->hasPrevAABB) return;
		float dt = _deltaTimeSec;
		if (dt < 1e-6f) dt = 1e-6f;
		const VECTOR prevA = ca->prevAABB.center;
		const VECTOR prevB = cb->prevAABB.center;
		const VECTOR moveA = VSub(ca->GetCenter(), prevA);
		const VECTOR moveB = VSub(cb->GetCenter(), prevB);
		const float speedSqA = LenSq(moveA) / (dt * dt);
		const float speedSqB = LenSq(moveB) / (dt * dt);
		const float thrA = static_cast<Collider*>(ca)->ccdDistanceThreshold;
		const float thrB = static_cast<Collider*>(cb)->ccdDistanceThreshold;
		const bool useSweep = static_cast<Collider*>(ca)->enableCCD || static_cast<Collider*>(cb)->enableCCD
			|| speedSqA > thrA * thrA || speedSqB > thrB * thrB;
		if (!useSweep) return;

		// Relative motion of centers: treat as sphere-sphere sweep
		const VECTOR relPrev = VSub(prevB, prevA);
		const VECTOR relCurr = VSub(cb->GetCenter(), ca->GetCenter());
		const VECTOR relDir = VSub(relCurr, relPrev);
		const float a2 = LenSq(relDir);
		if (a2 < 1e-8f) return;
		const float b2 = 2.0f * Dot3(relPrev, relDir);
		const float c2_ = LenSq(relPrev) - r * r;
		const float disc = b2 * b2 - 4.0f * a2 * c2_;
		if (disc < 0.0f) return;
		const float sqrtDisc = std::sqrt(disc);
		const float hitT = (-b2 - sqrtDisc) / (2.0f * a2);
		if (hitT < 0.0f || hitT > 1.0f) return;

		const VECTOR hitA = VAdd(prevA, VScale(moveA, hitT));
		const VECTOR hitB = VAdd(prevB, VScale(moveB, hitT));
		const VECTOR hitD = VSub(hitB, hitA);
		const float hitDist = std::sqrt((std::max)(LenSq(hitD), 1e-8f));
		const VECTOR hitN = VScale(hitD, 1.0f / hitDist);

		if (oa && !oa->isStatic) {
			VECTOR p = oa->transform.LocalPosition();
			p = VAdd(p, VSub(hitA, ca->GetCenter()));
			p = VSub(p, VScale(hitN, 1e-4f));
			oa->transform.SetLocalPosition(p);
		}
		if (ob && !ob->isStatic) {
			VECTOR p = ob->transform.LocalPosition();
			p = VAdd(p, VSub(hitB, cb->GetCenter()));
			p = VAdd(p, VScale(hitN, 1e-4f));
			ob->transform.SetLocalPosition(p);
		}
		ca->UpdateShape();
		cb->UpdateShape();

		const VECTOR contactPt = VScale(VAdd(hitA, hitB), 0.5f);
		Contact ct;
		ct.a = ca;
		ct.b = cb;
		ct.normal = hitN;
		ct.point = contactPt;
		ct.penetration = 1e-4f;
		EmitContact(ct);
		return;
	}

	const VECTOR n = VScale(VSub(c2, c1), 1.0f / dist);
	// 接触点は2線分最近点の中点
	const VECTOR contactPoint = VScale(VAdd(c1, c2), 0.5f);

	Contact ct;
	ct.a = ca;
	ct.b = cb;
	ct.normal = n;
	ct.point = contactPoint;
	ct.penetration = pen;
	EmitContact(ct);

	// Sphere-Sphere / Sphere-Box と同方針で、ペネトレーション解消は
	// SplitImpulseCorrection / PositionalCorrection に一任する。
	// 複数接触シーン (球が壁沿いに連なる等) では、ここでの直接位置補正が
	// 速度を更新しないまま隣接接触に伝播し、空中接着や張り付きを誘発するため。
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
	if (pen <= 0.0f) {
		// CCD: sweep sphere against capsule center line
		if (!s->hasPrevAABB || !c->hasPrevAABB) return;
		float dt = _deltaTimeSec;
		if (dt < 1e-6f) dt = 1e-6f;
		const VECTOR prevS = s->prevAABB.center;
		const VECTOR prevC = c->prevAABB.center;
		const VECTOR moveS0 = VSub(s->GetCenter(), prevS);
		const VECTOR moveC0 = VSub(c->GetCenter(), prevC);
		const float speedSqS = LenSq(moveS0) / (dt * dt);
		const float speedSqC = LenSq(moveC0) / (dt * dt);
		const float thrS = static_cast<Collider*>(s)->ccdDistanceThreshold;
		const float thrC = static_cast<Collider*>(c)->ccdDistanceThreshold;
		const bool useSweep = static_cast<Collider*>(s)->enableCCD || static_cast<Collider*>(c)->enableCCD
			|| speedSqS > thrS * thrS || speedSqC > thrC * thrC;
		if (!useSweep) return;

		// Approximate: sweep sphere center vs capsule center as sphere-sphere
		const VECTOR relPrev = VSub(prevS, prevC);
		const VECTOR relCurr = VSub(s->GetCenter(), c->GetCenter());
		const VECTOR relDir = VSub(relCurr, relPrev);
		const float a2 = LenSq(relDir);
		if (a2 < 1e-8f) return;
		const float b2 = 2.0f * Dot3(relPrev, relDir);
		const float c2_ = LenSq(relPrev) - r * r;
		const float disc = b2 * b2 - 4.0f * a2 * c2_;
		if (disc < 0.0f) return;
		const float sqrtDisc = std::sqrt(disc);
		const float hitT = (-b2 - sqrtDisc) / (2.0f * a2);
		if (hitT < 0.0f || hitT > 1.0f) return;

		const VECTOR hitS = VAdd(prevS, VScale(moveS0, hitT));
		const VECTOR hitC_ = VAdd(prevC, VScale(moveC0, hitT));
		const VECTOR hitD = VSub(hitS, hitC_);
		const float hitDist_ = std::sqrt((std::max)(LenSq(hitD), 1e-8f));
		const VECTOR hitN = VScale(hitD, 1.0f / hitDist_);

		if (os && !os->isStatic) {
			VECTOR p0 = os->transform.LocalPosition();
			p0 = VAdd(p0, VSub(hitS, s->GetCenter()));
			p0 = VAdd(p0, VScale(hitN, 1e-4f));
			os->transform.SetLocalPosition(p0);
		}
		if (oc && !oc->isStatic) {
			VECTOR p0 = oc->transform.LocalPosition();
			p0 = VAdd(p0, VSub(hitC_, c->GetCenter()));
			p0 = VSub(p0, VScale(hitN, 1e-4f));
			oc->transform.SetLocalPosition(p0);
		}
		s->UpdateShape();
		c->UpdateShape();

		const VECTOR contactPt = VSub(hitS, VScale(hitN, s->GetRadius()));
		Contact ct;
		ct.a = c;
		ct.b = s;
		ct.normal = hitN;
		ct.point = contactPt;
		ct.penetration = 1e-4f;
		EmitContact(ct);
		return;
	}

	VECTOR n = VScale(diff, 1.0f / dist);
	// 接触点は球表面上の最近点
	const VECTOR contactPoint = VSub(s->GetCenter(), VScale(n, s->GetRadius()));

	Contact ct;
	ct.a = c;
	ct.b = s;
	ct.normal = n;
	ct.point = contactPoint;
	ct.penetration = pen;
	EmitContact(ct);

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
	if (bestDistSq > r * r) {
		// 距離 > 半径 ⇔ 二乗比較で十分（sqrt 不要）
		// CCD: sweep capsule center against Minkowski-expanded box
		if (!box->hasPrevAABB || !cap->hasPrevAABB) return;
		float dt = _deltaTimeSec;
		if (dt < 1e-6f) dt = 1e-6f;
		const VECTOR prevBox_ = box->prevAABB.center;
		const VECTOR prevCap_ = cap->prevAABB.center;
		const VECTOR moveBx = VSub(box->GetCenter(), prevBox_);
		const VECTOR moveCp = VSub(cap->GetCenter(), prevCap_);
		const float speedSqBx = LenSq(moveBx) / (dt * dt);
		const float speedSqCp = LenSq(moveCp) / (dt * dt);
		const float thrBx = static_cast<Collider*>(box)->ccdDistanceThreshold;
		const float thrCp = static_cast<Collider*>(cap)->ccdDistanceThreshold;
		const bool useSweep = static_cast<Collider*>(box)->enableCCD || static_cast<Collider*>(cap)->enableCCD
			|| speedSqBx > thrBx * thrBx || speedSqCp > thrCp * thrCp;
		if (!useSweep) return;

		// Approximate: sweep capsule center as sphere against Minkowski box
		float hitT = 0.0f;
		VECTOR hitNormal = VGet(0, 1, 0);
		VECTOR hitCenter = VGet(0, 0, 0);
		if (!SweepSphereAgainstBoxRefined(prevCap_, cap->GetCenter(), r, box, &hitT, &hitNormal, &hitCenter)) return;

		if (obox && !obox->isStatic) {
			VECTOR p = obox->transform.LocalPosition();
			p = VAdd(p, VScale(moveBx, hitT));
			p = VSub(p, VScale(hitNormal, 1e-4f));
			p = VSub(p, box->GetCenter());
			p = VAdd(p, obox->transform.LocalPosition());
			// Simpler: just backstep
			VECTOR posFix = obox->transform.LocalPosition();
			posFix = VSub(posFix, VScale(hitNormal, 1e-4f));
			obox->transform.SetLocalPosition(posFix);
		}
		if (ocap && !ocap->isStatic) {
			VECTOR p = ocap->transform.LocalPosition();
			p = VAdd(p, VSub(hitCenter, cap->GetCenter()));
			p = VAdd(p, VScale(hitNormal, 1e-4f));
			ocap->transform.SetLocalPosition(p);
		}
		box->UpdateShape();
		cap->UpdateShape();

		Contact ct;
		ct.a = box;
		ct.b = cap;
		ct.normal = hitNormal;
		ct.point = hitCenter;
		ct.penetration = 1e-4f;
		EmitContact(ct);
		return;
	}

	VECTOR diffL = VSub(bestSegPointL, bestBoxPointL);
	float dist = std::sqrt((std::max)(bestDistSq, 1e-8f));
	float pen = r - dist;

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

	// 接触点はOBB表面上の最近点をワールドに戻した位置
	const VECTOR contactPointW = VAdd(
		VAdd(VScale(box->GetAxisX(), bestBoxPointL.x), VScale(box->GetAxisY(), bestBoxPointL.y)),
		VAdd(VScale(box->GetAxisZ(), bestBoxPointL.z), box->GetCenter())
	);

	Contact ct;
	ct.a = box;
	ct.b = cap;
	ct.normal = n;
	ct.point = contactPointW;
	ct.penetration = pen;
	EmitContact(ct);

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

// ============================================================
//  HalfPlane narrow-phase + push-out
// ============================================================
void ColliderManager::CheckSphereHalfPlane(Collider* sphere, Collider* plane) {
	auto* s = dynamic_cast<SphereCollider*>(sphere);
	auto* hp = dynamic_cast<HalfPlaneCollider*>(plane);
	if (!s || !hp) { _tlNarrowHit = false; return; }

	const HalfPlane& pl = hp->GetPlane();
	const float dist = Dot3(s->GetCenter(), pl.normal) - pl.d;
	const float pen = s->GetRadius() - dist;
	if (pen <= 0.0f) { _tlNarrowHit = false; return; }

	_tlNarrowHit = true;
	const VECTOR contactPoint = VSub(s->GetCenter(), VScale(pl.normal, dist));
	Contact ct;
	ct.a = plane;
	ct.b = sphere;
	ct.normal = pl.normal;
	ct.point = contactPoint;
	ct.penetration = pen;
	EmitContact(ct);
}

void ColliderManager::PushOutSphereHalfPlane(Collider* sphere, Collider* plane) {
	auto* s = dynamic_cast<SphereCollider*>(sphere);
	auto* hp = dynamic_cast<HalfPlaneCollider*>(plane);
	if (!s || !hp) return;

	const HalfPlane& pl = hp->GetPlane();
	const float dist = Dot3(s->GetCenter(), pl.normal) - pl.d;
	const float pen = s->GetRadius() - dist;
	if (pen <= 0.0f) {
		// CCD: sphere may have tunneled through the half-plane
		if (!s->hasPrevAABB) return;
		float dt = _deltaTimeSec;
		if (dt < 1e-6f) dt = 1e-6f;
		const VECTOR prevCenter = s->prevAABB.center;
		const float prevDist = Dot3(prevCenter, pl.normal) - pl.d;
		// Was the sphere on the positive side last frame and now on negative?
		if (prevDist >= s->GetRadius() && dist < s->GetRadius()) {
			const VECTOR move = VSub(s->GetCenter(), prevCenter);
			const float speedSq = LenSq(move) / (dt * dt);
			const float thr = static_cast<Collider*>(s)->ccdDistanceThreshold;
			const bool useSweep = static_cast<Collider*>(s)->enableCCD || speedSq > thr * thr;
			if (useSweep) {
				// Compute TOI: time when dist == radius
				const float vn = Dot3(move, pl.normal);
				if (std::fabs(vn) > 1e-6f) {
					const float hitT = (prevDist - s->GetRadius()) / (-vn);
					if (hitT >= 0.0f && hitT <= 1.0f) {
						const VECTOR hitPos = VAdd(prevCenter, VScale(move, hitT));
						GameObject* owner = sphere->owner;
						if (owner && !owner->isStatic) {
							VECTOR p = owner->transform.LocalPosition();
							p = VAdd(p, VSub(hitPos, s->GetCenter()));
							p = VAdd(p, VScale(pl.normal, 1e-4f));
							owner->transform.SetLocalPosition(p);
							s->UpdateShape();
						}
						// Emit contact at TOI
						const VECTOR contactPoint = VSub(hitPos, VScale(pl.normal, s->GetRadius()));
						Contact ct;
						ct.a = plane;
						ct.b = sphere;
						ct.normal = pl.normal;
						ct.point = contactPoint;
						ct.penetration = 1e-4f;
						EmitContact(ct);
					}
				}
			}
		}
		return;
	}

	GameObject* owner = sphere->owner;
	if (!owner || owner->isStatic) return;
	VECTOR p = owner->transform.LocalPosition();
	p = VAdd(p, VScale(pl.normal, pen));
	owner->transform.SetLocalPosition(p);
	s->UpdateShape();
}

void ColliderManager::CheckBoxHalfPlane(Collider* box, Collider* plane) {
	auto* b = dynamic_cast<BoxCollider*>(box);
	auto* hp = dynamic_cast<HalfPlaneCollider*>(plane);
	if (!b || !hp) { _tlNarrowHit = false; return; }

	const HalfPlane& pl = hp->GetPlane();
	const VECTOR he = b->GetHalfExtents();
	// Project box extent onto plane normal: sum of |axis_i . n| * halfExtent_i
	const float proj =
		std::fabs(Dot3(b->GetAxisX(), pl.normal)) * he.x +
		std::fabs(Dot3(b->GetAxisY(), pl.normal)) * he.y +
		std::fabs(Dot3(b->GetAxisZ(), pl.normal)) * he.z;
	const float dist = Dot3(b->GetCenter(), pl.normal) - pl.d;
	const float pen = proj - dist;
	if (pen <= 0.0f) { _tlNarrowHit = false; return; }

	_tlNarrowHit = true;

	// Multi-point manifold: test all 8 corners, emit contacts for those below the plane.
	const VECTOR axes[3] = { b->GetAxisX(), b->GetAxisY(), b->GetAxisZ() };
	const float exts[3] = { he.x, he.y, he.z };
	const float signs[8][3] = {
		{-1,-1,-1}, {1,-1,-1}, {-1,1,-1}, {1,1,-1},
		{-1,-1, 1}, {1,-1, 1}, {-1,1, 1}, {1,1, 1}
	};

	int contactCount = 0;
	for (int i = 0; i < 8; ++i) {
		VECTOR corner = b->GetCenter();
		corner = VAdd(corner, VScale(axes[0], signs[i][0] * exts[0]));
		corner = VAdd(corner, VScale(axes[1], signs[i][1] * exts[1]));
		corner = VAdd(corner, VScale(axes[2], signs[i][2] * exts[2]));
		const float cornerDist = Dot3(corner, pl.normal) - pl.d;
		if (cornerDist < 0.0f) {
			Contact ct;
			ct.a = plane;
			ct.b = box;
			ct.normal = pl.normal;
			ct.point = VSub(corner, VScale(pl.normal, cornerDist));
			ct.penetration = -cornerDist;
			EmitContact(ct);
			++contactCount;
		}
	}

	// Fallback: if no corners penetrated, use center projection (single point)
	if (contactCount == 0) {
		Contact ct;
		ct.a = plane;
		ct.b = box;
		ct.normal = pl.normal;
		ct.point = VSub(b->GetCenter(), VScale(pl.normal, dist));
		ct.penetration = pen;
		EmitContact(ct);
	}
}

void ColliderManager::PushOutBoxHalfPlane(Collider* box, Collider* plane) {
	auto* b = dynamic_cast<BoxCollider*>(box);
	auto* hp = dynamic_cast<HalfPlaneCollider*>(plane);
	if (!b || !hp) return;

	const HalfPlane& pl = hp->GetPlane();
	const VECTOR he = b->GetHalfExtents();
	const float proj =
		std::fabs(Dot3(b->GetAxisX(), pl.normal)) * he.x +
		std::fabs(Dot3(b->GetAxisY(), pl.normal)) * he.y +
		std::fabs(Dot3(b->GetAxisZ(), pl.normal)) * he.z;
	const float dist = Dot3(b->GetCenter(), pl.normal) - pl.d;
	const float pen = proj - dist;
	if (pen <= 0.0f) {
		// CCD: box may have tunneled through the half-plane
		if (b->hasPrevAABB) {
			float dt = _deltaTimeSec;
			if (dt < 1e-6f) dt = 1e-6f;
			const VECTOR prevCenter = b->prevAABB.center;
			const float prevDist = Dot3(prevCenter, pl.normal) - pl.d;
			if (prevDist >= proj && dist < proj) {
				const VECTOR move = VSub(b->GetCenter(), prevCenter);
				const float speedSq = LenSq(move) / (dt * dt);
				const float thr = static_cast<Collider*>(b)->ccdDistanceThreshold;
				const bool useSweep = static_cast<Collider*>(b)->enableCCD || speedSq > thr * thr;
				if (useSweep) {
					const float vn = Dot3(move, pl.normal);
					if (std::fabs(vn) > 1e-6f) {
						const float hitT = (prevDist - proj) / (-vn);
						if (hitT >= 0.0f && hitT <= 1.0f) {
							const VECTOR hitPos = VAdd(prevCenter, VScale(move, hitT));
							GameObject* owner = box->owner;
							if (owner && !owner->isStatic) {
								VECTOR p = owner->transform.LocalPosition();
								p = VAdd(p, VSub(hitPos, b->GetCenter()));
								p = VAdd(p, VScale(pl.normal, 1e-4f));
								owner->transform.SetLocalPosition(p);
								b->UpdateShape();
							}
							const VECTOR contactPt = VSub(hitPos, VScale(pl.normal, proj));
							Contact ct;
							ct.a = plane;
							ct.b = box;
							ct.normal = pl.normal;
							ct.point = contactPt;
							ct.penetration = 1e-4f;
							EmitContact(ct);
						}
					}
				}
			}
		}
		return;
	}

	GameObject* owner = box->owner;
	if (!owner || owner->isStatic) return;
	VECTOR p = owner->transform.LocalPosition();
	p = VAdd(p, VScale(pl.normal, pen));
	owner->transform.SetLocalPosition(p);
	b->UpdateShape();
}

void ColliderManager::CheckCapsuleHalfPlane(Collider* capsule, Collider* plane) {
	auto* c = dynamic_cast<CapsuleCollider*>(capsule);
	auto* hp = dynamic_cast<HalfPlaneCollider*>(plane);
	if (!c || !hp) { _tlNarrowHit = false; return; }

	const HalfPlane& pl = hp->GetPlane();
	const float distBot = Dot3(c->GetBottom(), pl.normal) - pl.d;
	const float distTop = Dot3(c->GetTop(), pl.normal) - pl.d;
	const float minDist = (std::min)(distBot, distTop);
	const float pen = c->GetRadius() - minDist;
	if (pen <= 0.0f) { _tlNarrowHit = false; return; }

	_tlNarrowHit = true;
	const VECTOR closestEnd = (distBot < distTop) ? c->GetBottom() : c->GetTop();
	const VECTOR contactPoint = VSub(closestEnd, VScale(pl.normal, minDist));
	Contact ct;
	ct.a = plane;
	ct.b = capsule;
	ct.normal = pl.normal;
	ct.point = contactPoint;
	ct.penetration = pen;
	EmitContact(ct);
}

void ColliderManager::PushOutCapsuleHalfPlane(Collider* capsule, Collider* plane) {
	auto* c = dynamic_cast<CapsuleCollider*>(capsule);
	auto* hp = dynamic_cast<HalfPlaneCollider*>(plane);
	if (!c || !hp) return;

	const HalfPlane& pl = hp->GetPlane();
	const float distBot = Dot3(c->GetBottom(), pl.normal) - pl.d;
	const float distTop = Dot3(c->GetTop(), pl.normal) - pl.d;
	const float minDist = (std::min)(distBot, distTop);
	const float pen = c->GetRadius() - minDist;
	if (pen <= 0.0f) {
		// CCD:はしごの両端が半平面を貫通している可能性。カプセル中心の前フレーム位置を見て、片側にいて今は反対側にいるなら要注意。
		if (c->hasPrevAABB) {
			float dt = _deltaTimeSec;
			if (dt < 1e-6f) dt = 1e-6f;
			const VECTOR prevCenter = c->prevAABB.center;
			const float prevMinDist = Dot3(prevCenter, pl.normal) - pl.d;
			if (prevMinDist >= c->GetRadius() && minDist < c->GetRadius()) {
				const VECTOR move = VSub(c->GetCenter(), prevCenter);
				const float speedSq = LenSq(move) / (dt * dt);
				const float thr = static_cast<Collider*>(c)->ccdDistanceThreshold;
				const bool useSweep = static_cast<Collider*>(c)->enableCCD || speedSq > thr * thr;
				if (useSweep) {
					const float vn = Dot3(move, pl.normal);
					if (std::fabs(vn) > 1e-6f) {
						const float hitT = (prevMinDist - c->GetRadius()) / (-vn);
						if (hitT >= 0.0f && hitT <= 1.0f) {
							const VECTOR hitPos = VAdd(prevCenter, VScale(move, hitT));
							GameObject* owner = capsule->owner;
							if (owner && !owner->isStatic) {
								VECTOR p = owner->transform.LocalPosition();
								p = VAdd(p, VSub(hitPos, c->GetCenter()));
								p = VAdd(p, VScale(pl.normal, 1e-4f));
								owner->transform.SetLocalPosition(p);
								c->UpdateShape();
							}
							const VECTOR closestEnd = (distBot < distTop) ? c->GetBottom() : c->GetTop();
							const VECTOR contactPt = VSub(closestEnd, VScale(pl.normal, minDist));
							Contact ct;
							ct.a = plane;
							ct.b = capsule;
							ct.normal = pl.normal;
							ct.point = contactPt;
							ct.penetration = 1e-4f;
							EmitContact(ct);
						}
					}
				}
			}
		}
		return;
	}

	GameObject* owner = capsule->owner;
	if (!owner || owner->isStatic) return;
	VECTOR p = owner->transform.LocalPosition();
	p = VAdd(p, VScale(pl.normal, pen));
	owner->transform.SetLocalPosition(p);
	c->UpdateShape();
}

// ============================================================
//  Mesh (Triangle Mesh) narrow-phase + push-out
//  メッシュ側は静的扱い：押し戻しは Sphere 側のみ移動する。
//  Mesh が動的（owner!=null かつ !isStatic）の場合でも、現状は相手のみ移動する設計。
// ============================================================
namespace {
	// 点 p から三角形 (a,b,c) 上の最近点を求める（Christer Ericke, RTCD）
	// 戻り値: 三角形上の最近点。outBary は (u,v,w) で p_near = u*a + v*b + w*c。
	inline VECTOR ClosestPointOnTriangle(
		const VECTOR& p,
		const VECTOR& a, const VECTOR& b, const VECTOR& c) noexcept
	{
		const VECTOR ab = VSub(b, a);
		const VECTOR ac = VSub(c, a);
		const VECTOR ap = VSub(p, a);
		const float d1 = Dot3(ab, ap);
		const float d2 = Dot3(ac, ap);
		if (d1 <= 0.0f && d2 <= 0.0f) return a; // 領域 A

		const VECTOR bp = VSub(p, b);
		const float d3 = Dot3(ab, bp);
		const float d4 = Dot3(ac, bp);
		if (d3 >= 0.0f && d4 <= d3) return b; // 領域 B

		const float vc = d1 * d4 - d3 * d2;
		if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
			const float v = d1 / (d1 - d3);
			return VAdd(a, VScale(ab, v)); // 辺 AB
		}

		const VECTOR cp = VSub(p, c);
		const float d5 = Dot3(ab, cp);
		const float d6 = Dot3(ac, cp);
		if (d6 >= 0.0f && d5 <= d6) return c; // 領域 C

		const float vb = d5 * d2 - d1 * d6;
		if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
			const float w = d2 / (d2 - d6);
			return VAdd(a, VScale(ac, w)); // 辺 AC
		}

		const float va = d3 * d6 - d5 * d4;
		if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
			const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
			return VAdd(b, VScale(VSub(c, b), w)); // 辺 BC
		}

		// 内部
		const float denom = 1.0f / (va + vb + vc);
		const float v = vb * denom;
		const float w = vc * denom;
		return VAdd(a, VAdd(VScale(ab, v), VScale(ac, w)));
	}

	// 半径 radius で膨らませた球の AABB を作る（BVH 問い合わせ用）
	inline AABB MakeSphereAABB(const VECTOR& center, float radius) noexcept {
		AABB box;
		box.min = VGet(center.x - radius, center.y - radius, center.z - radius);
		box.max = VGet(center.x + radius, center.y + radius, center.z + radius);
		box.center = center;
		return box;
	}
}

void ColliderManager::CheckSphereMesh(Collider* sphere, Collider* mesh) {
	auto* s = dynamic_cast<SphereCollider*>(sphere);
	auto* m = dynamic_cast<MeshCollider*>(mesh);
	if (!s || !m) { _tlNarrowHit = false; return; }

	const VECTOR sc = s->GetCenter();
	const float r = s->GetRadius();
	const float r2 = r * r;
	const AABB query = MakeSphereAABB(sc, r);

	bool anyHit = false;
	const auto& tris = m->Triangles();

	m->QueryOverlapping(query, [&](size_t triIdx) {
		const Triangle& tri = tris[triIdx];
		const VECTOR cp = ClosestPointOnTriangle(sc, tri.v0, tri.v1, tri.v2);
		const VECTOR diff = VSub(sc, cp);
		const float distSq = LenSq(diff);
		if (distSq > r2) return;

		// 接触法線：球中心→最近点 の逆向き。ゼロ距離時は三角形法線で代替。
		VECTOR normal;
		float dist;
		if (distSq > 1e-8f) {
			dist = std::sqrt(distSq);
			normal = VScale(diff, 1.0f / dist);
		}
		else {
			dist = 0.0f;
			normal = tri.normal;
		}
		const float pen = r - dist;
		if (pen <= 0.0f) return;

		anyHit = true;
		Contact ct;
		ct.a = mesh;   // mesh 側を a に統一（HalfPlane と同じ規約）
		ct.b = sphere;
		ct.normal = normal;
		ct.point = cp;
		ct.penetration = pen;
		EmitContact(ct);
	});

	_tlNarrowHit = anyHit;
}

void ColliderManager::PushOutSphereMesh(Collider* sphere, Collider* mesh) {
	auto* s = dynamic_cast<SphereCollider*>(sphere);
	auto* m = dynamic_cast<MeshCollider*>(mesh);
	if (!s || !m) return;

	GameObject* owner = sphere->owner;
	if (!owner || owner->isStatic) return;

	// メッシュ全体を一度走査して MTV（最深押し戻し）を集計する。
	// 単純に最深を採用すると角で振動するため、各接触の押し戻しベクトルを
	// 合成（重複方向はクランプ）するシンプルな反復押し出しを 1 パスで行う。
	const float rOrig = s->GetRadius();
	const float r2 = rOrig * rOrig;
	const auto& tris = m->Triangles();

	VECTOR totalPush = VGet(0, 0, 0);
	int hitCount = 0;

	const VECTOR sc0 = s->GetCenter();
	const AABB query = MakeSphereAABB(sc0, rOrig);

	m->QueryOverlapping(query, [&](size_t triIdx) {
		const Triangle& tri = tris[triIdx];
		const VECTOR cp = ClosestPointOnTriangle(sc0, tri.v0, tri.v1, tri.v2);
		const VECTOR diff = VSub(sc0, cp);
		const float distSq = LenSq(diff);
		if (distSq > r2) return;

		VECTOR normal;
		float dist;
		if (distSq > 1e-8f) {
			dist = std::sqrt(distSq);
			normal = VScale(diff, 1.0f / dist);
		}
		else {
			dist = 0.0f;
			normal = tri.normal;
		}
		const float pen = rOrig - dist;
		if (pen <= 0.0f) return;

		// 同方向の押し戻しが重複しないよう、既存合成方向への投影を差し引いて足す。
		const float already = Dot3(totalPush, normal);
		const float add = pen - already;
		if (add > 0.0f) {
			totalPush = VAdd(totalPush, VScale(normal, add));
		}
		++hitCount;
	});

	if (hitCount == 0) {
		// CCD: トンネル抜けの簡易検出（前フレーム中心→現フレーム中心の線分が
		// 三角形 BVH 範囲を貫いている場合に最小貫通量で押し戻す）。
		// Step3 では簡易対応に留め、本格的なスイープは将来 Step に回す。
		if (!s->hasPrevAABB) return;
		const VECTOR prevCenter = s->prevAABB.center;
		const VECTOR move = VSub(sc0, prevCenter);
		const float moveLen2 = LenSq(move);
		if (moveLen2 < 1e-8f) return;

		// 移動経路の AABB で再問い合わせ
		AABB sweptQuery;
		sweptQuery.min = VGet(
			(std::min)(prevCenter.x, sc0.x) - rOrig,
			(std::min)(prevCenter.y, sc0.y) - rOrig,
			(std::min)(prevCenter.z, sc0.z) - rOrig);
		sweptQuery.max = VGet(
			(std::max)(prevCenter.x, sc0.x) + rOrig,
			(std::max)(prevCenter.y, sc0.y) + rOrig,
			(std::max)(prevCenter.z, sc0.z) + rOrig);
		sweptQuery.center = VScale(VAdd(sweptQuery.min, sweptQuery.max), 0.5f);

		float bestT = 2.0f;
		VECTOR bestNormal = VGet(0, 1, 0);
		m->QueryOverlapping(sweptQuery, [&](size_t triIdx) {
			const Triangle& tri = tris[triIdx];
			const float vn = Dot3(move, tri.normal);
			if (std::fabs(vn) < 1e-6f) return;
			const float d0 = Dot3(VSub(prevCenter, tri.v0), tri.normal);
			// 球面が三角形平面に触れる時刻
			const float t = (rOrig - d0) / vn;
			if (t < 0.0f || t > 1.0f) return;
			const VECTOR hitCenter = VAdd(prevCenter, VScale(move, t));
			// 平面投影点が三角形内に近いかを最近点距離で判定
			const VECTOR cp = ClosestPointOnTriangle(hitCenter, tri.v0, tri.v1, tri.v2);
			if (LenSq(VSub(hitCenter, cp)) > rOrig * rOrig + 1e-3f) return;
			if (t < bestT) {
				bestT = t;
				bestNormal = (vn < 0.0f) ? tri.normal : VScale(tri.normal, -1.0f);
			}
		});
		if (bestT > 1.0f) return;

		const VECTOR hitCenter = VAdd(prevCenter, VScale(move, bestT));
		VECTOR p = owner->transform.LocalPosition();
		p = VAdd(p, VSub(hitCenter, sc0));
		p = VAdd(p, VScale(bestNormal, 1e-4f));
		owner->transform.SetLocalPosition(p);
		s->UpdateShape();

		Contact ct;
		ct.a = mesh;
		ct.b = sphere;
		ct.normal = bestNormal;
		ct.point = hitCenter;
		ct.penetration = 1e-4f;
		EmitContact(ct);
		return;
	}

	// 集計した押し戻しベクトルを適用
	VECTOR p = owner->transform.LocalPosition();
	p = VAdd(p, totalPush);
	owner->transform.SetLocalPosition(p);
	s->UpdateShape();
}

// ------------------------------------------------------------
//  Capsule vs Mesh
//  カプセル線分上の最近点を求め、その点と三角形最近点の距離で判定する。
//  Sphere vs Mesh と同じ「投影合成」押し戻しで角の振動を抑える。
// ------------------------------------------------------------
namespace {
	// 点 p と線分 (a,b) の最近点（線分上の点）を返す。outT は a→b の補間係数 [0,1]。
	inline VECTOR ClosestPointOnSegment(
		const VECTOR& p, const VECTOR& a, const VECTOR& b,
		float* outT = nullptr) noexcept
	{
		const VECTOR ab = VSub(b, a);
		const float ab2 = LenSq(ab);
		if (ab2 < 1e-12f) {
			if (outT) *outT = 0.0f;
			return a;
		}
		float t = Dot3(VSub(p, a), ab) / ab2;
		if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
		if (outT) *outT = t;
		return VAdd(a, VScale(ab, t));
	}

	// 線分 (s0,s1) と三角形 (v0,v1,v2) の最近点ペアを求める。
	// 戻り値: 距離の二乗。outSeg は線分上の最近点、outTri は三角形上の最近点。
	// 実装は単純化のため「両端＋三角形辺3本上の線分-線分最近点」と
	// 「線分のクランプ平面投影」を組み合わせた近似（ゲーム用途には十分）。
	inline float ClosestPointSegmentTriangle(
		const VECTOR& s0, const VECTOR& s1,
		const VECTOR& v0, const VECTOR& v1, const VECTOR& v2,
		VECTOR* outSeg, VECTOR* outTri) noexcept
	{
		// 候補1: 線分両端 → 三角形最近点
		const VECTOR cp0 = ClosestPointOnTriangle(s0, v0, v1, v2);
		const VECTOR cp1 = ClosestPointOnTriangle(s1, v0, v1, v2);
		float best = LenSq(VSub(s0, cp0));
		VECTOR bestSeg = s0;
		VECTOR bestTri = cp0;
		{
			const float d1 = LenSq(VSub(s1, cp1));
			if (d1 < best) { best = d1; bestSeg = s1; bestTri = cp1; }
		}

		// 候補2: 線分 vs 三角形の各辺の線分-線分最近点
		auto SegSegClosest = [&](const VECTOR& p0, const VECTOR& p1,
			const VECTOR& q0, const VECTOR& q1,
			VECTOR* op, VECTOR* oq) -> float
		{
			const VECTOR d1 = VSub(p1, p0);
			const VECTOR d2 = VSub(q1, q0);
			const VECTOR r = VSub(p0, q0);
			const float a = Dot3(d1, d1);
			const float e = Dot3(d2, d2);
			const float f = Dot3(d2, r);
			float s, t;
			if (a <= 1e-12f && e <= 1e-12f) { s = t = 0.0f; }
			else if (a <= 1e-12f) {
				s = 0.0f;
				t = f / e; if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
			}
			else {
				const float c = Dot3(d1, r);
				if (e <= 1e-12f) {
					t = 0.0f;
					s = -c / a; if (s < 0.0f) s = 0.0f; else if (s > 1.0f) s = 1.0f;
				}
				else {
					const float b = Dot3(d1, d2);
					const float denom = a * e - b * b;
					if (denom != 0.0f) {
						s = (b * f - c * e) / denom;
						if (s < 0.0f) s = 0.0f; else if (s > 1.0f) s = 1.0f;
					}
					else {
						s = 0.0f;
					}
					t = (b * s + f) / e;
					if (t < 0.0f) { t = 0.0f; s = -c / a; if (s < 0.0f) s = 0.0f; else if (s > 1.0f) s = 1.0f; }
					else if (t > 1.0f) { t = 1.0f; s = (b - c) / a; if (s < 0.0f) s = 0.0f; else if (s > 1.0f) s = 1.0f; }
				}
			}
			const VECTOR cp = VAdd(p0, VScale(d1, s));
			const VECTOR cq = VAdd(q0, VScale(d2, t));
			if (op) *op = cp;
			if (oq) *oq = cq;
			return LenSq(VSub(cp, cq));
		};

		VECTOR ep, eq;
		float d;
		d = SegSegClosest(s0, s1, v0, v1, &ep, &eq);
		if (d < best) { best = d; bestSeg = ep; bestTri = eq; }
		d = SegSegClosest(s0, s1, v1, v2, &ep, &eq);
		if (d < best) { best = d; bestSeg = ep; bestTri = eq; }
		d = SegSegClosest(s0, s1, v2, v0, &ep, &eq);
		if (d < best) { best = d; bestSeg = ep; bestTri = eq; }

		if (outSeg) *outSeg = bestSeg;
		if (outTri) *outTri = bestTri;
		return best;
	}

	// カプセル線分 AABB を radius で膨らませる
	inline AABB MakeCapsuleAABB(const VECTOR& a, const VECTOR& b, float radius) noexcept {
		AABB box;
		box.min = VGet(
			(std::min)(a.x, b.x) - radius,
			(std::min)(a.y, b.y) - radius,
			(std::min)(a.z, b.z) - radius);
		box.max = VGet(
			(std::max)(a.x, b.x) + radius,
			(std::max)(a.y, b.y) + radius,
			(std::max)(a.z, b.z) + radius);
		box.center = VScale(VAdd(box.min, box.max), 0.5f);
		return box;
	}
}

void ColliderManager::CheckCapsuleMesh(Collider* capsule, Collider* mesh) {
	auto* c = dynamic_cast<CapsuleCollider*>(capsule);
	auto* m = dynamic_cast<MeshCollider*>(mesh);
	if (!c || !m) { _tlNarrowHit = false; return; }

	const VECTOR ca = c->GetBottom();
	const VECTOR cb = c->GetTop();
	const float r = c->GetRadius();
	const float r2 = r * r;
	const AABB query = MakeCapsuleAABB(ca, cb, r);

	bool anyHit = false;
	const auto& tris = m->Triangles();

	m->QueryOverlapping(query, [&](size_t triIdx) {
		const Triangle& tri = tris[triIdx];
		VECTOR segP, triP;
		const float distSq = ClosestPointSegmentTriangle(ca, cb, tri.v0, tri.v1, tri.v2, &segP, &triP);
		if (distSq > r2) return;

		VECTOR normal;
		float dist;
		const VECTOR diff = VSub(segP, triP);
		if (distSq > 1e-8f) {
			dist = std::sqrt(distSq);
			normal = VScale(diff, 1.0f / dist);
		}
		else {
			dist = 0.0f;
			normal = tri.normal;
		}
		const float pen = r - dist;
		if (pen <= 0.0f) return;

		anyHit = true;
		Contact ct;
		ct.a = mesh;
		ct.b = capsule;
		ct.normal = normal;
		ct.point = triP;
		ct.penetration = pen;
		EmitContact(ct);
	});

	_tlNarrowHit = anyHit;
}

void ColliderManager::PushOutCapsuleMesh(Collider* capsule, Collider* mesh) {
	auto* c = dynamic_cast<CapsuleCollider*>(capsule);
	auto* m = dynamic_cast<MeshCollider*>(mesh);
	if (!c || !m) return;

	GameObject* owner = capsule->owner;
	if (!owner || owner->isStatic) return;

	const VECTOR ca = c->GetBottom();
	const VECTOR cb = c->GetTop();
	const float r = c->GetRadius();
	const float r2 = r * r;
	const auto& tris = m->Triangles();
	const AABB query = MakeCapsuleAABB(ca, cb, r);

	VECTOR totalPush = VGet(0, 0, 0);
	int hitCount = 0;

	m->QueryOverlapping(query, [&](size_t triIdx) {
		const Triangle& tri = tris[triIdx];
		VECTOR segP, triP;
		const float distSq = ClosestPointSegmentTriangle(ca, cb, tri.v0, tri.v1, tri.v2, &segP, &triP);
		if (distSq > r2) return;

		VECTOR normal;
		float dist;
		const VECTOR diff = VSub(segP, triP);
		if (distSq > 1e-8f) {
			dist = std::sqrt(distSq);
			normal = VScale(diff, 1.0f / dist);
		}
		else {
			dist = 0.0f;
			normal = tri.normal;
		}
		const float pen = r - dist;
		if (pen <= 0.0f) return;

		const float already = Dot3(totalPush, normal);
		const float add = pen - already;
		if (add > 0.0f) {
			totalPush = VAdd(totalPush, VScale(normal, add));
		}
		++hitCount;
	});

	if (hitCount == 0) {
		// CCD: 簡易スイープ（カプセル中心の移動経路で再走査）。
		// 実用上は十分な近似。完全なスイープは将来 Step へ。
		if (!c->hasPrevAABB) return;
		const VECTOR center = c->GetCenter();
		const VECTOR prevCenter = c->prevAABB.center;
		const VECTOR move = VSub(center, prevCenter);
		if (LenSq(move) < 1e-8f) return;

		AABB sweptQuery;
		sweptQuery.min = VGet(
			(std::min)(query.min.x, query.min.x - move.x),
			(std::min)(query.min.y, query.min.y - move.y),
			(std::min)(query.min.z, query.min.z - move.z));
		sweptQuery.max = VGet(
			(std::max)(query.max.x, query.max.x - move.x),
			(std::max)(query.max.y, query.max.y - move.y),
			(std::max)(query.max.z, query.max.z - move.z));
		sweptQuery.center = VScale(VAdd(sweptQuery.min, sweptQuery.max), 0.5f);

		float bestT = 2.0f;
		VECTOR bestNormal = VGet(0, 1, 0);
		m->QueryOverlapping(sweptQuery, [&](size_t triIdx) {
			const Triangle& tri = tris[triIdx];
			const float vn = Dot3(move, tri.normal);
			if (std::fabs(vn) < 1e-6f) return;
			// 線分のうち最も平面に近い端点で判定（簡易）
			const float dPrev = Dot3(VSub(prevCenter, tri.v0), tri.normal);
			const float t = (r - dPrev) / vn;
			if (t < 0.0f || t > 1.0f) return;
			if (t < bestT) {
				bestT = t;
				bestNormal = (vn < 0.0f) ? tri.normal : VScale(tri.normal, -1.0f);
			}
		});
		if (bestT > 1.0f) return;

		const VECTOR hitCenter = VAdd(prevCenter, VScale(move, bestT));
		VECTOR p = owner->transform.LocalPosition();
		p = VAdd(p, VSub(hitCenter, center));
		p = VAdd(p, VScale(bestNormal, 1e-4f));
		owner->transform.SetLocalPosition(p);
		c->UpdateShape();

		Contact ct;
		ct.a = mesh;
		ct.b = capsule;
		ct.normal = bestNormal;
		ct.point = hitCenter;
		ct.penetration = 1e-4f;
		EmitContact(ct);
		return;
	}

	VECTOR p = owner->transform.LocalPosition();
	p = VAdd(p, totalPush);
	owner->transform.SetLocalPosition(p);
	c->UpdateShape();
}

// ------------------------------------------------------------
//  Box (OBB) vs Mesh
//  各候補三角形に対し 13 軸 SAT で重なり判定し、最小貫通量(MTV)を求める。
//  押し戻しは Sphere/Capsule と同じ「投影合成」方式で角の振動を抑える。
// ------------------------------------------------------------
namespace {
	// 三角形と OBB の SAT 重なり判定。
	// 戻り値 true なら重なっている。outNormal は OBB 側を押し戻す方向（OBB中心→外）、
	// outPen は最小貫通量。
	inline bool OBBTriangleSAT(
		const BoxCollider* box,
		const Triangle& tri,
		VECTOR* outNormal, float* outPen) noexcept
	{
		const VECTOR center = box->GetCenter();
		const VECTOR axisA[3] = { box->GetAxisX(), box->GetAxisY(), box->GetAxisZ() };
		const VECTOR he = box->GetHalfExtents();
		const float extA[3] = { he.x, he.y, he.z };

		// 三角形の頂点を OBB 中心基準にする
		const VECTOR v[3] = {
			VSub(tri.v0, center),
			VSub(tri.v1, center),
			VSub(tri.v2, center),
		};
		const VECTOR edges[3] = {
			VSub(tri.v1, tri.v0),
			VSub(tri.v2, tri.v1),
			VSub(tri.v0, tri.v2),
		};

		float minPen = 1e30f;
		VECTOR bestAxis = VGet(0, 1, 0);

		auto Test = [&](VECTOR axis) -> bool {
			const float al2 = LenSq(axis);
			if (al2 < 1e-10f) return true; // 退化軸は無視（重なり判定では分離なし扱い）
			const float invLen = 1.0f / std::sqrt(al2);
			axis = VScale(axis, invLen);

			// OBB 投影半径
			float rA = 0.0f;
			for (int i = 0; i < 3; ++i) rA += std::fabs(Dot3(axisA[i], axis)) * extA[i];

			// 三角形投影 [tmin, tmax]
			const float p0 = Dot3(v[0], axis);
			const float p1 = Dot3(v[1], axis);
			const float p2 = Dot3(v[2], axis);
			const float tmin = (std::min)(p0, (std::min)(p1, p2));
			const float tmax = (std::max)(p0, (std::max)(p1, p2));

			// 重なり量: 分離していれば <=0
			const float overlap = (std::min)(rA - tmin, tmax + rA);
			if (overlap <= 0.0f) return false; // 分離軸

			if (overlap < minPen) {
				minPen = overlap;
				// 押し戻し方向：OBB 中心→三角形 と逆向き＝OBB を遠ざける方向にする
				const float center2tri = 0.5f * (tmin + tmax);
				bestAxis = (center2tri >= 0.0f) ? VScale(axis, -1.0f) : axis;
			}
			return true;
		};

		// 1. OBB の 3 軸
		for (int i = 0; i < 3; ++i) {
			if (!Test(axisA[i])) return false;
		}
		// 2. 三角形の法線
		if (!Test(tri.normal)) return false;
		// 3. OBB 軸 × 三角形辺 の 9 軸
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				if (!Test(VCross(axisA[i], edges[j]))) return false;
			}
		}

		if (outNormal) *outNormal = bestAxis;
		if (outPen) *outPen = minPen;
		return true;
	}

	// OBB の AABB を求める（広域 BVH 問い合わせ用）。BoxCollider::GetAABB() を使う。
	inline AABB GetBoxQueryAABB(const BoxCollider* box) noexcept {
		return box->GetAABB();
	}
}

void ColliderManager::CheckBoxMesh(Collider* box, Collider* mesh) {
	auto* b = dynamic_cast<BoxCollider*>(box);
	auto* m = dynamic_cast<MeshCollider*>(mesh);
	if (!b || !m) { _tlNarrowHit = false; return; }

	const AABB query = GetBoxQueryAABB(b);
	const auto& tris = m->Triangles();

	bool anyHit = false;
	m->QueryOverlapping(query, [&](size_t triIdx) {
		const Triangle& tri = tris[triIdx];
		VECTOR normal;
		float pen;
		if (!OBBTriangleSAT(b, tri, &normal, &pen)) return;
		if (pen <= 0.0f) return;

		anyHit = true;
		Contact ct;
		ct.a = mesh;
		ct.b = box;
		ct.normal = normal;
		// 接触点：三角形重心の最も近い面投影で近似（Multi-point contact までは作らない）
		ct.point = VScale(VAdd(VAdd(tri.v0, tri.v1), tri.v2), 1.0f / 3.0f);
		ct.penetration = pen;
		EmitContact(ct);
	});

	_tlNarrowHit = anyHit;
}

void ColliderManager::PushOutBoxMesh(Collider* box, Collider* mesh) {
	auto* b = dynamic_cast<BoxCollider*>(box);
	auto* m = dynamic_cast<MeshCollider*>(mesh);
	if (!b || !m) return;

	GameObject* owner = box->owner;
	if (!owner || owner->isStatic) return;

	const AABB query = GetBoxQueryAABB(b);
	const auto& tris = m->Triangles();

	VECTOR totalPush = VGet(0, 0, 0);
	int hitCount = 0;

	m->QueryOverlapping(query, [&](size_t triIdx) {
		const Triangle& tri = tris[triIdx];
		VECTOR normal;
		float pen;
		if (!OBBTriangleSAT(b, tri, &normal, &pen)) return;
		if (pen <= 0.0f) return;

		const float already = Dot3(totalPush, normal);
		const float add = pen - already;
		if (add > 0.0f) {
			totalPush = VAdd(totalPush, VScale(normal, add));
		}
		++hitCount;
	});

	if (hitCount == 0) {
		// CCD: OBB 中心の移動経路で簡易再走査（Sphere/Capsule と同方針）。
		if (!b->hasPrevAABB) return;
		const VECTOR center = b->GetCenter();
		const VECTOR prevCenter = b->prevAABB.center;
		const VECTOR move = VSub(center, prevCenter);
		if (LenSq(move) < 1e-8f) return;

		AABB sweptQuery;
		sweptQuery.min = VGet(
			(std::min)(query.min.x, query.min.x - move.x),
			(std::min)(query.min.y, query.min.y - move.y),
			(std::min)(query.min.z, query.min.z - move.z));
		sweptQuery.max = VGet(
			(std::max)(query.max.x, query.max.x - move.x),
			(std::max)(query.max.y, query.max.y - move.y),
			(std::max)(query.max.z, query.max.z - move.z));
		sweptQuery.center = VScale(VAdd(sweptQuery.min, sweptQuery.max), 0.5f);

		float bestT = 2.0f;
		VECTOR bestNormal = VGet(0, 1, 0);
		const VECTOR axisA[3] = { b->GetAxisX(), b->GetAxisY(), b->GetAxisZ() };
		const VECTOR he = b->GetHalfExtents();
		const float extA[3] = { he.x, he.y, he.z };

		m->QueryOverlapping(sweptQuery, [&](size_t triIdx) {
			const Triangle& tri = tris[triIdx];
			const float vn = Dot3(move, tri.normal);
			if (std::fabs(vn) < 1e-6f) return;
			// OBB の三角形法線方向への投影半径
			float rA = 0.0f;
			for (int i = 0; i < 3; ++i) rA += std::fabs(Dot3(axisA[i], tri.normal)) * extA[i];
			const float dPrev = Dot3(VSub(prevCenter, tri.v0), tri.normal);
			const float t = (rA - dPrev) / vn;
			if (t < 0.0f || t > 1.0f) return;
			if (t < bestT) {
				bestT = t;
				bestNormal = (vn < 0.0f) ? tri.normal : VScale(tri.normal, -1.0f);
			}
		});
		if (bestT > 1.0f) return;

		const VECTOR hitCenter = VAdd(prevCenter, VScale(move, bestT));
		VECTOR p = owner->transform.LocalPosition();
		p = VAdd(p, VSub(hitCenter, center));
		p = VAdd(p, VScale(bestNormal, 1e-4f));
		owner->transform.SetLocalPosition(p);
		b->UpdateShape();

		Contact ct;
		ct.a = mesh;
		ct.b = box;
		ct.normal = bestNormal;
		ct.point = hitCenter;
		ct.penetration = 1e-4f;
		EmitContact(ct);
		return;
	}

	VECTOR p = owner->transform.LocalPosition();
	p = VAdd(p, totalPush);
	owner->transform.SetLocalPosition(p);
	b->UpdateShape();
}

// ============================================================
// Compound vs Any: check each child against the other, with AABB culling
// ============================================================
void ColliderManager::CheckCompoundVsAny(Collider* compound, Collider* other) {
	auto* comp = dynamic_cast<CompoundCollider*>(compound);
	if (!comp) { _tlNarrowHit = false; return; }

	bool anyHit = false;
	const AABB& otherAABB = other->GetAABB();

	// Compound の子供たちと other を全てチェック。子供のAABBと otherAABB が重なっているものだけ narrow-phase へ。
	comp->QueryOverlapping(otherAABB, [&](size_t childIdx) {
		Collider* child = comp->GetChild(childIdx);
		if (!child) return;

		_tlNarrowHit = false;
		const auto ck = child->GetKind();
		const auto ok = other->GetKind();

		if (IsPair(ck, ok, Collider::Kind::Sphere, Collider::Kind::Sphere)) CheckSphereSphere(child, other);
		else if (IsPair(ck, ok, Collider::Kind::Sphere, Collider::Kind::Box)) CheckSphereBox(child, other);
		else if (IsPair(ck, ok, Collider::Kind::Box, Collider::Kind::Box)) CheckBoxBox(child, other);
		else if (IsPair(ck, ok, Collider::Kind::Capsule, Collider::Kind::Capsule)) CheckCapsuleCapsule(child, other);
		else if (IsPair(ck, ok, Collider::Kind::Sphere, Collider::Kind::Capsule)) CheckSphereCapsule(child, other);
		else if (IsPair(ck, ok, Collider::Kind::Box, Collider::Kind::Capsule)) CheckBoxCapsule(child, other);
		else if (IsPair(ck, ok, Collider::Kind::Sphere, Collider::Kind::HalfPlane)) {
			Collider* sp = (ck == Collider::Kind::Sphere) ? child : other;
			Collider* hp = (ck == Collider::Kind::HalfPlane) ? child : other;
			CheckSphereHalfPlane(sp, hp);
		}
		else if (IsPair(ck, ok, Collider::Kind::Box, Collider::Kind::HalfPlane)) {
			Collider* bx = (ck == Collider::Kind::Box) ? child : other;
			Collider* hp = (ck == Collider::Kind::HalfPlane) ? child : other;
			CheckBoxHalfPlane(bx, hp);
		}
		else if (IsPair(ck, ok, Collider::Kind::Capsule, Collider::Kind::HalfPlane)) {
			Collider* cp = (ck == Collider::Kind::Capsule) ? child : other;
			Collider* hp = (ck == Collider::Kind::HalfPlane) ? child : other;
			CheckCapsuleHalfPlane(cp, hp);
		}
		else if (IsPair(ck, ok, Collider::Kind::Sphere, Collider::Kind::Mesh)) {
			Collider* sp = (ck == Collider::Kind::Sphere) ? child : other;
			Collider* ms = (ck == Collider::Kind::Mesh) ? child : other;
			CheckSphereMesh(sp, ms);
		}
		else if (IsPair(ck, ok, Collider::Kind::Capsule, Collider::Kind::Mesh)) {
			Collider* cp = (ck == Collider::Kind::Capsule) ? child : other;
			Collider* ms = (ck == Collider::Kind::Mesh) ? child : other;
			CheckCapsuleMesh(cp, ms);
		}
		else if (IsPair(ck, ok, Collider::Kind::Box, Collider::Kind::Mesh)) {
			Collider* bx = (ck == Collider::Kind::Box) ? child : other;
			Collider* ms = (ck == Collider::Kind::Mesh) ? child : other;
			CheckBoxMesh(bx, ms);
		}

		if (_tlNarrowHit) anyHit = true;
	});

	_tlNarrowHit = anyHit;
}

// 1フレームぶんのコライダ更新入口。
void ColliderManager::Update(float dtSec) {
	if (IsShuttingDown()) {
		return;
	}
	#ifdef _DEBUG
	auto _s = PerformanceMonitor::Instance().Scope("Collider.UpdateFrame");
	#endif
	_deltaTimeSec = (dtSec > 1e-6f) ? dtSec : 1e-6f;

	// Adaptive cell size: 毎フレーム計算すると高負荷なので、30フレームごとに更新
	static int frameCounter = 0;
	if (_adaptiveCellSize && (++frameCounter % 30 == 0)) {
		#ifdef _DEBUG
		auto _cs = PerformanceMonitor::Instance().Scope("Collider.ComputeAdaptiveCellSize");
		#endif
		ComputeAdaptiveCellSize();
	}
	UpdateAllShapes();
	CheckDetailedCollisions();
}

// セルサイズの自動調整。大まかな目安として、コライダの半径や半辺の中央値を基にする。
// 極端に小さい/大きいコライダは無視して、平均的なサイズを取る。
void ColliderManager::ComputeAdaptiveCellSize() {
	auto& snapshot = _snapshotBuf;
	{
		std::lock_guard lk(_mtx);
		if (_colliders.empty()) return;
		snapshot.assign(_colliders.begin(), _colliders.end());
	}
	float totalExtent = 0.0f;
	int count = 0;
	for (auto* c : snapshot) {
		if (!c) continue;
		const AABB& aabb = c->GetAABB();
		const float ex = aabb.max.x - aabb.min.x;
		const float ey = aabb.max.y - aabb.min.y;
		const float ez = aabb.max.z - aabb.min.z;
		const float maxE = (std::max)({ex, ey, ez});
		if (maxE > 1e-4f && maxE < 1e5f) {
			totalExtent += maxE;
			++count;
		}
	}
	if (count > 0) {
		float avg = totalExtent / static_cast<float>(count);
		// Cell size = 2x average extent, clamped
		float newSize = std::clamp(avg * 2.0f, 1.0f, 32.0f);
		_cellSize = newSize;
	}
}

// Sphere-Sphere 詳細判定。
// 距離二乗と半径和二乗の比較で sqrt を避ける。
// 離れている場合でも CCD 条件を満たせばスイープテストで補う。
void ColliderManager::CheckSphereSphere(Collider* a, Collider* b) {
	auto* sa = dynamic_cast<SphereCollider*>(a);
	auto* sb = dynamic_cast<SphereCollider*>(b);
	if (!sa || !sb) {
		_tlNarrowHit = false;
		return;
	}
	const VECTOR ca = sa->GetCenter();
	const VECTOR cb = sb->GetCenter();
	const VECTOR d = VSub(cb, ca);
	const float r = sa->GetRadius() + sb->GetRadius();
	if (LenSq(d) <= r * r) {
		_tlNarrowHit = true;
		return;
	}

	// CCD フォールバック: 前フレーム位置からのスイープで通過判定
	if (!sa->hasPrevAABB || !sb->hasPrevAABB) {
		_tlNarrowHit = false;
		return;
	}

	const VECTOR prevA = sa->prevAABB.center;
	const VECTOR prevB = sb->prevAABB.center;
	const VECTOR moveA = VSub(ca, prevA);
	const VECTOR moveB = VSub(cb, prevB);
	float dt = _deltaTimeSec;
	if (dt < 1e-6f) dt = 1e-6f;
	const float speedSqA = LenSq(moveA) / (dt * dt);
	const float speedSqB = LenSq(moveB) / (dt * dt);
	const float thrA = sa->ccdDistanceThreshold;
	const float thrB = sb->ccdDistanceThreshold;
	const bool useSweep = sa->enableCCD || sb->enableCCD || speedSqA > thrA * thrA || speedSqB > thrB * thrB;
	if (!useSweep) {
		_tlNarrowHit = false;
		return;
	}

	// 相対移動として球-球スイープ: A を固定し B の相対移動で判定
	const VECTOR relPrev = VSub(prevB, prevA);
	const VECTOR relCurr = VSub(cb, ca);
	const VECTOR relDir = VSub(relCurr, relPrev);
	const float a2 = LenSq(relDir);
	if (a2 < 1e-8f) {
		_tlNarrowHit = false;
		return;
	}
	const float b2 = 2.0f * Dot3(relPrev, relDir);
	const float c2 = LenSq(relPrev) - r * r;
	const float disc = b2 * b2 - 4.0f * a2 * c2;
	if (disc < 0.0f) {
		_tlNarrowHit = false;
		return;
	}
	const float sqrtDisc = std::sqrt(disc);
	const float t0 = (-b2 - sqrtDisc) / (2.0f * a2);
	_tlNarrowHit = (t0 >= 0.0f && t0 <= 1.0f);
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
		_tlNarrowHit = false;
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
	_tlNarrowHit = (LenSq(diff) <= r * r);
	if (_tlNarrowHit) return;

	if (!s->hasPrevAABB) return;

	const VECTOR prevCenter = s->prevAABB.center;
	const VECTOR move = VSub(c, prevCenter);
	const float distSq = LenSq(move);
	float dt = _deltaTimeSec;
	if (dt < 1e-6f) dt = 1e-6f;
	const float speedSq = distSq / (dt * dt);
	const float thr = s->ccdDistanceThreshold;
	const bool useSweep = s->enableCCD || box->enableCCD || speedSq > thr * thr;
	if (!useSweep) return;

	_tlNarrowHit = SweepSphereAgainstBox(prevCenter, c, r, box, nullptr, nullptr, nullptr);
}

// Box-Box 詳細判定。
// SAT により「分離軸が1本でもあれば非衝突」と判定する。
void ColliderManager::CheckBoxBox(Collider* a, Collider* b) {
	BoxCollider* ba = dynamic_cast<BoxCollider*>(a);
	BoxCollider* bb = dynamic_cast<BoxCollider*>(b);
	if (!ba || !bb) {
		_tlNarrowHit = false;
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
		if (std::fabs(t[i]) > ra + rb) { _tlNarrowHit = false; return; }
	}

	for (int j =0; j <3; ++j) {
		ra = aExt[0] * AbsR[0][j] + aExt[1] * AbsR[1][j] + aExt[2] * AbsR[2][j];
		rb = bExt[j];
		tval = std::fabs(t[0] * R[0][j] + t[1] * R[1][j] + t[2] * R[2][j]);
		if (tval > ra + rb) { _tlNarrowHit = false; return; }
	}

	ra = aExt[1] * AbsR[2][0] + aExt[2] * AbsR[1][0];
	rb = bExt[1] * AbsR[0][2] + bExt[2] * AbsR[0][1];
	tval = std::fabs(t[2] * R[1][0] - t[1] * R[2][0]);
	if (tval > ra + rb) { _tlNarrowHit = false; return; }

	ra = aExt[1] * AbsR[2][1] + aExt[2] * AbsR[1][1];
	rb = bExt[0] * AbsR[0][2] + bExt[2] * AbsR[0][0];
	tval = std::fabs(t[2] * R[1][1] - t[1] * R[2][1]);
	if (tval > ra + rb) { _tlNarrowHit = false; return; }

	ra = aExt[1] * AbsR[2][2] + aExt[2] * AbsR[1][2];
	rb = bExt[0] * AbsR[0][1] + bExt[1] * AbsR[0][0];
	tval = std::fabs(t[2] * R[1][2] - t[1] * R[2][2]);
	if (tval > ra + rb) { _tlNarrowHit = false; return; }

	ra = aExt[0] * AbsR[2][0] + aExt[2] * AbsR[0][0];
	rb = bExt[1] * AbsR[1][2] + bExt[2] * AbsR[1][1];
	tval = std::fabs(t[0] * R[2][0] - t[2] * R[0][0]);
	if (tval > ra + rb) { _tlNarrowHit = false; return; }

	ra = aExt[0] * AbsR[2][1] + aExt[2] * AbsR[0][1];
	rb = bExt[0] * AbsR[1][2] + bExt[2] * AbsR[1][0];
	tval = std::fabs(t[0] * R[2][1] - t[2] * R[0][1]);
	if (tval > ra + rb) { _tlNarrowHit = false; return; }

	ra = aExt[0] * AbsR[2][2] + aExt[2] * AbsR[0][2];
	rb = bExt[0] * AbsR[1][1] + bExt[1] * AbsR[1][0];
	tval = std::fabs(t[0] * R[2][2] - t[2] * R[0][2]);
	if (tval > ra + rb) { _tlNarrowHit = false; return; }

	ra = aExt[0] * AbsR[1][0] + aExt[1] * AbsR[0][0];
	rb = bExt[1] * AbsR[2][2] + bExt[2] * AbsR[2][1];
	tval = std::fabs(t[1] * R[0][0] - t[0] * R[1][0]);
	if (tval > ra + rb) { _tlNarrowHit = false; return; }

	ra = aExt[0] * AbsR[1][1] + aExt[1] * AbsR[0][1];
	rb = bExt[0] * AbsR[2][2] + bExt[2] * AbsR[2][0];
	tval = std::fabs(t[1] * R[0][1] - t[0] * R[1][1]);
	if (tval > ra + rb) { _tlNarrowHit = false; return; }

	ra = aExt[0] * AbsR[1][2] + aExt[1] * AbsR[0][2];
	rb = bExt[0] * AbsR[2][1] + bExt[1] * AbsR[2][0];
	tval = std::fabs(t[1] * R[0][2] - t[0] * R[1][2]);
	if (tval > ra + rb) { _tlNarrowHit = false; return; }

	_tlNarrowHit = true;
}

// Capsule-Capsule 詳細判定。
// 線分間最近点距離と半径和の比較。
void ColliderManager::CheckCapsuleCapsule(Collider* a, Collider* b) {
	auto* ca = dynamic_cast<CapsuleCollider*>(a);
	auto* cb = dynamic_cast<CapsuleCollider*>(b);
	if (!ca || !cb) {
		_tlNarrowHit = false;
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

	_tlNarrowHit = (LenSq(diff) <= r * r);
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
		_tlNarrowHit = false;
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
	_tlNarrowHit = (LenSq(diff) <= r * r);
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
		_tlNarrowHit = false;
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
	_tlNarrowHit = (best <= r * r);
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
	std::lock_guard lk(_mtx);
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

	std::lock_guard lk(_mtx);
	{
		auto it = std::find(_colliders.begin(), _colliders.end(), collider);
		if (it != _colliders.end()) {
			_colliders.erase(it);
		}
	}

	collider->hasPrevAABB = false;

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
	// 双方向のレイヤー/マスク一致が成立していなければ衝突させない
	return !BitOperation::MutualMatch(a->layer, a->mask, b->layer, b->mask);
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

Collider* ColliderManager::FindColliderByOwner(GameObject* owner) const noexcept {
	if (!owner) return nullptr;
	for (auto* c : _colliders) {
		if (!c) continue;
		if (c->owner == owner) return c;
	}
	return nullptr;
}






