#include "ColliderManager.h"

#include "GameObject.h"
#include "SceneManager.h"

#include <algorithm> // clamp, min/max
#include <cmath>
#include <cfloat>

#include "SphereCollider.h"
#include "BoxCollider.h"
#include "CapsuleCollider.h"
#include "Assert.h"

// ユーティリティ関数群
namespace {
	// AABB同士の当たり判定（ワールド座標系）
	inline bool IntersectAABBWorld(const AABB& a, const AABB& b) noexcept {
		return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
			(a.min.y <= b.max.y && a.max.y >= b.min.y) &&
			(a.min.z <= b.max.z && a.max.z >= b.min.z);
	}
	// ベクトル演算
	inline float Dot3(const VECTOR& a, const VECTOR& b) noexcept { return a.x * b.x + a.y * b.y + a.z * b.z; }
	// ベクトル長の二乗
	inline float LenSq(const VECTOR& v) noexcept { return Dot3(v, v); }
}

// 明示的終了処理
void ColliderManager::Shutdown() noexcept {
	// 多重呼び出し安全
	const bool wasShuttingDown = shuttingDown_.exchange(true, std::memory_order_relaxed);
	if (wasShuttingDown) {
		return;
	}

	// 終了中にデストラクタ経由で UnregisterCollider が呼ばれても
	// unordered_set / vector に触らないように、ここで空にしておく。
	currPairs_.clear();
	prevPairs_.clear();
	colliders_.clear();
	// narrowHit_ はローカル状態なので放置でOK
}

// イベントディスパッチ
void ColliderManager::DispatchEnter(Collider* a, Collider* b) {
	if (!a || !b) return;
	if (a->isTrigger || b->isTrigger) {
		a->OnTriggerEnter(b);
		b->OnTriggerEnter(a);
	}
	else {
		a->OnCollisionEnter(b);
		b->OnCollisionEnter(a);
	}
}

// イベントディスパッチ
void ColliderManager::DispatchStay(Collider* a, Collider* b) {
	if (!a || !b) return;
	if (a->isTrigger || b->isTrigger) {
		a->OnTriggerStay(b);
		b->OnTriggerStay(a);
	}
	else {
		a->OnCollisionStay(b);
		b->OnCollisionStay(a);
	}
}

// イベントディスパッチ
void ColliderManager::DispatchExit(Collider* a, Collider* b) {
	if (!a || !b) return;
	if (a->isTrigger || b->isTrigger) {
		a->OnTriggerExit(b);
		b->OnTriggerExit(a);
	}
	else {
		a->OnCollisionExit(b);
		b->OnCollisionExit(a);
	}
}

//	コライダー種別ペア判定
static bool IsPair(Collider::Kind a, Collider::Kind b, Collider::Kind x, Collider::Kind y) {
	return (a == x && b == y) || (a == y && b == x);
}

//形状更新
void ColliderManager::UpdateAllShapes() {
	for (auto* c : colliders_) {
		if (!c) continue;
		c->UpdateShape();
	}
}

// ペア構築
void ColliderManager::BuildCurrentPairs() {
	currPairs_.clear();

	// --- scene filtering (default ON) ---
	const int currentSceneId = SceneManager::Instance().CurrentSceneId();

	// owner が無いColliderはデフォルト除外（=どのシーンにも属さない）
	// グローバルに使いたい場合は useSceneFilter=false を設定する
	for (size_t i =0; i < colliders_.size(); ++i) {
		Collider* a = colliders_[i];
		if (!a) continue; // aが無効ならスキップ
		if (!a->owner) { if (a->useSceneFilter) continue; }
		else { if (a->useSceneFilter && a->owner->ownerSceneId != currentSceneId) continue; }

		// Active / Sleep 判定（最序盤）
		if (!a->IsEnabled()) continue;
		if (a->owner && !a->owner->IsActive()) continue;

		for (size_t j = i +1; j < colliders_.size(); ++j) {
			Collider* b = colliders_[j];
			if (!b) continue; // bが無効ならスキップ
			if (!b->owner) { if (b->useSceneFilter) continue; }
			else { if (b->useSceneFilter && b->owner->ownerSceneId != currentSceneId) continue; }

			// Active / Sleep 判定（最序盤）
			if (!b->IsEnabled()) continue;
			if (b->owner && !b->owner->IsActive()) continue;

			// 空間分割判定
			//if (!SpatialPartitioning()) continue; // 空間分割判定で外れている

			// Layer/Mask判定
			if (CheckLayerMaskCollisions(a, b)) continue; // Layer/Mask判定で外れている

			// AABB判定
			if (CheckAABBCollisions(a, b)) continue; // AABB判定で外れている

			// 詳細判定
			narrowHit_ = false; // 詳細判定結果初期化
			const auto ka = a->GetKind(); // aの種別判定
			const auto kb = b->GetKind(); // bの種別判定
			if (IsPair(ka, kb, Collider::Kind::Sphere, Collider::Kind::Sphere)) CheckSphereSphere(a, b);				// Sphere-Sphere
			else if (IsPair(ka, kb, Collider::Kind::Sphere, Collider::Kind::Box)) CheckSphereBox(a, b);					// Sphere-Box
			else if (IsPair(ka, kb, Collider::Kind::Box, Collider::Kind::Box)) CheckBoxBox(a, b);						// Box-Box
			else if (IsPair(ka, kb, Collider::Kind::Capsule, Collider::Kind::Capsule)) CheckCapsuleCapsule(a, b);		// Capsule-Capsule
			else if (IsPair(ka, kb, Collider::Kind::Sphere, Collider::Kind::Capsule)) CheckSphereCapsule(a, b);			// Sphere-Capsule
			else if (IsPair(ka, kb, Collider::Kind::Box, Collider::Kind::Capsule)) CheckBoxCapsule(a, b);				// Box-Capsule
			else {
				//ここでは実行時アサート（Debug時のみ停止）に置き換える。
				ASSERT_MSG(false, "未定義のコライダー組み合わせ: kindA=%d kindB=%d", static_cast<int>(ka), static_cast<int>(kb));
				narrowHit_ = false; // 安全側（衝突なし扱い）
			}

			if (!narrowHit_) continue; // 詳細判定で外れている

			currPairs_.insert(MakeKey(a, b)); // ペア登録

			// 押し戻し（Triggerは除外）
			if (!(a->isTrigger || b->isTrigger)) {
				ResolvePushOut(a, b); // 押し戻し処理
			}
		}
	}
}

// イベント処理
void ColliderManager::ProcessPairEvents() {
	// Enter/Stay
	for (const auto& k : currPairs_) {
		if (prevPairs_.contains(k)) {
			DispatchStay(k.a, k.b);
		}
		else {
			DispatchEnter(k.a, k.b);
		}
	}

	// Exit
	for (const auto& k : prevPairs_) {
		if (!currPairs_.contains(k)) {
			DispatchExit(k.a, k.b);
		}
	}

	prevPairs_ = currPairs_;
}

// 詳細判定
void ColliderManager::CheckDetailedCollisions() {
	BuildCurrentPairs(); // ペア構築
	ProcessPairEvents(); // イベント処理
}

// 押し戻し
void ColliderManager::ResolvePushOut(Collider* a, Collider* b) {
	// 最小実装：
	// - Sphere-Sphere : オーナーTransformを半々で押し戻し
	// - Sphere-Box(OBB): SphereのオーナーTransformを押し戻し（Boxは固定扱い）
	// ※ owner が無い場合は何もしない

	// Sphere-Sphere
	auto* sa = dynamic_cast<SphereCollider*>(a);
	auto* sb = dynamic_cast<SphereCollider*>(b);
	if (sa && sb) {
		GameObject* oa = static_cast<Collider*>(sa)->owner;
		GameObject* ob = static_cast<Collider*>(sb)->owner;
		if (!oa && !ob) return;

		const VECTOR ca = sa->GetCenter();
		const VECTOR cb = sb->GetCenter();
		const VECTOR d = VSub(cb, ca);
		const float dist = std::sqrt((std::max)(LenSq(d),1e-8f));
		const float r = sa->sphere_.radius + sb->sphere_.radius;
		const float pen = r - dist;
		if (pen <=0.0f) return;

		const VECTOR n = VScale(d,1.0f / dist);

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

		sa->UpdateShape();
		sb->UpdateShape();
		return;
	}

	// Sphere-Box
	SphereCollider* s = dynamic_cast<SphereCollider*>(a);
	BoxCollider* box = dynamic_cast<BoxCollider*>(b);
	if (!s || !box) {
		s = dynamic_cast<SphereCollider*>(b);
		box = dynamic_cast<BoxCollider*>(a);
	}
	if (s && box) {
		GameObject* os = static_cast<Collider*>(s)->owner;
		if (!os) return;

		// 最近点
		const VECTOR c = s->GetCenter();
		const VECTOR d = VSub(c, box->box_.center);
		float x = Dot3(d, box->box_.axisX);
		float y = Dot3(d, box->box_.axisY);
		float z = Dot3(d, box->box_.axisZ);
		x = std::clamp(x, -box->box_.halfExtents.x, box->box_.halfExtents.x);
		y = std::clamp(y, -box->box_.halfExtents.y, box->box_.halfExtents.y);
		z = std::clamp(z, -box->box_.halfExtents.z, box->box_.halfExtents.z);
		VECTOR closest = box->box_.center;
		closest = VAdd(closest, VScale(box->box_.axisX, x));
		closest = VAdd(closest, VScale(box->box_.axisY, y));
		closest = VAdd(closest, VScale(box->box_.axisZ, z));

		VECTOR diff = VSub(c, closest);
		float dist = std::sqrt((std::max)(LenSq(diff),1e-8f));
		float pen = s->sphere_.radius - dist;
		if (pen <=0.0f) return;

		VECTOR n = VScale(diff,1.0f / dist);

		VECTOR p = os->transform.LocalPosition();
		p = VAdd(p, VScale(n, pen));
		os->transform.SetLocalPosition(p);

		s->UpdateShape();
		return;
	}
}

// 更新
void ColliderManager::Update() {
	if (IsShuttingDown()) {
		return;
	}
	UpdateAllShapes();			//形状更新
	CheckDetailedCollisions();	// 詳細判定
}

// Sphere-Sphere 当たり判定
void ColliderManager::CheckSphereSphere(Collider* a, Collider* b) {
	auto* sa = dynamic_cast<SphereCollider*>(a);
	auto* sb = dynamic_cast<SphereCollider*>(b);
	if (!sa || !sb) {
		narrowHit_ = false;
		return;
	}
	const VECTOR ca = sa->sphere_.center;
	const VECTOR cb = sb->sphere_.center;
	const VECTOR d = VSub(cb, ca);
	const float r = sa->sphere_.radius + sb->sphere_.radius;
	narrowHit_ = (LenSq(d) <= r * r);
}

// Sphere-Box(OBB) 当たり判定
void ColliderManager::CheckSphereBox(Collider* a, Collider* b) {
	SphereCollider* s = dynamic_cast<SphereCollider*>(a);
	BoxCollider* box = dynamic_cast<BoxCollider*>(b);
	if (!s || !box) {
		//逆順の可能性
		s = dynamic_cast<SphereCollider*>(b);
		box = dynamic_cast<BoxCollider*>(a);
	}
	if (!s || !box) {
		narrowHit_ = false;
		return;
	}

	const VECTOR c = s->sphere_.center;
	const float r = s->sphere_.radius;

	// OBB近傍点を求める
	const VECTOR d = VSub(c, box->box_.center);
	float x = Dot3(d, box->box_.axisX);
	float y = Dot3(d, box->box_.axisY);
	float z = Dot3(d, box->box_.axisZ);

	x = std::clamp(x, -box->box_.halfExtents.x, box->box_.halfExtents.x);
	y = std::clamp(y, -box->box_.halfExtents.y, box->box_.halfExtents.y);
	z = std::clamp(z, -box->box_.halfExtents.z, box->box_.halfExtents.z);

	VECTOR closest = box->box_.center;
	closest = VAdd(closest, VScale(box->box_.axisX, x));
	closest = VAdd(closest, VScale(box->box_.axisY, y));
	closest = VAdd(closest, VScale(box->box_.axisZ, z));

	const VECTOR diff = VSub(c, closest);
	narrowHit_ = (LenSq(diff) <= r * r);
}

// Box-Box 当たり判定
void ColliderManager::CheckBoxBox(Collider* a, Collider* b) {
	BoxCollider* ba = dynamic_cast<BoxCollider*>(a);
	BoxCollider* bb = dynamic_cast<BoxCollider*>(b);
	if (!ba || !bb) {
		narrowHit_ = false;
		return;
	}

	// SAT: OBB vs OBB
	const VECTOR A0 = ba->box_.axisX;
	const VECTOR A1 = ba->box_.axisY;
	const VECTOR A2 = ba->box_.axisZ;
	const VECTOR B0 = bb->box_.axisX;
	const VECTOR B1 = bb->box_.axisY;
	const VECTOR B2 = bb->box_.axisZ;

	const VECTOR tV = VSub(bb->box_.center, ba->box_.center);
	// t in A's frame
	const float t[3] = { Dot3(tV, A0), Dot3(tV, A1), Dot3(tV, A2) };

	// rotation matrix R = A^T * B
	float R[3][3] = {
		{ Dot3(A0, B0), Dot3(A0, B1), Dot3(A0, B2) },
		{ Dot3(A1, B0), Dot3(A1, B1), Dot3(A1, B2) },
		{ Dot3(A2, B0), Dot3(A2, B1), Dot3(A2, B2) },
	};

	// abs(R) + epsilon
	const float eps =1e-6f;
	float AbsR[3][3];
	for (int i=0;i<3;++i) {
		for (int j=0;j<3;++j) {
			AbsR[i][j] = std::fabs(R[i][j]) + eps;
		}
	}

	const float aExt[3] = { ba->box_.halfExtents.x, ba->box_.halfExtents.y, ba->box_.halfExtents.z };
	const float bExt[3] = { bb->box_.halfExtents.x, bb->box_.halfExtents.y, bb->box_.halfExtents.z };

	float ra, rb;
	float tval;

	// Axes A0,A1,A2
	for (int i=0;i<3;++i) {
		ra = aExt[i];
		rb = bExt[0]*AbsR[i][0] + bExt[1]*AbsR[i][1] + bExt[2]*AbsR[i][2];
		if (std::fabs(t[i]) > ra + rb) { narrowHit_ = false; return; }
	}

	// Axes B0,B1,B2
	for (int j=0;j<3;++j) {
		ra = aExt[0]*AbsR[0][j] + aExt[1]*AbsR[1][j] + aExt[2]*AbsR[2][j];
		rb = bExt[j];
		tval = std::fabs(t[0]*R[0][j] + t[1]*R[1][j] + t[2]*R[2][j]);
		if (tval > ra + rb) { narrowHit_ = false; return; }
	}

	// Axes A_i x B_j
	// A0 x B0
	ra = aExt[1]*AbsR[2][0] + aExt[2]*AbsR[1][0];
	rb = bExt[1]*AbsR[0][2] + bExt[2]*AbsR[0][1];
	tval = std::fabs(t[2]*R[1][0] - t[1]*R[2][0]);
	if (tval > ra + rb) { narrowHit_ = false; return; }

	// A0 x B1
	ra = aExt[1]*AbsR[2][1] + aExt[2]*AbsR[1][1];
	rb = bExt[0]*AbsR[0][2] + bExt[2]*AbsR[0][0];
	tval = std::fabs(t[2]*R[1][1] - t[1]*R[2][1]);
	if (tval > ra + rb) { narrowHit_ = false; return; }

	// A0 x B2
	ra = aExt[1]*AbsR[2][2] + aExt[2]*AbsR[1][2];
	rb = bExt[0]*AbsR[0][1] + bExt[1]*AbsR[0][0];
	tval = std::fabs(t[2]*R[1][2] - t[1]*R[2][2]);
	if (tval > ra + rb) { narrowHit_ = false; return; }

	// A1 x B0
	ra = aExt[0]*AbsR[2][0] + aExt[2]*AbsR[0][0];
	rb = bExt[1]*AbsR[1][2] + bExt[2]*AbsR[1][1];
	tval = std::fabs(t[0]*R[2][0] - t[2]*R[0][0]);
	if (tval > ra + rb) { narrowHit_ = false; return; }

	// A1 x B1
	ra = aExt[0]*AbsR[2][1] + aExt[2]*AbsR[0][1];
	rb = bExt[0]*AbsR[1][2] + bExt[2]*AbsR[1][0];
	tval = std::fabs(t[0]*R[2][1] - t[2]*R[0][1]);
	if (tval > ra + rb) { narrowHit_ = false; return; }

	// A1 x B2
	ra = aExt[0]*AbsR[2][2] + aExt[2]*AbsR[0][2];
	rb = bExt[0]*AbsR[1][1] + bExt[1]*AbsR[1][0];
	tval = std::fabs(t[0]*R[2][2] - t[2]*R[0][2]);
	if (tval > ra + rb) { narrowHit_ = false; return; }

	// A2 x B0
	ra = aExt[0]*AbsR[1][0] + aExt[1]*AbsR[0][0];
	rb = bExt[1]*AbsR[2][2] + bExt[2]*AbsR[2][1];
	tval = std::fabs(t[1]*R[0][0] - t[0]*R[1][0]);
	if (tval > ra + rb) { narrowHit_ = false; return; }

	// A2 x B1
	ra = aExt[0]*AbsR[1][1] + aExt[1]*AbsR[0][1];
	rb = bExt[0]*AbsR[2][2] + bExt[2]*AbsR[2][0];
	tval = std::fabs(t[1]*R[0][1] - t[0]*R[1][1]);
	if (tval > ra + rb) { narrowHit_ = false; return; }

	// A2 x B2
	ra = aExt[0]*AbsR[1][2] + aExt[1]*AbsR[0][2];
	rb = bExt[0]*AbsR[2][1] + bExt[1]*AbsR[2][0];
	tval = std::fabs(t[1]*R[0][2] - t[0]*R[1][2]);
	if (tval > ra + rb) { narrowHit_ = false; return; }

	narrowHit_ = true;
}

// Capsule-Capsule 当たり判定
void ColliderManager::CheckCapsuleCapsule(Collider* a, Collider* b) {
	auto* ca = dynamic_cast<CapsuleCollider*>(a);
	auto* cb = dynamic_cast<CapsuleCollider*>(b);
	if (!ca || !cb) {
		narrowHit_ = false;
		return;
	}

	const VECTOR p1 = ca->cap_.bottom;
	const VECTOR q1 = ca->cap_.top;
	const VECTOR p2 = cb->cap_.bottom;
	const VECTOR q2 = cb->cap_.top;
	const float r = ca->cap_.radius + cb->cap_.radius;

	// 線分-線分の最近距離（二乗）
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
	} else {
		//ほぼ平行：とりあえず始点側に寄せる
		s =0.0f;
	}

	// t を sから求める
	const float tNom = a12 * s + b2;
	if (a22 >1e-6f) {
		t = tNom / a22;
		t = std::clamp(t,0.0f,1.0f);
	} else {
		t =0.0f;
	}

	// s を tで再調整（端点クランプ後の補正）
	const float sNom = a12 * t - b1;
	if (a11 >1e-6f) {
		s = sNom / a11;
		s = std::clamp(s,0.0f,1.0f);
	} else {
		s =0.0f;
	}

	const VECTOR c1 = VAdd(p1, VScale(d1, s));
	const VECTOR c2 = VAdd(p2, VScale(d2, t));
	const VECTOR diff = VSub(c1, c2);

	narrowHit_ = (LenSq(diff) <= r * r);
}

// Sphere-Capsule 当たり判定
void ColliderManager::CheckSphereCapsule(Collider* a, Collider* b) {
	SphereCollider* s = dynamic_cast<SphereCollider*>(a);
	CapsuleCollider* c = dynamic_cast<CapsuleCollider*>(b);
	if (!s || !c) {
		//逆順の可能性
		s = dynamic_cast<SphereCollider*>(b);
		c = dynamic_cast<CapsuleCollider*>(a);
	}
	if (!s || !c) {
		narrowHit_ = false;
		return;
	}

	const VECTOR p = c->cap_.bottom;
	const VECTOR q = c->cap_.top;
	const VECTOR seg = VSub(q, p);
	const VECTOR v = VSub(s->sphere_.center, p);

	const float segLenSq = Dot3(seg, seg);
	float t =0.0f;
	if (segLenSq >1e-6f) {
		t = Dot3(v, seg) / segLenSq;
		t = std::clamp(t,0.0f,1.0f);
	}

	const VECTOR closest = VAdd(p, VScale(seg, t));
	const VECTOR diff = VSub(s->sphere_.center, closest);
	const float r = s->sphere_.radius + c->cap_.radius;
	narrowHit_ = (LenSq(diff) <= r * r);
}

// Box(OBB)-Capsule 当たり判定
void ColliderManager::CheckBoxCapsule(Collider* a, Collider* b) {
	BoxCollider* box = dynamic_cast<BoxCollider*>(a);
	CapsuleCollider* cap = dynamic_cast<CapsuleCollider*>(b);
	if (!box || !cap) {
		//逆順の可能性
		box = dynamic_cast<BoxCollider*>(b);
		cap = dynamic_cast<CapsuleCollider*>(a);
	}
	if (!box || !cap) {
		narrowHit_ = false;
		return;
	}

	//1) Capsule 軸（線分）を OBB ローカル空間へ
	// OBBローカル: 原点=box.center, 軸=axisX/Y/Z
	auto ToLocal = [&](const VECTOR& w) {
		const VECTOR d = VSub(w, box->box_.center);
		return VGet(Dot3(d, box->box_.axisX), Dot3(d, box->box_.axisY), Dot3(d, box->box_.axisZ));
	};

	const VECTOR pL = ToLocal(cap->cap_.bottom);
	const VECTOR qL = ToLocal(cap->cap_.top);
	const VECTOR dL = VSub(qL, pL);

	const float hx = box->box_.halfExtents.x;
	const float hy = box->box_.halfExtents.y;
	const float hz = box->box_.halfExtents.z;

	//2) 線分とAABB（ローカルで軸整列箱）の最近距離^2
	// アルゴリズム:クリップして内点があれば0、なければ線分上の最近点候補を評価
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

	//2-1)端点候補
	{
		const VECTOR cp = ClampPointToAABB(pL);
		best = (std::min)(best, DistSq(pL, cp));
		const VECTOR cq = ClampPointToAABB(qL);
		best = (std::min)(best, DistSq(qL, cq));
	}

	//2-2)交差（内部）チェック + 面への射影候補
	// 各スラブ境界 t を計算し、線分上の点を評価
	auto ConsiderT = [&](float t) {
		if (t <0.0f || t >1.0f) return;
		const VECTOR s = VAdd(pL, VScale(dL, t));
		const VECTOR cs = ClampPointToAABB(s);
		best = (std::min)(best, DistSq(s, cs));
	};

	// x = +/-hx
	if (std::fabs(dL.x) >1e-6f) {
		ConsiderT((-hx - pL.x) / dL.x);
		ConsiderT(( hx - pL.x) / dL.x);
	}
	// y = +/-hy
	if (std::fabs(dL.y) >1e-6f) {
		ConsiderT((-hy - pL.y) / dL.y);
		ConsiderT(( hy - pL.y) / dL.y);
	}
	// z = +/-hz
	if (std::fabs(dL.z) >1e-6f) {
		ConsiderT((-hz - pL.z) / dL.z);
		ConsiderT(( hz - pL.z) / dL.z);
	}

	//3) 判定（capsule半径）
	const float r = cap->cap_.radius;
	narrowHit_ = (best <= r * r);
}

// デバッグ描画
void ColliderManager::DrawDebugAll() {
	for (auto* collider : colliders_) {
		if (!collider) continue;
		collider->DrawDebug();
	}
}

// AABBデバッグ描画
void ColliderManager::DrawDebugAABBAll() {
	for (auto* collider : colliders_) {
		if (!collider) continue;
		collider->DrawDebugAABB();
	}
}

// Colliderの登録
void ColliderManager::RegisterCollider(Collider* collider) {
	if (IsShuttingDown()) {
		return;
	}
	if (!collider) return;
	if (std::find(colliders_.begin(), colliders_.end(), collider) != colliders_.end()) return; //既に登録済み
	colliders_.push_back(collider);
}

// Colliderの登録解除
void ColliderManager::UnregisterCollider(Collider* collider) {
	if (IsShuttingDown()) {
		return;
	}
	if (!collider) return;

	// colliders_から削除（存在する時だけ）
	{
		auto it = std::find(colliders_.begin(), colliders_.end(), collider);
		if (it != colliders_.end()) {
			colliders_.erase(it);
		}
	}

	// ペア状態からも除去（unordered_setなので iterator erase は安全）
	for (auto it = prevPairs_.begin(); it != prevPairs_.end();) {
		if (it->a == collider || it->b == collider) {
			it = prevPairs_.erase(it);
			continue;
		}
		++it;
	}
	for (auto it = currPairs_.begin(); it != currPairs_.end();) {
		if (it->a == collider || it->b == collider) {
			it = currPairs_.erase(it);
			continue;
		}
		++it;
	}
}

// 空間分割
bool ColliderManager::SpatialPartitioning() {
	// TODO: SpatialHash等
	return true;
}

// Layer/Maskで判定
bool ColliderManager::CheckLayerMaskCollisions(Collider* a, Collider* b) {
	// Layer/Mask チェック
	if ((a->layer & b->mask) == 0) return true; // aのlayerとbのmaskが判定非通過
	if ((b->layer & a->mask) == 0) return true; // bのlayerとaのmaskが判定非通過
	return false; // 判定通過
}

// AABBで判定
bool ColliderManager::CheckAABBCollisions(Collider* a, Collider* b) {
	return !IntersectAABBWorld(a->GetAABB(), b->GetAABB());
}

