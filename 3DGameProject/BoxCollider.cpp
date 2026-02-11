#include "BoxCollider.h"

#include <cmath>

#include "GameObject.h"

namespace { // ユーティリティ関数群
	// ベクトルの各要素絶対値
	inline VECTOR AbsVec(const VECTOR& v) {
		return VGet(std::fabs(v.x), std::fabs(v.y), std::fabs(v.z));
	}
	// 3つのベクトル和
	inline VECTOR Add3(const VECTOR& a, const VECTOR& b, const VECTOR& c) {
		return VAdd(VAdd(a,b),c);
	}
	// 3次元ドット積
	inline float Dot3Local(const VECTOR& a, const VECTOR& b) {
		return a.x*b.x + a.y*b.y + a.z*b.z;
	}
	// ベクトル長
	inline float LenLocal(const VECTOR& v) {
		return std::sqrt(Dot3Local(v,v));
	}
	// ベクトル正規化（ゼロベクトル対策付き）
	inline VECTOR SafeNormalize(const VECTOR& v, const VECTOR& fallback) {
		const float len = LenLocal(v);
		if (len >1e-6f) return VScale(v,1.0f / len);
		return fallback;
	}
}

// コンストラクタ
BoxCollider::BoxCollider() : Collider() {
	box_.center = VGet(0,0,0);
	box_.halfExtents = VGet(0.5f,0.5f,0.5f);
	box_.axisX = VGet(1,0,0);
	box_.axisY = VGet(0,1,0);
	box_.axisZ = VGet(0,0,1);
	UpdateShape();
}

// デストラクタ
BoxCollider::~BoxCollider() = default;

// 形状更新
void BoxCollider::UpdateShape() {
	// owner Transformから中心/軸を算出（Transformが無い場合は保持値を使用）
	if (owner) {
		box_.center = owner->transform.WorldPosition();
		box_.axisX = owner->transform.Right();
		box_.axisY = owner->transform.Up();
		box_.axisZ = owner->transform.Forward();
	}

	// axis を正規化（Transform連動時に重要）
	box_.axisX = SafeNormalize(box_.axisX, VGet(1,0,0));
	box_.axisY = SafeNormalize(box_.axisY, VGet(0,1,0));
	box_.axisZ = SafeNormalize(box_.axisZ, VGet(0,0,1));

	// OBB を内包する AABB を作成
	// ext = |axisX|*hx + |axisY|*hy + |axisZ|*hz
	const VECTOR ex = VScale(AbsVec(box_.axisX), box_.halfExtents.x);
	const VECTOR ey = VScale(AbsVec(box_.axisY), box_.halfExtents.y);
	const VECTOR ez = VScale(AbsVec(box_.axisZ), box_.halfExtents.z);
	const VECTOR ext = Add3(ex, ey, ez);

	aabb_.center = box_.center;
	aabb_.min = VSub(box_.center, ext);
	aabb_.max = VAdd(box_.center, ext);
}

// 本体デバッグ描画の実装
void BoxCollider::DrawDebug() {
	// OBB ワイヤーフレーム描画
	const unsigned int col = isTrigger ? GetColor(255,220,80) : GetColor(80,200,200);

	auto Corner = [&](float sx, float sy, float sz) {
		VECTOR p = box_.center;
		p = VAdd(p, VScale(box_.axisX, box_.halfExtents.x * sx));
		p = VAdd(p, VScale(box_.axisY, box_.halfExtents.y * sy));
		p = VAdd(p, VScale(box_.axisZ, box_.halfExtents.z * sz));
		return p;
	};

	const VECTOR p000 = Corner(-1,-1,-1);
	const VECTOR p001 = Corner(-1,-1,1);
	const VECTOR p010 = Corner(-1,1,-1);
	const VECTOR p011 = Corner(-1,1,1);
	const VECTOR p100 = Corner(1,-1,-1);
	const VECTOR p101 = Corner(1,-1,1);
	const VECTOR p110 = Corner(1,1,-1);
	const VECTOR p111 = Corner(1,1,1);

	DrawLine3D(p000, p001, col);
	DrawLine3D(p001, p011, col);
	DrawLine3D(p011, p010, col);
	DrawLine3D(p010, p000, col);

	DrawLine3D(p100, p101, col);
	DrawLine3D(p101, p111, col);
	DrawLine3D(p111, p110, col);
	DrawLine3D(p110, p100, col);

	DrawLine3D(p000, p100, col);
	DrawLine3D(p001, p101, col);
	DrawLine3D(p010, p110, col);
	DrawLine3D(p011, p111, col);

	DrawSphere3D(box_.center,0.05f,8, col, col, TRUE);
}

// AABBデバッグ描画の実装
void BoxCollider::DrawDebugAABB() {
	const unsigned int col = GetColor(120,120,120);
	const VECTOR mn = aabb_.min;
	const VECTOR mx = aabb_.max;

	const VECTOR p000 = VGet(mn.x,mn.y,mn.z);
	const VECTOR p001 = VGet(mn.x,mn.y,mx.z);
	const VECTOR p010 = VGet(mn.x,mx.y,mn.z);
	const VECTOR p011 = VGet(mn.x,mx.y,mx.z);
	const VECTOR p100 = VGet(mx.x,mn.y,mn.z);
	const VECTOR p101 = VGet(mx.x,mn.y,mx.z);
	const VECTOR p110 = VGet(mx.x,mx.y,mn.z);
	const VECTOR p111 = VGet(mx.x,mx.y,mx.z);

	DrawLine3D(p000, p001, col);
	DrawLine3D(p001, p011, col);
	DrawLine3D(p011, p010, col);
	DrawLine3D(p010, p000, col);

	DrawLine3D(p100, p101, col);
	DrawLine3D(p101, p111, col);
	DrawLine3D(p111, p110, col);
	DrawLine3D(p110, p100, col);

	DrawLine3D(p000, p100, col);
	DrawLine3D(p001, p101, col);
	DrawLine3D(p010, p110, col);
	DrawLine3D(p011, p111, col);
}

