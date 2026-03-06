#include "BoxCollider.h"

#include <cmath>

#include "GameObject.h"
#include "DxLib.h"

namespace { // ユーティリティ関数群
	// ベクトルの各要素絶対値
	inline VECTOR AbsVec(const VECTOR& v) {
		return VGet(std::fabs(v.x), std::fabs(v.y), std::fabs(v.z));
	}
	// 3つのベクトル和
	inline VECTOR Add3(const VECTOR& a, const VECTOR& b, const VECTOR& c) {
		return VAdd(VAdd(a, b), c);
	}
	// 3次元ドット積
	inline float Dot3Local(const VECTOR& a, const VECTOR& b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}
	// ベクトル長
	inline float LenLocal(const VECTOR& v) {
		return std::sqrt(Dot3Local(v, v));
	}
	// ベクトル正規化（ゼロベクトル対策付き）
	inline VECTOR SafeNormalize(const VECTOR& v, const VECTOR& fallback) {
		const float len = LenLocal(v);
		if (len >1e-6f) return VScale(v,1.0f / len);
		return fallback;
	}
}

BoxCollider::BoxCollider() : Collider() {
	_box.center = VGet(0,0,0);
	_box.halfExtents = VGet(0.5f,0.5f,0.5f);
	_box.axisX = VGet(1,0,0);
	_box.axisY = VGet(0,1,0);
	_box.axisZ = VGet(0,0,1);
	UpdateShape();
}

BoxCollider::~BoxCollider() = default;

void BoxCollider::UpdateShape() {
	if (owner) {
		_box.center = owner->transform.WorldPosition();
		_box.axisX = owner->transform.Right();
		_box.axisY = owner->transform.Up();
		_box.axisZ = owner->transform.Forward();
	}

	_box.axisX = SafeNormalize(_box.axisX, VGet(1,0,0));
	_box.axisY = SafeNormalize(_box.axisY, VGet(0,1,0));
	_box.axisZ = SafeNormalize(_box.axisZ, VGet(0,0,1));

	const VECTOR ex = VScale(AbsVec(_box.axisX), _box.halfExtents.x);
	const VECTOR ey = VScale(AbsVec(_box.axisY), _box.halfExtents.y);
	const VECTOR ez = VScale(AbsVec(_box.axisZ), _box.halfExtents.z);
	const VECTOR ext = Add3(ex, ey, ez);

	_aabb.center = _box.center;
	_aabb.min = VSub(_box.center, ext);
	_aabb.max = VAdd(_box.center, ext);
}

void BoxCollider::DrawDebug() {
	const unsigned int col = isTrigger ? GetColor(255,220,80) : GetColor(80,200,200);

	auto Corner = [&](float sx, float sy, float sz) {
		VECTOR p = _box.center;
		p = VAdd(p, VScale(_box.axisX, _box.halfExtents.x * sx));
		p = VAdd(p, VScale(_box.axisY, _box.halfExtents.y * sy));
		p = VAdd(p, VScale(_box.axisZ, _box.halfExtents.z * sz));
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

	DrawSphere3D(_box.center,0.05f,8, col, col, TRUE);
}

void BoxCollider::DrawDebugAABB() {
	const unsigned int col = GetColor(120,120,120);
	const VECTOR mn = _aabb.min;
	const VECTOR mx = _aabb.max;

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

