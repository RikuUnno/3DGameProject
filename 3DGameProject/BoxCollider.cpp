#include "BoxCollider.h"

#include <algorithm>
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
		const VECTOR worldScale = owner->transform.WorldScale();
		// _box.center は owner基準のローカル中心。
		// 親の回転・スケール込みでワールドへ変換する。
		_center = owner->transform.TransformPoint(_box.center);
		// OBB の軸は owner のワールド軸をそのまま使うことで、
		// 親の回転を受けた子オブジェクトでも向きが一致する。
		_axisX = owner->transform.Right();
		_axisY = owner->transform.Up();
		_axisZ = owner->transform.Forward();
		// 半サイズは親スケール込みのワールド半サイズに変換する。
		_halfExtents = VGet(
			std::fabs(_box.halfExtents.x * worldScale.x),
			std::fabs(_box.halfExtents.y * worldScale.y),
			std::fabs(_box.halfExtents.z * worldScale.z)
		);
	}
	else {
		_center = _box.center;
		_axisX = _box.axisX;
		_axisY = _box.axisY;
		_axisZ = _box.axisZ;
		_halfExtents = _box.halfExtents;
	}

	_axisX = SafeNormalize(_axisX, VGet(1,0,0));
	_axisY = SafeNormalize(_axisY, VGet(0,1,0));
	_axisZ = SafeNormalize(_axisZ, VGet(0,0,1));

	// OBB を包むAABBは、各ワールド軸ベクトルの絶対値成分を使って求める。
	const VECTOR ex = VScale(AbsVec(_axisX), _halfExtents.x);
	const VECTOR ey = VScale(AbsVec(_axisY), _halfExtents.y);
	const VECTOR ez = VScale(AbsVec(_axisZ), _halfExtents.z);
	const VECTOR ext = Add3(ex, ey, ez);

	_aabb.center = _center;
	_aabb.min = VSub(_center, ext);
	_aabb.max = VAdd(_center, ext);
}

void BoxCollider::DrawDebug() {
	const unsigned int defaultCol = isTrigger ? GetColor(255,220,80) : GetColor(80,200,200);
	const unsigned int col = DebugColor() != 0 ? DebugColor() : defaultCol;

	auto Corner = [&](float sx, float sy, float sz) {
		VECTOR p = _center;
		p = VAdd(p, VScale(_axisX, _halfExtents.x * sx));
		p = VAdd(p, VScale(_axisY, _halfExtents.y * sy));
		p = VAdd(p, VScale(_axisZ, _halfExtents.z * sz));
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

	DrawSphere3D(_center,0.05f,8, col, col, TRUE);
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

