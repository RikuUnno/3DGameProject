#include "Transform.h"

namespace {
	inline VECTOR Vec(float x, float y, float z) noexcept { return VGet(x,y,z); }
}

Transform::Transform() {
	_localPosition = Vec(0,0,0);
	_localEulerRad = Vec(0,0,0);
	_localScale = Vec(1,1,1);
	_localMatrix = MGetIdent();
	_worldMatrix = MGetIdent();
}

void Transform::SetLocalPosition(const VECTOR& p) noexcept {
	_localPosition = p;
	MarkDirty();
}

void Transform::SetLocalEulerRad(const VECTOR& eulerRad) noexcept {
	_localEulerRad = eulerRad;
	MarkDirty();
}

void Transform::SetLocalScale(const VECTOR& s) noexcept {
	_localScale = s;
	MarkDirty();
}

void Transform::SetParent(Transform* parent) noexcept {
	_parent = parent;
	MarkDirty();
}

void Transform::MarkDirty() noexcept {
	_localDirty = true;
	_worldDirty = true;
	// 子への伝播は、子リストを持つ設計に拡張した時に行う
}

const MATRIX& Transform::LocalMatrix() const {
	if (!_localDirty) return _localMatrix;
	_localDirty = false;

	const MATRIX S = MakeScale(_localScale);
	const Quaternion q = Quaternion::FromEulerRad(_localEulerRad.x, _localEulerRad.y, _localEulerRad.z);
	const MATRIX R = q.ToRotationMatrix();
	const MATRIX T = MakeTranslation(_localPosition);

	//まずは S*R*Tで統一（DxLib行列の扱いは運用でテストして調整）
	_localMatrix = Mul(Mul(S, R), T);
	return _localMatrix;
}

const MATRIX& Transform::WorldMatrix() const {
	if (!_worldDirty) return _worldMatrix;
	_worldDirty = false;

	const MATRIX L = LocalMatrix();
	if (_parent) {
		_worldMatrix = Mul(L, _parent->WorldMatrix());
	} else {
		_worldMatrix = L;
	}
	return _worldMatrix;
}

VECTOR Transform::WorldPosition() const {
	const MATRIX& W = WorldMatrix();
	return VGet(W.m[3][0], W.m[3][1], W.m[3][2]);
}

MATRIX Transform::MakeScale(const VECTOR& s) {
	MATRIX m = MGetIdent();
	m.m[0][0] = s.x;
	m.m[1][1] = s.y;
	m.m[2][2] = s.z;
	return m;
}

MATRIX Transform::MakeTranslation(const VECTOR& p) {
	MATRIX m = MGetIdent();
	m.m[3][0] = p.x;
	m.m[3][1] = p.y;
	m.m[3][2] = p.z;
	return m;
}

MATRIX Transform::Mul(const MATRIX& a, const MATRIX& b) {
	MATRIX r{};
	for (int i =0; i <4; ++i) {
		for (int j =0; j <4; ++j) {
			r.m[i][j] =0.0f;
			for (int k =0; k <4; ++k) {
				r.m[i][j] += a.m[i][k] * b.m[k][j];
			}
		}
	}
	return r;
}
