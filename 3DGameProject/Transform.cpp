#include "Transform.h"

// ヘルパー
namespace {
	inline VECTOR Vec(float x, float y, float z) noexcept { return VGet(x,y,z); }
}

// Transform 実装
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

// 親子関係
void Transform::SetParent(Transform* parent) noexcept {
	_parent = parent;
	MarkDirty();
}

// dirty: 行列キャッシュを古くする
void Transform::MarkDirty() noexcept {
	_localDirty = true;
	_worldDirty = true;
	// 子への伝播は、子リストを持つ設計に拡張した時に行う
}

// ローカル行列取得
const MATRIX& Transform::LocalMatrix() const {
	if (!_localDirty) return _localMatrix;
	_localDirty = false;

	// DxLib の行列ユーティリティを使用
	const MATRIX S = MGetScale(_localScale);
	const Quaternion q = Quaternion::FromEulerRad(_localEulerRad.x, _localEulerRad.y, _localEulerRad.z);
	const MATRIX R = q.ToRotationMatrix();
	const MATRIX T = MGetTranslate(_localPosition);

	// 組み立て順はプロジェクト内で統一（必要ならテストして入れ替える）
	_localMatrix = MMult(MMult(S, R), T);
	return _localMatrix;
}

// ワールド行列取得
const MATRIX& Transform::WorldMatrix() const {
	if (!_worldDirty) return _worldMatrix;
	_worldDirty = false;

	const MATRIX L = LocalMatrix();
	if (_parent) {
		_worldMatrix = MMult(L, _parent->WorldMatrix());
	} else {
		_worldMatrix = L;
	}
	return _worldMatrix;
}

// ワールド位置取得
VECTOR Transform::WorldPosition() const {
	const MATRIX& W = WorldMatrix();
	return VGet(W.m[3][0], W.m[3][1], W.m[3][2]);
}
