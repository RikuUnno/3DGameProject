#include "Transform.h"
#include <algorithm>
#include <cmath>

// ヘルパー
namespace {
	inline VECTOR Vec(float x, float y, float z) noexcept { return VGet(x, y, z); }
	inline float Len3Local(const VECTOR& v) noexcept {
		return std::sqrt((std::max)(v.x * v.x + v.y * v.y + v.z * v.z, 0.0f));
	}
	inline VECTOR SafeNormalizeLocal(const VECTOR& v, const VECTOR& fallback) noexcept {
		const float len = Len3Local(v);
		if (len > 1e-6f) return VScale(v, 1.0f / len);
		return fallback;
	}
}

// Transform 生成
Transform::Transform() {
	_localPosition = Vec(0,0,0);
	_localRotation = Quaternion::Identity();
	_localScale = Vec(1,1,1);
	_localMatrix = MGetIdent();
	_worldMatrix = MGetIdent();
}

Transform::~Transform() {
	// 親子リスト相互参照が残らないように切り離す
	SetParent(nullptr);
	for (auto* c : _children) {
		if (c) c->_parent = nullptr;
	}
	_children.clear();
}

void Transform::SetLocalPosition(const VECTOR& p) noexcept {
	_localPosition = p;
	MarkDirty();
}

void Transform::SetLocalScale(const VECTOR& s) noexcept {
	_localScale = s;
	MarkDirty();
}

VECTOR Transform::LocalEulerRad() const noexcept {
	// デバッグ表示やGUI入力用
	return _localRotation.ToEulerRad();
}

void Transform::SetLocalEulerRad(const VECTOR& eulerRad) noexcept {
	// Euler入力を内部回転（Quaternion）へ反映
	_localRotation = Quaternion::FromEulerRad(eulerRad.x, eulerRad.y, eulerRad.z);
	MarkDirty();
}

void Transform::SetLocalRotation(const Quaternion& q) noexcept {
	_localRotation = q.Normalized();
	MarkDirty();
}

// 親子関係
void Transform::SetParent(Transform* parent) noexcept {
	if (_parent == parent) {
		return;
	}

	// 旧親から除去
	if (_parent) {
		_parent->RemoveChild(this);
	}

	_parent = parent;

	// 新親へ登録
	if (_parent) {
		_parent->AddChild(this);
	}

	MarkDirty();
}

void Transform::AddChild(Transform* child) noexcept {
	if (!child) return;
	for (auto* c : _children) {
		if (c == child) return;
	}
	_children.push_back(child);
}

void Transform::RemoveChild(Transform* child) noexcept {
	if (!child) return;
	auto it = std::remove(_children.begin(), _children.end(), child);
	_children.erase(it, _children.end());
}

void Transform::PropagateDirtyToChildren() noexcept {
	for (auto* c : _children) {
		if (!c) continue;
		// 子はワールドのみ古くなる（ローカルは変わらない）
		if (!c->_worldDirty) {
			c->_worldDirty = true;
			// 孫にも伝播
			c->PropagateDirtyToChildren();
		}
	}
}

// dirty: 行列キャッシュが古くなった状態
void Transform::MarkDirty() noexcept {
	_localDirty = true;
	_worldDirty = true;
	PropagateDirtyToChildren();
}

// ローカルベクトルをワールド空間の方向ベクトルへ変換する。
// 平行移動は含めず、回転とスケールのみ反映する。
// 親が回転・拡大縮小している子オブジェクトでも、正しい向きを得るために使う。
VECTOR Transform::TransformVector(const VECTOR& localVector) const noexcept {
	const MATRIX& W = WorldMatrix();
	return VGet(
		localVector.x * W.m[0][0] + localVector.y * W.m[1][0] + localVector.z * W.m[2][0],
		localVector.x * W.m[0][1] + localVector.y * W.m[1][1] + localVector.z * W.m[2][1],
		localVector.x * W.m[0][2] + localVector.y * W.m[1][2] + localVector.z * W.m[2][2]
	);
}

// ローカル点をワールド空間へ変換する。
// 親の回転・スケール・平行移動をすべて反映するため、
// 子オブジェクトのコライダ中心や端点の計算に使う。
VECTOR Transform::TransformPoint(const VECTOR& localPoint) const noexcept {
	const MATRIX& W = WorldMatrix();
	return VGet(
		localPoint.x * W.m[0][0] + localPoint.y * W.m[1][0] + localPoint.z * W.m[2][0] + W.m[3][0],
		localPoint.x * W.m[0][1] + localPoint.y * W.m[1][1] + localPoint.z * W.m[2][1] + W.m[3][1],
		localPoint.x * W.m[0][2] + localPoint.y * W.m[1][2] + localPoint.z * W.m[2][2] + W.m[3][2]
	);
}

// ワールド軸の長さからスケールを取り出す。
// 親スケール込みの値になるため、子コライダの最終サイズ反映に使える。
VECTOR Transform::WorldScale() const noexcept {
	const MATRIX& W = WorldMatrix();
	const VECTOR axisX = VGet(W.m[0][0], W.m[0][1], W.m[0][2]);
	const VECTOR axisY = VGet(W.m[1][0], W.m[1][1], W.m[1][2]);
	const VECTOR axisZ = VGet(W.m[2][0], W.m[2][1], W.m[2][2]);
	return VGet(Len3Local(axisX), Len3Local(axisY), Len3Local(axisZ));
}

// 前方ベクトル取得（親回転込みのワールド軸）
VECTOR Transform::WForward() const noexcept {
	const MATRIX& W = WorldMatrix();
	return SafeNormalizeLocal(VGet(W.m[2][0], W.m[2][1], W.m[2][2]), VGet(0,0,1));
}

// 右方向ベクトル取得（親回転込みのワールド軸）
VECTOR Transform::WRight() const noexcept {
	const MATRIX& W = WorldMatrix();
	return SafeNormalizeLocal(VGet(W.m[0][0], W.m[0][1], W.m[0][2]), VGet(1,0,0));
}

// 上方向ベクトル取得（親回転込みのワールド軸）
VECTOR Transform::WUp() const noexcept {
	const MATRIX& W = WorldMatrix();
	return SafeNormalizeLocal(VGet(W.m[1][0], W.m[1][1], W.m[1][2]), VGet(0,1,0));
}

// ローカル行列取得
const MATRIX& Transform::LocalMatrix() const {
	if (!_localDirty) return _localMatrix;
	_localDirty = false;

	// DxLib の行列ユーティリティを使用
	const MATRIX S = MGetScale(_localScale);
	const MATRIX R = _localRotation.ToRotationMatrix();
	const MATRIX T = MGetTranslate(_localPosition);

	// 組み立て順はプロジェクトで統一（必要ならテストして調整する）
	_localMatrix = MMult(MMult(S, R), T);
	return _localMatrix;
}

// ワールド行列取得
const MATRIX& Transform::WorldMatrix() const {
	if (!_worldDirty) return _worldMatrix;
	_worldDirty = false;

	const MATRIX L = LocalMatrix();
	if (_parent) {
		// 親のワールド行列を掛けることで、子は親の位置・回転・スケールを継承する。
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

// ワールド回転取得（親回転込みの Quaternion）
Quaternion Transform::WorldRotation() const noexcept {
	if (_parent) {
		return _parent->WorldRotation() * _localRotation;
	}
	return _localRotation;
}

// ローカル軸方向に delta だけ移動する（Unity の Translate(delta, Space.Self) 相当）
// 自身の現在の向きに沿って動くため、回転済みオブジェクトの「前進」などに使う。
void Transform::Translate(const VECTOR& delta) noexcept {
	// ローカル軸 -> ワールド空間に変換してから加算する
	_localPosition = VAdd(_localPosition, TransformVector(delta));
	// 注意: TransformVector は WorldMatrix のスケールも含むため、
	//       スケールが1でない場合は意図した距離と異なる場合がある。
	MarkDirty();
}

// ワールド軸方向に delta だけ移動する（Unity の Translate(delta, Space.World) 相当）
// 親子関係があっても常にワールド座標基準で動かしたい場合に使う。
void Transform::TranslateWorld(const VECTOR& delta) noexcept {
	if (_parent) {
		// 親のワールド逆行列で delta をローカル空間へ変換してから加算する
		const MATRIX invParent = MInverse(_parent->WorldMatrix());
		const VECTOR localDelta = VGet(
			delta.x * invParent.m[0][0] + delta.y * invParent.m[1][0] + delta.z * invParent.m[2][0],
			delta.x * invParent.m[0][1] + delta.y * invParent.m[1][1] + delta.z * invParent.m[2][1],
			delta.x * invParent.m[0][2] + delta.y * invParent.m[1][2] + delta.z * invParent.m[2][2]
		);
		_localPosition = VAdd(_localPosition, localDelta);
	} else {
		// 親なし → ローカル = ワールドなのでそのまま加算
		_localPosition = VAdd(_localPosition, delta);
	}
	MarkDirty();
}
