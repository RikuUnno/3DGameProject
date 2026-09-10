#include "PachinkoNail.h"

#include "DxLib.h"
#include "CapsuleCollider.h"

// コンストラクタ デストラクタ
PachinkoNail::PachinkoNail() = default;
PachinkoNail::~PachinkoNail() = default;

// GetCollider_: コライダーを返す
Collider* PachinkoNail::GetCollider_() const noexcept {
	return _capsuleCollider.get();
}

// EnsureCollider_: コライダーが存在しない場合は作成する
void PachinkoNail::EnsureCollider_() {
	if (!_capsuleCollider) _capsuleCollider = std::make_unique<CapsuleCollider>();
	_capsuleCollider->owner = this;
}

// ConfigureShape_: パラメータに基づいて形状を設定する
void PachinkoNail::ConfigureShape_(const VariantMap& params) {
	_radius = ParseFloatParam_(params, "radius", _radius);
	_halfHeight = ParseFloatParam_(params, "halfHeight", _halfHeight);
	_capsuleCollider->_cap.center = VGet(0.0f, 0.0f, 0.0f);
	_capsuleCollider->_cap.bottom = VGet(0.0f, -_halfHeight, 0.0f);
	_capsuleCollider->_cap.top = VGet(0.0f, _halfHeight, 0.0f);
	_capsuleCollider->_cap.radius = _radius;
}

// DefaultColor_: デフォルトの描画色を返す
unsigned int PachinkoNail::DefaultColor_() const noexcept {
	return GetColor(200, 205, 215);
}
