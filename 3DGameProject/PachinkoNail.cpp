#include "PachinkoNail.h"

#include "DxLib.h"
#include "CapsuleCollider.h"

PachinkoNail::PachinkoNail() = default;
PachinkoNail::~PachinkoNail() = default;

Collider* PachinkoNail::GetCollider_() const noexcept {
	return _capsuleCollider.get();
}

void PachinkoNail::EnsureCollider_() {
	if (!_capsuleCollider) _capsuleCollider = std::make_unique<CapsuleCollider>();
	_capsuleCollider->owner = this;
}

void PachinkoNail::ConfigureShape_(const VariantMap& params) {
	_radius = ParseFloatParam_(params, "radius", _radius);
	_halfHeight = ParseFloatParam_(params, "halfHeight", _halfHeight);
	_capsuleCollider->_cap.center = VGet(0.0f, 0.0f, 0.0f);
	_capsuleCollider->_cap.bottom = VGet(0.0f, -_halfHeight, 0.0f);
	_capsuleCollider->_cap.top = VGet(0.0f, _halfHeight, 0.0f);
	_capsuleCollider->_cap.radius = _radius;
}

unsigned int PachinkoNail::DefaultColor_() const noexcept {
	return GetColor(200, 205, 215);
}
