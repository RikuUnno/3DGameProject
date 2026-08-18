#include "PachinkoBall.h"

#include "DxLib.h"
#include "SphereCollider.h"

PachinkoBall::PachinkoBall() = default;
PachinkoBall::~PachinkoBall() = default;

Collider* PachinkoBall::GetCollider_() const noexcept {
	return _sphereCollider.get();
}

void PachinkoBall::EnsureCollider_() {
	if (!_sphereCollider) _sphereCollider = std::make_unique<SphereCollider>();
	_sphereCollider->owner = this;
}

void PachinkoBall::ConfigureShape_(const VariantMap& params) {
	_radius = ParseFloatParam_(params, "radius", _radius);
	_sphereCollider->_sphere.center = VGet(0.0f, 0.0f, 0.0f);
	_sphereCollider->_sphere.radius = _radius;
}

unsigned int PachinkoBall::DefaultColor_() const noexcept {
	return GetColor(190, 195, 210);
}
