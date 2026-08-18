#include "PachinkoFieldTpl.h"

#include <algorithm>

#include "ColliderManager.h"
#include "PhysicsManager.h"
#include "PhysicsMaterial.h"

// コンストラクタ/デストラクタ
PachinkoFieldTpl::PachinkoFieldTpl() {
	PrepareForAcquire_();
	_boxCollider = std::make_unique<BoxCollider>();
	_boxCollider->owner = this;
	_physicsBody._owner = this;
	_physicsBody.Reset();
}

PachinkoFieldTpl::~PachinkoFieldTpl() = default;

// ライフサイクル
void PachinkoFieldTpl::OnDestroy() {
	ReleaseFromManagers_();
}

// プール/再利用フック
void PachinkoFieldTpl::OnAcquire(const VariantMap& params) {
	PrepareForAcquire_();
	transform.SetLocalScale(VGet(1.0f, 1.0f, 1.0f));
	_physicsBody.Reset();
	_physicsBody._owner = this;

	_halfExtents = DefaultHalfExtents_();
	_drawColor = DefaultColor_();
	_materialName = DefaultMaterialName_();
	ApplyParams_(params);

	RebuildCollider_();
	if (_boxCollider) {
		_boxCollider->owner = this;
		_boxCollider->isTrigger = ParseBoolParam_(params, "trigger", false);
		_boxCollider->layer = layerMask::ENVIRONMENT;
		_boxCollider->mask = mask::ALL;
		_boxCollider->enableCCD = false;
		_boxCollider->SetDebugColor(_drawColor);
		_boxCollider->UpdateShape();
	}

	_physicsBody._useGravity = false;
	_physicsBody._isKinematic = true;
	_physicsBody._freezeRotation = true;
	_physicsBody._linearDamping = 0.0f;
	_physicsBody._angularDamping = 0.0f;
	_physicsBody._restitution = 0.0f;
	_physicsBody._friction = 0.0f;
	_physicsBody._material = PhysicsMaterial::FromName(_materialName.c_str());
	_physicsBody._material.friction = 0.0f;
	_physicsBody._material.staticFriction = 0.0f;
	_physicsBody._material.restitution = 0.0f;
	_physicsBody._material.linearDamping = 0.0f;
	_physicsBody._material.angularDamping = 0.0f;
	_physicsBody.SetMass(0.0f);
	_physicsBody.ComputeInertia(_boxCollider.get());

	if (_boxCollider) ColliderManager::Instance().RegisterCollider(_boxCollider.get());
	PhysicsManager::Instance().RegisterBody(&_physicsBody);
	_registered = true;
}

// プールに返却される直前の後片付け
void PachinkoFieldTpl::OnRelease() {
	ReleaseFromManagers_();
	PrepareForRelease_();
}

// 更新（毎フレーム呼ばれる）
void PachinkoFieldTpl::Draw() {
	if (!_boxCollider) return;
	switch (FieldDrawStyle_()) {
	case DrawStyle::AABB:
		_boxCollider->DrawDebugAABB();
		break;
	case DrawStyle::Solid:
		_boxCollider->DrawPrimitive();
		break;
	case DrawStyle::OBBWire:
	default:
		_boxCollider->DrawDebug();
		break;
	}
}

// プールに返却される直前の後片付け
void PachinkoFieldTpl::ReleaseFromManagers_() {
	if (!_registered) return;
	if (_boxCollider) ColliderManager::Instance().UnregisterCollider(_boxCollider.get());
	PhysicsManager::Instance().UnregisterBody(&_physicsBody);
	_registered = false;
}

// コライダーの再構築
void PachinkoFieldTpl::RebuildCollider_() {
	if (!_boxCollider) _boxCollider = std::make_unique<BoxCollider>();
	_boxCollider->owner = this;
	_boxCollider->_box.center = VGet(0.0f, 0.0f, 0.0f);
	_boxCollider->_box.halfExtents = _halfExtents;
	_boxCollider->UpdateShape();
}

// パラメータマップから設定を適用
void PachinkoFieldTpl::ApplyParams_(const VariantMap& params) {
	ApplyTransformFromParams_(params);

	_halfExtents = VGet(
		ParseFloatParam_(params, "hx", _halfExtents.x),
		ParseFloatParam_(params, "hy", _halfExtents.y),
		ParseFloatParam_(params, "hz", _halfExtents.z)
	);

	const int color = ParseIntParam_(params, "color", 0);
	if (color != 0) _drawColor = static_cast<unsigned int>(color);
	_materialName = ParseStringParam_(params, "material", _materialName);
}