#include "PachinkoPhysicsObjectTpl.h"

#include <algorithm>

#include "Collider.h"
#include "ColliderManager.h"
#include "PhysicsManager.h"
#include "PhysicsMaterial.h"
#include "SceneManager.h"

PachinkoPhysicsObjectTpl::PachinkoPhysicsObjectTpl() {
	_ownerSceneId = SceneManager::Instance().CurrentSceneId();
	_physicsBody._owner = this;
	_physicsBody.Reset();
}

PachinkoPhysicsObjectTpl::~PachinkoPhysicsObjectTpl() = default;

void PachinkoPhysicsObjectTpl::OnDestroy() {
	ReleaseFromManagers_();
}

void PachinkoPhysicsObjectTpl::OnAcquire(const VariantMap& params) {
	PrepareForAcquire_();
	_physicsBody.Reset();
	_physicsBody._owner = this;

	_drawColor = DefaultColor_();
	_materialName = DefaultMaterial_();

	ApplyTransformFromParams_(params);
	EnsureCollider_();
	ConfigureShape_(params);

	Collider* collider = GetCollider_();
	if (!collider) return;

	collider->owner = this;
	collider->isTrigger = ParseBoolParam_(params, "trigger", false);
	collider->layer = DefaultLayer_();
	collider->mask = mask::ALL;
	collider->enableCCD = ParseBoolParam_(params, "ccd", DefaultCcd_());
	collider->SetDebugColor(_drawColor);
	collider->UpdateShape();

	const bool isStaticObject = ParseBoolParam_(params, "static", DefaultStatic_());
	isStatic = isStaticObject;
	if (isStaticObject) {
		_physicsBody.SetMass(0.0f);
		_physicsBody._isKinematic = true;
		_physicsBody._useGravity = false;
		_physicsBody._freezeRotation = true;
	} else {
		_physicsBody._isKinematic = false;
		_physicsBody._useGravity = ParseBoolParam_(params, "gravity", DefaultUseGravity_());
		_physicsBody._freezeRotation = ParseBoolParam_(params, "freezeRotation", DefaultFreezeRotation_());
		const float mass = ParseFloatParam_(params, "mass", DefaultMass_());
		_physicsBody.SetMass((std::max)(mass, 1e-6f));
	}

	_materialName = ParseStringParam_(params, "material", _materialName);
	PhysicsMaterial mat = PhysicsMaterial::FromName(_materialName.c_str());
	_physicsBody.ApplyMaterial(mat, collider);

	if (params.count("restitution")) {
		_physicsBody._restitution = ParseFloatParam_(params, "restitution", _physicsBody._restitution);
		_physicsBody._material.restitution = _physicsBody._restitution;
	}
	if (params.count("friction")) {
		_physicsBody._friction = ParseFloatParam_(params, "friction", _physicsBody._friction);
		_physicsBody._material.friction = _physicsBody._friction;
		_physicsBody._material.staticFriction = _physicsBody._friction * 1.2f;
	}

	_physicsBody._velocity = VGet(
		ParseFloatParam_(params, "vx", 0.0f),
		ParseFloatParam_(params, "vy", 0.0f),
		ParseFloatParam_(params, "vz", 0.0f));
	_physicsBody._angularVelocity = VGet(
		ParseFloatParam_(params, "avx", 0.0f),
		ParseFloatParam_(params, "avy", 0.0f),
		ParseFloatParam_(params, "avz", 0.0f));

	_physicsBody.ComputeInertia(collider);

	ColliderManager::Instance().RegisterCollider(collider);
	PhysicsManager::Instance().RegisterBody(&_physicsBody);
	_registered = true;
}

void PachinkoPhysicsObjectTpl::OnRelease() {
	ReleaseFromManagers_();
	PrepareForRelease_();
}

void PachinkoPhysicsObjectTpl::Draw() {
	if (Collider* c = GetCollider_()) DrawCollider_(c);
}

void PachinkoPhysicsObjectTpl::DrawCollider_(Collider* collider) {
	if (collider) collider->DrawPrimitive();
}

void PachinkoPhysicsObjectTpl::ReleaseFromManagers_() {
	if (!_registered) return;
	if (Collider* c = GetCollider_()) ColliderManager::Instance().UnregisterCollider(c);
	PhysicsManager::Instance().UnregisterBody(&_physicsBody);
	_registered = false;
}
