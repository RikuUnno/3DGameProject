#pragma once

#include <string>

#include "GameObject.h"
#include "PhysicsBody.h"

class Collider;

class PachinkoPhysicsObjectTpl : public GameObject {
public:
	PachinkoPhysicsObjectTpl();
	~PachinkoPhysicsObjectTpl() override;

	void Awake() override {}
	void Start() override {}
	void End() override {}
	void OnDestroy() override;
	void OnAcquire(const VariantMap& params) override;
	void OnRelease() override;
	void Update(float /*dt*/) override {}
	void Draw() override;

protected:
	virtual Collider* GetCollider_() const noexcept = 0;
	virtual void EnsureCollider_() = 0;
	virtual void ConfigureShape_(const VariantMap& params) = 0;

	virtual bool DefaultStatic_() const noexcept = 0;
	virtual int DefaultLayer_() const noexcept = 0;
	virtual float DefaultMass_() const noexcept { return 1.0f; }
	virtual bool DefaultUseGravity_() const noexcept { return true; }
	virtual bool DefaultFreezeRotation_() const noexcept { return false; }
	virtual bool DefaultCcd_() const noexcept { return true; }
	virtual unsigned int DefaultColor_() const noexcept = 0;
	virtual const char* DefaultMaterial_() const noexcept = 0;
	virtual void DrawCollider_(Collider* collider);

	PhysicsBody& Body_() noexcept { return _physicsBody; }

	unsigned int _drawColor = 0;
	std::string _materialName;

private:
	void ReleaseFromManagers_();

private:
	PhysicsBody _physicsBody;
	bool _registered = false;
};
