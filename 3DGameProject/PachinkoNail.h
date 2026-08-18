#pragma once

#include <memory>
#include <string>

#include "LayerMask.h"
#include "PachinkoPhysicsObjectTpl.h"

class CapsuleCollider;

class PachinkoNail : public PachinkoPhysicsObjectTpl {
public:
	static std::string StaticPoolKey() { return "PachinkoNail"; }

	PachinkoNail();
	~PachinkoNail() override;

protected:
	Collider* GetCollider_() const noexcept override;
	void EnsureCollider_() override;
	void ConfigureShape_(const VariantMap& params) override;

	bool DefaultStatic_() const noexcept override { return true; }
	int DefaultLayer_() const noexcept override { return layerMask::ENVIRONMENT; }
	float DefaultMass_() const noexcept override { return 0.0f; }
	bool DefaultUseGravity_() const noexcept override { return false; }
	bool DefaultFreezeRotation_() const noexcept override { return true; }
	bool DefaultCcd_() const noexcept override { return false; }
	unsigned int DefaultColor_() const noexcept override;
	const char* DefaultMaterial_() const noexcept override { return "metal"; }

private:
	std::unique_ptr<CapsuleCollider> _capsuleCollider;
	float _radius = 0.06f;
	float _halfHeight = 0.22f;
};
