#pragma once

#include <memory>
#include <string>

#include "LayerMask.h"
#include "PachinkoPhysicsObjectTpl.h"

class SphereCollider;

class PachinkoBall : public PachinkoPhysicsObjectTpl {
public:
	static std::string StaticPoolKey() { return "PachinkoBall"; }

	PachinkoBall();
	~PachinkoBall() override;

protected:
	Collider* GetCollider_() const noexcept override;
	void EnsureCollider_() override;
	void ConfigureShape_(const VariantMap& params) override;

	bool DefaultStatic_() const noexcept override { return false; }
	int DefaultLayer_() const noexcept override { return layerMask::BALL; }
	float DefaultMass_() const noexcept override { return 1.0f; }
	bool DefaultUseGravity_() const noexcept override { return true; }
	bool DefaultFreezeRotation_() const noexcept override { return false; }
	bool DefaultCcd_() const noexcept override { return true; }
	unsigned int DefaultColor_() const noexcept override;
	const char* DefaultMaterial_() const noexcept override { return "metal"; }

private:
	std::unique_ptr<SphereCollider> _sphereCollider;
	float _radius = 0.18f;
};
