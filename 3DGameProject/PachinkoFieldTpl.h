#pragma once

#include <memory>
#include <string>

#include "GameObject.h"
#include "BoxCollider.h"
#include "PhysicsBody.h"

// PachinkoFieldTpl
class PachinkoFieldTpl : public GameObject {
public:
	enum class DrawStyle {
		AABB,
		OBBWire,
		Solid,
	};

	PachinkoFieldTpl();
	~PachinkoFieldTpl() override;

	void Awake() override {}
	void Start() override {}
	void End() override {}
	void OnDestroy() override;

	void OnAcquire(const VariantMap& params) override;
	void OnRelease() override;
	void Update(float /*dt*/) override {}
	void Draw() override;

protected:
	virtual const char* FieldName_() const noexcept = 0;
	virtual VECTOR DefaultHalfExtents_() const noexcept = 0;
	virtual unsigned int DefaultColor_() const noexcept = 0;
	virtual std::string DefaultMaterialName_() const { return "frictionless"; }
	virtual DrawStyle FieldDrawStyle_() const noexcept { return DrawStyle::OBBWire; }

private:
	void ReleaseFromManagers_();
	void RebuildCollider_();
	void ApplyParams_(const VariantMap& params);

private:
	std::unique_ptr<BoxCollider> _boxCollider;
	PhysicsBody _physicsBody;
	bool _registered = false;
	VECTOR _halfExtents = VGet(0.5f, 4.0f, 0.5f);
	unsigned int _drawColor = 0;
	std::string _materialName = "frictionless";
};