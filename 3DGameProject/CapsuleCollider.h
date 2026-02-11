#pragma once
#include "Collider.h"

class CapsuleCollider : public Collider {
public:
	CapsuleCollider();
	virtual ~CapsuleCollider();

public:
	Kind GetKind() const override { return Kind::Capsule; }
	const AABB& GetAABB() const override { return aabb_; }
	VECTOR GetCenter() const override { return cap_.center; }
	void UpdateShape() override;

public:
	Capsule cap_{};

public:
	void DrawDebug() override;
	void DrawDebugAABB() override;

private:
	AABB aabb_{};
};