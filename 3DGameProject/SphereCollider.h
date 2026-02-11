#pragma once
#include "Collider.h"

class SphereCollider : public Collider {
public:
	SphereCollider();
	virtual ~SphereCollider();

public:
	Kind GetKind() const override { return Kind::Sphere; }
	const AABB& GetAABB() const override { return aabb_; }
	VECTOR GetCenter() const override { return sphere_.center; }
	void UpdateShape() override;

public:
	// 設定値（ワールド）
	Sphere sphere_{};

public:
	void DrawDebug() override;
	void DrawDebugAABB() override;

private:
	AABB aabb_{};
};