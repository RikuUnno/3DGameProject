#pragma once
#include "Collider.h"

class SphereCollider : public Collider {
public:
	SphereCollider();
	virtual ~SphereCollider();

public:
	Kind GetKind() const override { return Kind::Sphere; }
	const AABB& GetAABB() const override { return _aabb; }
	VECTOR GetCenter() const override { return sphere_.center; }
	void UpdateShape() override;

public:
	// 設定値（ワールド）
	Sphere sphere_{};

public:
	void DrawDebug() override;
	void DrawDebugAABB() override;

private:
	AABB _aabb{};
};