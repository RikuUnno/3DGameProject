#pragma once
#include "Collider.h"

class SphereCollider : public Collider {
public:
	SphereCollider();
	virtual ~SphereCollider();

public:
	Kind GetKind() const override { return Kind::Sphere; }
	const AABB& GetAABB() const override { return _aabb; }
	VECTOR GetCenter() const override { return _center; }
	void UpdateShape() override;
	float GetRadius() const noexcept { return _radius; }

public:
	// 設定値。
	// owner がある場合は local offset / local radius として扱い、
	// UpdateShape で親子込みのワールド値へ変換して使う。
	Sphere _sphere{};

public:
	void DrawDebug() override;
	void DrawDebugAABB() override;

private:
	VECTOR _center{}; // ワールド中心
	float _radius = 0.0f; // ワールド半径
	AABB _aabb{};
};