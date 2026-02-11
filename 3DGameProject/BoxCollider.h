#pragma once
#include "Collider.h"

class BoxCollider : public Collider {
public:
	BoxCollider();
	virtual ~BoxCollider();

public:
	Kind GetKind() const override { return Kind::Box; }
	const AABB& GetAABB() const override { return aabb_; }
	VECTOR GetCenter() const override { return box_.center; }
	void UpdateShape() override;

public:
	// 設定値（ワールド）
	Box box_{};

public:
	void DrawDebug() override;
	void DrawDebugAABB() override;

private:
	AABB aabb_{};
};