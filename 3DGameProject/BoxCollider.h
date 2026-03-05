#pragma once
#include "Collider.h"

class BoxCollider : public Collider {
public:
	BoxCollider();
	virtual ~BoxCollider();

public:
	Kind GetKind() const override { return Kind::Box; }
	const AABB& GetAABB() const override { return _aabb; }
	VECTOR GetCenter() const override { return box_.center; }
	void UpdateShape() override;

public:
	// 設定値（ワールド）
	Box box_{};

public:
	void DrawDebug() override;
	void DrawDebugAABB() override;

private:
	AABB _aabb{};
};