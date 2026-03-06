#pragma once
#include "Collider.h"

class CapsuleCollider : public Collider {
public:
	CapsuleCollider();
	virtual ~CapsuleCollider();

public:
	Kind GetKind() const override { return Kind::Capsule; }
	const AABB& GetAABB() const override { return _aabb; }
	VECTOR GetCenter() const override { return _cap.center; }
	void UpdateShape() override;

public:
	// 設定値（ワールド）
	Capsule _cap{};

public:
	void DrawDebug() override;
	void DrawDebugAABB() override;

private:
	AABB _aabb{};
};