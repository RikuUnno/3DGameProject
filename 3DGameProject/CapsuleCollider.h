#pragma once
#include "Collider.h"

class CapsuleCollider : public Collider {
public:
	CapsuleCollider();
	virtual ~CapsuleCollider();

public:
	Kind GetKind() const override { return Kind::Capsule; }
	const AABB& GetAABB() const override { return _aabb; }
	VECTOR GetCenter() const override { return _center; }
	void UpdateShape() override;
	const VECTOR& GetBottom() const noexcept { return _bottom; }
	const VECTOR& GetTop() const noexcept { return _top; }
	float GetRadius() const noexcept { return _radius; }

public:
	// 設定値。
	// owner がある場合は center / bottom / top / radius をローカル設定として扱い、
	// UpdateShape で親子込みのワールド端点・ワールド半径へ変換する。
	Capsule _cap{};

public:
	void DrawDebug() override;
	void DrawDebugAABB() override;

private:
	VECTOR _center{}; // ワールド中心
	VECTOR _bottom{}; // ワールド端点A
	VECTOR _top{}; // ワールド端点B
	float _radius = 0.0f; // ワールド半径
	AABB _aabb{};
};