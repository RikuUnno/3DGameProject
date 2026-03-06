#pragma once

#include "Collider.h"

class BoxCollider : public Collider {
public:
	BoxCollider();
	virtual ~BoxCollider();

public:
	Kind GetKind() const override { return Kind::Box; }
	const AABB& GetAABB() const override { return _aabb; }
	VECTOR GetCenter() const override { return _center; }
	void UpdateShape() override;
	const VECTOR& GetAxisX() const noexcept { return _axisX; }
	const VECTOR& GetAxisY() const noexcept { return _axisY; }
	const VECTOR& GetAxisZ() const noexcept { return _axisZ; }
	const VECTOR& GetHalfExtents() const noexcept { return _halfExtents; }

public:
	// 設定値。
	// owner がある場合は center / halfExtents をローカル設定として扱い、
	// UpdateShape で親子込みのワールド中心・ワールド軸・ワールドサイズへ変換する。
	Box _box{};

public:
	void DrawDebug() override;
	void DrawDebugAABB() override;

private:
	VECTOR _center{}; // ワールド中心
	VECTOR _axisX = VGet(1,0,0); // ワールドX軸
	VECTOR _axisY = VGet(0,1,0); // ワールドY軸
	VECTOR _axisZ = VGet(0,0,1); // ワールドZ軸
	VECTOR _halfExtents{}; // ワールド半サイズ
	AABB _aabb{};
};