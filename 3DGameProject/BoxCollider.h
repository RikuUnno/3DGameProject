#pragma once

#include "Collider.h"

class BoxCollider : public Collider {
public:
	// コンストラクタ/デストラクタ
	BoxCollider();
	virtual ~BoxCollider();

public:
	// Collider インターフェース
	Kind GetKind() const override { return Kind::Box; }						// OBB想定
	const AABB& GetAABB() const override { return _aabb; }					// Broad-phase 用 AABB（ワールド）
	VECTOR GetCenter() const override { return _center; }					// 中心点（ワールド）
	void UpdateShape() override;											// 更新(Transform変更時に呼び出す)
	const VECTOR& GetAxisX() const noexcept { return _axisX; }				// ワールドX軸
	const VECTOR& GetAxisY() const noexcept { return _axisY; }				// ワールドY軸
	const VECTOR& GetAxisZ() const noexcept { return _axisZ; }				// ワールドZ軸
	const VECTOR& GetHalfExtents() const noexcept { return _halfExtents; }	// ワールド半サイズ

public:
	// OBB設定（ローカル座標系）
	Box _box{};

public:
	// デバッグ描画
	void DrawDebug() override;		// デバッグ描画 (線状の形状)
	void DrawDebugAABB() override;	// デバッグ描画（AABBのみ）
	void DrawPrimitive() override;	// デバッグ描画（DXLibのプリミティブ描画）

private:
	// 永久バッファ（毎フレームの heap 確保を排除）
	VECTOR _center{};				// ワールド中心
	VECTOR _axisX = VGet(1,0,0);	// ワールドX軸
	VECTOR _axisY = VGet(0,1,0);	// ワールドY軸
	VECTOR _axisZ = VGet(0,0,1);	// ワールドZ軸
	VECTOR _halfExtents{};			// ワールド半サイズ
	AABB _aabb{};					// Broad-phase 用 AABB（ワールド）
};