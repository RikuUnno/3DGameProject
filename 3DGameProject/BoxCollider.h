#pragma once
#include "Collider.h"
#include "ColliderType.h"

class BoxCollider : public Collider
{
public:
	// コンストラクタ/デストラクタ
	BoxCollider();
	virtual ~BoxCollider();

private:
	// AABB設定
	virtual void SetAABB() override;

public:
	// デバッグ描画
	virtual void DrawDebug() override;		// 本体デバッグ描画
	virtual void DrawDebugAABB() override; // AABBデバッグ描画

private:
	Box box; // ボックス情報
};