#pragma once
#include "Collider.h"
#include "ColliderType.h"

class CapsuleCollider : public Collider
{
public:
	// コンストラクタ/デストラクタ
	CapsuleCollider();
	virtual ~CapsuleCollider();

	// AABB設定
	virtual void SetAABB() override;

public:
	// デバッグ描画
	virtual void DrawDebug() override;		// 本体デバッグ描画
	virtual void DrawDebugAABB() override; // AABBデバッグ描画

public:
	Capsule cap; // カプセル情報
};