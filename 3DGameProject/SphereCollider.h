#pragma once
#include "Collider.h"
#include "ColliderType.h"

class SphereCollider : public Collider
{
public:
	// コンストラクタ/デストラクタ	
	SphereCollider();
	virtual ~SphereCollider();

public:
	// AABB設定
	virtual void SetAABB() override;

public:
	// デバッグ描画
	virtual void DrawDebug() override;		// 本体デバッグ描画
	virtual void DrawDebugAABB() override; // AABBデバッグ描画

private:
	Capsule cap; // カプセル情報

};