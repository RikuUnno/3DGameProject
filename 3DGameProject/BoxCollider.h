#pragma once
#include "Collider.h"

class BoxCollider : public Collider
{
public:
	BoxCollider();
	virtual ~BoxCollider();

private:
	// AABB設定
	virtual void SetAABB() override;

public:
	// デバッグ描画
	virtual void DrawDebug() override;		// 本体デバッグ描画
	virtual void DrawDebugAABB() override; // AABBデバッグ描画

};