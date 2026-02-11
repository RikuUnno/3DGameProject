#pragma once

#include <memory>

#include "GameObject.h"
#include "DxLib.h"
#include "Collider.h"

class SphereCollider;
class BoxCollider;

// Debug Class
// デバッグ用途の簡易オブジェクト/ユーティリティを集約する。
//まずは当たり判定確認用のSphere/Boxを提供。

// DebugSphereObject
class DebugSphereObject final : public GameObject {
public:
	DebugSphereObject(float radius, int layerBits, int maskBits);
	~DebugSphereObject() override;

	void Draw() override;

	bool IsColliding() const noexcept { return isColliding_; }
	bool IsTriggering() const noexcept { return isTriggering_; }
	void SetColliding(bool v) noexcept { isColliding_ = v; }
	void SetTriggering(bool v) noexcept { isTriggering_ = v; }

	SphereCollider* GetCollider() const noexcept;

private:
	std::unique_ptr<Collider> collider_;
	bool isColliding_ = false;
	bool isTriggering_ = false;
};

// DebugBoxObject
class DebugBoxObject final : public GameObject {
public:
	DebugBoxObject(const VECTOR& halfExtents, int layerBits, int maskBits);
	~DebugBoxObject() override;

	void Draw() override;

	bool IsColliding() const noexcept { return isColliding_; }		// 物理反応アリ
	bool IsTriggering() const noexcept { return isTriggering_; }	// 物理反応ナシ
	void SetColliding(bool v) noexcept { isColliding_ = v; }		// 物理反応アリ
	void SetTriggering(bool v) noexcept { isTriggering_ = v; }		// 物理反応ナシ

	BoxCollider* GetCollider() const noexcept;						// 型安全版取得

private:
	std::unique_ptr<Collider> collider_;// コライダー本体
	bool isColliding_ = false;			// 物理反応アリ
	bool isTriggering_ = false;			// 物理反応ナシ
};

namespace DebugClass {
	// MenuScene等で使う当たり判定デバッグ用の状態
	struct CollisionDebugState { // 当たり判定状態
		bool enabled = false;
		int enter =0;
		int stay =0;
		int exit =0;
	};

	//3Dデバッグ用の簡易描画（グリッド＋軸）
	void DrawSimple3DDebug();

	// MenuScene の当たり判定デバッグ用オブジェクト群
	struct MenuCollisionDebugObjects {
		CollisionDebugState state{};				// 状態
		std::unique_ptr<DebugSphereObject> sphereA; // デバッグ用球体A
		std::unique_ptr<DebugSphereObject> sphereB; // デバッグ用球体B
		std::unique_ptr<DebugBoxObject> box;		// デバッグ用箱体
	};
}
