#pragma once
#include "Collider.h"
#include "DxLib.h"
#include <vector>
#include <memory>

class ColliderManager {
private: // コンストラクタ/デストラクタ
	// シングルトンパターンにするのでコンストラクタをprivateにする
	ColliderManager() = default;
	~ColliderManager() = default;

public: // インスタンス取得
	// シングルトンインスタンス取得
	static ColliderManager& GetInstance() {
		static ColliderManager instance;
		return instance;
	}

public: // 更新
	void Update();


public: // デバッグ描画
	// 本体デバッグ描画
	void DrawDebugAll();
	// AABBデバッグ描画
	void DrawDebugAABBAll();
	// 上下とも生成されているObjectのみ描画

public: // 登録/解除
	// Colliderの登録
	void RegisterCollider(Collider* collider);
	// Colliderの登録解除
	void UnregisterCollider(Collider* collider);

public: // 当たり判定
	// Broad Phase
	// 空間分割
	void SpatialPartitioning();
	// Layer/Maskによる当たり判定
	void CheckLayerMaskCollisions();
	// AABB同士の当たり判定
	void CheckAABBCollisions();

	// Narrow Phase
	// 詳細な当たり判定
	void CheckDetailedCollisions();

private: // 変数
	// 登録されているColliderのリスト
	std::vector<Collider*> colliders_; // すべてのコライダーを保持

};