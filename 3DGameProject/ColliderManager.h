#pragma once
#include "Collider.h"
#include "DxLib.h"
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <cstdint>
#include <cstddef>
#include <atomic>

class ColliderManager {
private: // ペア状態管理（※unordered_set のメンバより先に定義が必要）
	// PairKey と PairHash の定義
	struct PairKey {
		Collider* a{};
		Collider* b{};
		bool operator==(const PairKey& o) const noexcept { return a == o.a && b == o.b; }
	};
	// ハッシュ関数
	struct PairHash {
		std::size_t operator()(const PairKey& k) const noexcept {
			return (reinterpret_cast<std::uintptr_t>(k.a) >>4) ^ (reinterpret_cast<std::uintptr_t>(k.b) <<1);
		}
	};

	// ペアキー作成
	PairKey MakeKey(Collider* a, Collider* b) const noexcept {
		return (a < b) ? PairKey{ a,b } : PairKey{ b,a };
	}

	// イベントディスパッチ
	void DispatchEnter(Collider* a, Collider* b);
	void DispatchStay(Collider* a, Collider* b);
	void DispatchExit(Collider* a, Collider* b);

	// ペア管理
	std::unordered_set<PairKey, PairHash> _prevPairs{}; // 前フレームのペア
	std::unordered_set<PairKey, PairHash> _currPairs{}; // 今フレームのペア

	// 終了処理ガード：終了中は Update/Register/Unregister を no-op にする
	std::atomic_bool _shuttingDown{ false };

private:
	ColliderManager() = default;
	virtual ~ColliderManager() {};

public:
	static ColliderManager& Instance() {
		static ColliderManager instance;
		return instance;
	}

	// 明示的終了（main/WinMainから呼び、静的デストラクタより前に安全化する）
	void Shutdown() noexcept; // 終了処理(Mainの最後で呼ぶ)
	bool IsShuttingDown() const noexcept { return _shuttingDown.load(std::memory_order_relaxed); }

public:
	// 更新
	void Update();
	void Update(float dtSec);

public:
	// デバッグ描画
	void DrawDebugAll();
	void DrawDebugAABBAll();

public:
	// Colliderの登録/解除	
	void RegisterCollider(Collider* collider);
	void UnregisterCollider(Collider* collider);

public:
	// Broad Phase（現状は BuildCurrentPairs 内で使用）
	void SpatialPartitioning();
	bool CheckLayerMaskCollisions(Collider* a, Collider* b);
	bool CheckAABBCollisions(Collider* a, Collider* b);
	bool CheckAABBCollisionsSwept(Collider* a, Collider* b);

	void CheckDetailedCollisions(); // 詳細判定

private:
	// 判定ヘルパー
	void UpdateAllShapes();					//形状更新
	void BuildCurrentPairs();				// ペア構築
	void ProcessPairEvents();				// イベント処理
	void ResolvePushOut(Collider* a, Collider* b);	// 押し戻し
	AABB GetSweptAABB(Collider* collider) const;

private:
	// 各種詳細判定
	void CheckSphereSphere(Collider* a, Collider* b);	// Sphere-Sphere 当たり判定
	void CheckSphereBox(Collider* a, Collider* b);		// Sphere-Box(OBB) 当たり判定
	void CheckBoxBox(Collider* a, Collider* b);			// Box-Box(OBB) 当たり判定
	void CheckCapsuleCapsule(Collider* a, Collider* b);	// Capsule-Capsule 当たり判定
	void CheckSphereCapsule(Collider* a, Collider* b);	// Sphere-Capsule 当たり判定
	void CheckBoxCapsule(Collider* a, Collider* b);		// Box(OBB)-Capsule 当たり判定

	// 各種押し戻し処理
	void PushOutSphereSphere(Collider* a, Collider* b);		// Sphere-Sphere 押し戻し
	void PushOutSphereBox(Collider* a, Collider* b);		// Sphere-Box 押し戻し
	void PushOutBoxBox(Collider* a, Collider* b); 			// Box-Box 押し戻し
	void PushOutCapsuleCapsule(Collider* a, Collider* b); 	// Capsule-Capsule 押し戻し
	void PushOutSphereCapsule(Collider* a, Collider* b); 	// Sphere-Capsule 押し戻し
	void PushOutBoxCapsule(Collider* a, Collider* b); 		// Box-Capsule 押し戻し

public:
	// 空間分割セルサイズ（ワールド単位）。デフォルト:50
	float GetCellSize() const noexcept { return _cellSize; }
	void SetCellSize(float cellSize) noexcept { _cellSize = (cellSize >0.01f) ? cellSize :0.01f; }

	// Contact情報（Physics 用）
	struct Contact {
		Collider* a = nullptr; // contact normal は a -> b
		Collider* b = nullptr;
		VECTOR normal = VGet(0,0,0);
		float penetration = 0.0f;
	};

	const std::vector<Contact>& GetContacts() const noexcept { return _contacts; }

private:
	// 空間分割（Spatial Hash）セルサイズ
	float _cellSize =50.0f;
	float _deltaTimeSec = 1.0f / 60.0f;

	std::vector<Collider*> _colliders{}; 	// 登録コライダー群
	bool _narrowHit = false; 			// 詳細判定結果
	std::vector<Contact> _contacts; // 今フレームの接触情報
	std::unordered_map<Collider*, AABB> _prevAABBs; // 前フレームのAABB（CCD用）
};