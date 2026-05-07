#pragma once
#include "Collider.h"
#include "DxLib.h"
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <cstdint>
#include <cstddef>
#include <atomic>
#include <mutex>

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

	// HalfPlane narrow-phase
	void CheckSphereHalfPlane(Collider* sphere, Collider* plane);
	void CheckBoxHalfPlane(Collider* box, Collider* plane);
	void CheckCapsuleHalfPlane(Collider* capsule, Collider* plane);

	// Compound dispatch
	void CheckCompoundVsAny(Collider* compound, Collider* other);

	// 各種押し戻し処理
	void PushOutSphereSphere(Collider* a, Collider* b);		// Sphere-Sphere 押し戻し
	void PushOutSphereBox(Collider* a, Collider* b);		// Sphere-Box 押し戻し
	void PushOutBoxBox(Collider* a, Collider* b); 			// Box-Box 押し戻し
	void PushOutCapsuleCapsule(Collider* a, Collider* b); 	// Capsule-Capsule 押し戻し
	void PushOutSphereCapsule(Collider* a, Collider* b); 	// Sphere-Capsule 押し戻し
	void PushOutBoxCapsule(Collider* a, Collider* b); 	// Box-Capsule 押し戻し
	void PushOutSphereHalfPlane(Collider* sphere, Collider* plane);
	void PushOutBoxHalfPlane(Collider* box, Collider* plane);
	void PushOutCapsuleHalfPlane(Collider* capsule, Collider* plane);

public:
	// 空間分割セルサイズ（ワールド単位）。デフォルト:4
	float GetCellSize() const noexcept { return _cellSize; }
	void SetCellSize(float cellSize) noexcept { _cellSize = (cellSize >0.01f) ? cellSize :0.01f; }

	// Adaptive cell size: auto-compute based on average collider extent
	void SetAdaptiveCellSize(bool enabled) noexcept { _adaptiveCellSize = enabled; }
	bool IsAdaptiveCellSize() const noexcept { return _adaptiveCellSize; }

	// Contact情報（Physics 用）
	struct Contact {
		Collider* a = nullptr; // contact normal は a -> b
		Collider* b = nullptr;
		VECTOR normal = VGet(0,0,0);
		VECTOR point  = VGet(0,0,0); // ワールド空間の接触点
		float penetration = 0.0f;
	};

	const std::vector<Contact>& GetContacts() const noexcept { return _contacts; }

	// 登録コライダー一覧の取得（Raycast 等で使用）
	const std::vector<Collider*>& GetColliders() const noexcept { return _colliders; }

	// Find first collider owned by the given GameObject (for inertia computation etc.)
	Collider* FindColliderByOwner(GameObject* owner) const noexcept;

private:
	// 空間分割（Spatial Hash）セルサイズ
	float _cellSize = 4.0f;
	bool _adaptiveCellSize = true;
	float _deltaTimeSec = 1.0f / 60.0f;

	void ComputeAdaptiveCellSize();

	mutable std::mutex _mtx;							// スレッド安全用ミューテックス
	std::vector<Collider*> _colliders{}; 	// 登録コライダー群
	bool _narrowHit = false; 			// 詳細判定結果 (serial fallback)
	std::vector<Contact> _contacts; // 今フレームの接触情報
	std::unordered_map<Collider*, AABB> _prevAABBs; // 前フレームのAABB（CCD用）

	// ================================================================
	//  SpatialPartitioning 永続バッファ（毎フレームの heap 確保を抑制）
	//  毎フレーム clear()/resize() のみ行い、capacity は保持する。
	// ================================================================
	struct CellKey {
		int x{}, y{}, z{};
		bool operator==(const CellKey& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
	};
	struct CellHash {
		size_t operator()(const CellKey& k) const noexcept {
			size_t h = 1469598103934665603ull;
			h ^= static_cast<size_t>(k.x) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
			h ^= static_cast<size_t>(k.y) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
			h ^= static_cast<size_t>(k.z) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
			return h;
		}
	};
	struct CellEntry { CellKey key; int colliderIdx; };
	struct CandidatePair { int a; int b; };

	std::vector<Collider*>                               _snapshotBuf;       // UpdateAllShapes 用
	std::vector<Collider*>                               _activeBuf;         // フィルタ済みコライダ
	std::vector<AABB>                                    _sweptAABBsBuf;     // swept AABB
	std::vector<std::vector<CellEntry>>                  _perColliderCellsBuf; // セル分解結果
	std::unordered_map<CellKey, std::vector<int>, CellHash> _gridBuf;        // 空間グリッド
	std::vector<CandidatePair>                           _candidatesBuf;     // broad-phase 候補ペア
	std::vector<uint64_t>                                _seenPairsBuf;      // 重複除去用
	std::vector<uint8_t>                                 _perPairHitBuf;     // narrow-phase ヒット結果（uint8_t: vector<bool>の並列書き込み競合を回避）
	std::vector<std::vector<Contact>>                    _perPairContactsBuf; // narrow-phase 接触点

	// ================================================================
	//  Week 1-2: Narrow Phase Parallelization
	// ================================================================
	// During parallel narrow phase each CheckXxx writes to _tlNarrowHit/_tlContactOut.
	// Serial mode: _tlContactOut == nullptr -> EmitContact falls back to _contacts.
	static thread_local bool                  _tlNarrowHit;
	static thread_local std::vector<Contact>* _tlContactOut; // nullptr = serial mode

	// Output helper used by ALL CheckXxx functions.
	// Replaces direct _contacts.push_back() calls to support both serial and parallel modes.
	inline void EmitContact(const Contact& ct) {
		if (_tlContactOut) _tlContactOut->push_back(ct);
		else               _contacts.push_back(ct);
	}
};