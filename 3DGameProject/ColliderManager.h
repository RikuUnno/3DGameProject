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
private:
	// ペア状態管理用の構造体定義
	struct PairKey {
		Collider* a{};
		Collider* b{};
		bool operator==(const PairKey& o) const noexcept { return a == o.a && b == o.b; }
	};

	// ペアキーのハッシュ関数
	struct PairHash {
		std::size_t operator()(const PairKey& k) const noexcept {
			return (reinterpret_cast<std::uintptr_t>(k.a) >>4) ^ (reinterpret_cast<std::uintptr_t>(k.b) <<1);
		}
	};

	// 衝突ペア管理: (a, b) と (b, a) を同一ペアとして扱う
	PairKey MakeKey(Collider* a, Collider* b) const noexcept {
		return (a < b) ? PairKey{ a,b } : PairKey{ b,a };
	}

	// イベントディスパッチ: Enter/Stay/Exit コールバックの実行
	void DispatchEnter(Collider* a, Collider* b);
	void DispatchStay(Collider* a, Collider* b);
	void DispatchExit(Collider* a, Collider* b);

	// フレーム毎の衝突ペア追跡
	std::unordered_set<PairKey, PairHash> _prevPairs{}; // 前フレームの衝突ペア
	std::unordered_set<PairKey, PairHash> _currPairs{}; // 現フレームの衝突ペア

	// 終了処理ガード: true の場合 Update/Register/Unregister は no-op になる
	std::atomic_bool _shuttingDown{ false };

private:
	ColliderManager() = default;
	virtual ~ColliderManager() {};

public:
	static ColliderManager& Instance() {
		static ColliderManager instance;
		return instance;
	}

	// 明示的終了処理 (Main 終了時に呼び出し、静的デストラクタより前に実行)
	void Shutdown() noexcept;
	bool IsShuttingDown() const noexcept { return _shuttingDown.load(std::memory_order_relaxed); }

public:
	// フレーム毎の更新処理
	void Update(float dtSec);

public:
	// デバッグ描画機能
	void DrawDebugAll();		// コライダーの形状描画
	void DrawDebugAABBAll();	// AABB の描画

public:
	// コライダーの登録/解除
	void RegisterCollider(Collider* collider);
	void UnregisterCollider(Collider* collider);

public:
	// Broad Phase: 空間分割による粗い衝突判定
	void SpatialPartitioning();
	bool CheckLayerMaskCollisions(Collider* a, Collider* b);	// レイヤーマスク衝突判定
	bool CheckAABBCollisions(Collider* a, Collider* b);		// AABB 衝突判定
	bool CheckAABBCollisionsSwept(Collider* a, Collider* b);	// Swept AABB 衝突判定 (CCD)

	// Narrow Phase: 詳細な衝突判定
	void CheckDetailedCollisions();

private:
	// ブロードフェーズヘルパー関数
	void UpdateAllShapes();		// 全コライダーの形状を更新
	void BuildCurrentPairs();	// 現フレームの衝突ペアを構築
	void ProcessPairEvents();	// 衝突イベント (Enter/Stay/Exit) を処理
	void ResolvePushOut(Collider* a, Collider* b);	// 接触による押し出し処理
	AABB GetSweptAABB(Collider* collider) const;	// スウェプト AABB を計算

private:
	// Narrow Phase: 各種詳細衝突判定
	void CheckSphereSphere(Collider* a, Collider* b);
	void CheckSphereBox(Collider* a, Collider* b);
	void CheckBoxBox(Collider* a, Collider* b);
	void CheckCapsuleCapsule(Collider* a, Collider* b);
	void CheckSphereCapsule(Collider* a, Collider* b);
	void CheckBoxCapsule(Collider* a, Collider* b);

	// HalfPlane との衝突判定
	void CheckSphereHalfPlane(Collider* sphere, Collider* plane);
	void CheckBoxHalfPlane(Collider* box, Collider* plane);
	void CheckCapsuleHalfPlane(Collider* capsule, Collider* plane);

	// 複合コライダー (Compound) の処理
	void CheckCompoundVsAny(Collider* compound, Collider* other);

	// 各種押し出し処理
	void PushOutSphereSphere(Collider* a, Collider* b);
	void PushOutSphereBox(Collider* a, Collider* b);
	void PushOutBoxBox(Collider* a, Collider* b);
	void PushOutCapsuleCapsule(Collider* a, Collider* b);
	void PushOutSphereCapsule(Collider* a, Collider* b);
	void PushOutBoxCapsule(Collider* a, Collider* b);
	void PushOutSphereHalfPlane(Collider* sphere, Collider* plane);
	void PushOutBoxHalfPlane(Collider* box, Collider* plane);
	void PushOutCapsuleHalfPlane(Collider* capsule, Collider* plane);

public:
	// 空間分割セルサイズ設定 (ワールド単位、デフォルト: 4)
	float GetCellSize() const noexcept { return _cellSize; }
	void SetCellSize(float cellSize) noexcept { _cellSize = (cellSize > 0.01f) ? cellSize : 0.01f; }

	// 適応的セルサイズ: コライダーの平均サイズから自動計算
	void SetAdaptiveCellSize(bool enabled) noexcept { _adaptiveCellSize = enabled; }
	bool IsAdaptiveCellSize() const noexcept { return _adaptiveCellSize; }

	// Contact 情報 (Physics エンジン用)
	struct Contact {
		Collider* a = nullptr;			// 接触相手 a (法線方向: a → b)
		Collider* b = nullptr;			// 接触相手 b
		VECTOR normal = VGet(0,0,0);	// 接触法線ベクトル
		VECTOR point  = VGet(0,0,0);	// ワールド座標系での接触点
		float penetration = 0.0f;		// ペネトレーション深度
	};

	const std::vector<Contact>& GetContacts() const noexcept { return _contacts; }

	// 登録済みコライダー一覧 (Raycast 等で使用)
	const std::vector<Collider*>& GetColliders() const noexcept { return _colliders; }

	// 指定した GameObject が所有するコライダーを検索
	Collider* FindColliderByOwner(GameObject* owner) const noexcept;

private:
	// 空間分割 (Spatial Hash) 設定
	float _cellSize = 4.0f;				// セルサイズ (ワールド座標)
	bool _adaptiveCellSize = true;		// 適応的なセルサイズ調整の有効化
	float _deltaTimeSec = 1.0f / 60.0f;	// デルタタイム

	// 登録されているコライダーの平均サイズから適応的なセルサイズを計算
	void ComputeAdaptiveCellSize();

	mutable std::mutex _mtx;	// スレッド安全用ミューテックス

	// コライダー管理
	std::vector<Collider*> _colliders{};			// 登録されているコライダー
	bool _narrowHit = false;						// 詳細判定結果フラグ (シリアルフォールバック用)
	std::vector<Contact> _contacts;					// 現フレームの接触情報
	// (_prevAABBs removed: migrated to Collider::prevAABB / hasPrevAABB)

	// SpatialPartitioning 用バッファ
	// 毎フレームの動的メモリ確保を回避するための永続バッファ群
	// 各フレーム clear() または resize() で再利用
	// 空間分割セルのキー (整数座標)
	struct CellKey {
		int x{}, y{}, z{};
		bool operator==(const CellKey& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
	};

	// CellKey のハッシュ関数
	struct CellHash {
		size_t operator()(const CellKey& k) const noexcept {
			size_t h = 1469598103934665603ull;
			h ^= static_cast<size_t>(k.x) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
			h ^= static_cast<size_t>(k.y) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
			h ^= static_cast<size_t>(k.z) + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
			return h;
		}
	};

	struct CellEntry { CellKey key; int colliderIdx; };	// セル分解結果エントリ (セルキー + コライダーインデックス)
	struct CandidatePair { int a; int b; };				// 候補ペア (コライダーインデックス a, b)

	// 空間分割用バッファ群
	std::vector<Collider*>                               _snapshotBuf;			// 形状更新用
	std::vector<Collider*>                               _activeBuf;			// フィルタリング済みコライダー
	std::vector<AABB>                                    _sweptAABBsBuf;		// スウェプト AABB
	std::vector<std::vector<CellEntry>>                  _perColliderCellsBuf;	// セル分解結果
	std::unordered_map<CellKey, std::vector<int>, CellHash> _gridBuf;			// 空間グリッド
	std::vector<CandidatePair>                           _candidatesBuf;		// 候補ペア
	std::vector<uint64_t>                                _seenPairsBuf;			// 重複排除用
	std::vector<uint8_t>                                 _perPairHitBuf;		// ナロープレイズ判定結果 (vector<bool> の並列競合回避)
	std::vector<std::vector<Contact>>                    _perPairContactsBuf;	// ナロープレイズ接触点群


	// Narrow Phase 並列化用スレッドローカル変数

	// 並列処理中の各スレッドが独立した判定結果を出力
	static thread_local bool                  _tlNarrowHit;			// 判定フラグ
	static thread_local std::vector<Contact>* _tlContactOut;			// 接触点出力バッファ (nullptr = シリアルモード)

	// 接触情報を出力 (シリアル/並列両対応)
	inline void EmitContact(const Contact& ct) {
		if (_tlContactOut) _tlContactOut->push_back(ct);
		else               _contacts.push_back(ct);
	}
};