#pragma once

#include <vector>
#include <atomic>
#include <mutex>
#include <future>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <algorithm>

#include "DxLib.h"
#include "Manager.h"

class PhysicsController;
class PhysicsBody;
class GameObject;
class Collider;

// ============================================================
//  SolverContact ? 永続コンタクト + ウォームスタート用キャッシュ
// ============================================================
// Havok / Box2D と同じ手法:
//   - 各接触点ごとに累積インパルス (normalLambda, frictionLambda) を保持
//   - フレーム間で PairKey + 接触点ローカル座標が近いものを照合 (matching)
//   - ソルバーイテレーション開始時に前フレームの lambda で速度を初期化 (warm-start)
//   - イテレーション中は Δλ ではなく累積λ をクランプ (λ_new = max(λ_old + Δλ, 0))
struct SolverContact {
	// ============================================================
	// === ホットパス（ソルバー反復で毎回アクセス）: 先頭 128B = 2 キャッシュライン ===
	// ============================================================

	// Body ポインタ・逆質量（インパルス適用に必須）
	PhysicsBody* bodyA = nullptr;       // @ 0   (8B)
	PhysicsBody* bodyB = nullptr;       // @ 8   (8B)
	float invA = 0.0f;                  // @ 16  (4B)
	float invB = 0.0f;                  // @ 20  (4B)

	// 有効逆質量（BuildSolverContacts で事前計算）
	float effectiveInvMassN  = 0.0f;   // @ 24  (4B)
	float effectiveInvMassT1 = 0.0f;   // @ 28  (4B)
	float effectiveInvMassT2 = 0.0f;   // @ 32  (4B)

	// バイアス・摩擦係数
	float normalBias     = 0.0f;       // @ 36  (4B)
	float restitution    = 0.0f;       // @ 40  (4B)
	float friction       = 0.0f;       // @ 44  (4B)
	float staticFriction = 0.0f;       // @ 48  (4B)

	// 累積インパルス（warm-start / クランプ用）
	float normalLambda    = 0.0f;      // @ 52  (4B)
	float frictionLambda1 = 0.0f;      // @ 56  (4B)
	float frictionLambda2 = 0.0f;      // @ 60  (4B)
	//  ↑ ここまで 64B = キャッシュライン 1

	// 法線・腕ベクトル・接線（速度計算・インパルス適用に使用）
	VECTOR normal   = VGet(0,1,0);     // @ 64  (12B)
	VECTOR rA       = VGet(0,0,0);     // @ 76  (12B)
	VECTOR rB       = VGet(0,0,0);     // @ 88  (12B)
	VECTOR tangent1 = VGet(1,0,0);     // @ 100 (12B)
	VECTOR tangent2 = VGet(0,0,1);     // @ 112 (12B)
	bool   speculative = false;        // @ 124 (1B + 3B padding)
	//  ↑ ここまで 128B = キャッシュライン 1-2

	// ============================================================
	// === コールドパス（位置補正・マッチングのみ使用）: 128B 以降 ===
	// ============================================================

	// Split impulse（位置補正フェーズのみ）
	float splitNormalLambda = 0.0f;    // @ 128
	float splitBias         = 0.0f;    // @ 132
	float penetration       = 0.0f;    // @ 136

	// 転がり抵抗の累積角インパルス（接触法線まわりの回転暴走を抑える）
	VECTOR rollingLambda = VGet(0, 0, 0);

	// アイランド ID
	int islandId = -1;                 // @ 140

	// 接触点・ローカル座標（Warm-start マッチング用）
	VECTOR point  = VGet(0,0,0);       // @ 144
	VECTOR localA = VGet(0,0,0);       // @ 156
	VECTOR localB = VGet(0,0,0);       // @ 168

	// コライダー識別子（マッチング・イベント用）
	Collider* colA = nullptr;          // @ 180
	Collider* colB = nullptr;          // @ 188
	// 合計 196B = 3 キャッシュライン（ホットデータは先頭 2 本に収まる）
};

// ============================================================
//  Island ? 接触グラフから導出した独立ボディ群
// ============================================================
struct PhysicsIsland {
	std::vector<PhysicsBody*> bodies;
	std::vector<int> contactIndices; // _solverContacts へのインデックス
	// Graph-coloring constraint batches: contacts within each batch
	// do not share bodies, so they can be solved in parallel.
	std::vector<std::vector<int>> constraintBatches;
	bool allSleeping = false;
};

// ============================================================
//  PhysicsManager
// ============================================================
class PhysicsManager : public Manager
{
private:
	PhysicsManager() = default;
	virtual ~PhysicsManager() = default;

public:
	static PhysicsManager& Instance() {
		static PhysicsManager inst;
		return inst;
	}

	PhysicsManager(const PhysicsManager&) = delete;
	PhysicsManager& operator=(const PhysicsManager&) = delete;

	// Manager
	void Initialize() override {}
	void Shutdown() override;
	void Update(float dt) override;

	bool IsShuttingDown() const noexcept { return _shuttingDown.load(std::memory_order_relaxed); }

	void Register(PhysicsController* controller);
	void Unregister(PhysicsController* controller);
	void RegisterBody(PhysicsBody* body);
	void UnregisterBody(PhysicsBody* body);

	// モニター用公開API
	const std::vector<PhysicsBody*>& GetBodies() const noexcept { return _bodies; }
	const std::vector<PhysicsIsland>& GetIslands() const noexcept { return _islands; }

	// Ground half-plane
	void SetGroundPlaneEnabled(bool enabled) noexcept { _groundPlaneEnabled = enabled; }
	bool IsGroundPlaneEnabled() const noexcept { return _groundPlaneEnabled; }
	void SetGroundPlaneY(float y) noexcept { _groundPlaneNormal = VGet(0,1,0); _groundPlaneD = y; }
	float GroundPlaneY() const noexcept { return _groundPlaneD; }
	void SetGroundPlane(const VECTOR& normal, float d) noexcept { _groundPlaneNormal = normal; _groundPlaneD = d; }
	VECTOR GroundPlaneNormal() const noexcept { return _groundPlaneNormal; }
	float GroundPlaneD() const noexcept { return _groundPlaneD; }

	void SetFixedDeltaTime(float fixedDeltaTime) noexcept;
	float FixedDeltaTime() const noexcept { return _fixedDeltaTime; }
	void SetMaxSubSteps(int maxSubSteps) noexcept;
	int MaxSubSteps() const noexcept { return _maxSubSteps; }
	void SetSolverIterations(int solverIterations) noexcept;
	int SolverIterations() const noexcept { return _solverIterations; }

	// Split impulse position correction (alternative to Baumgarte)
	void SetSplitImpulseEnabled(bool enabled) noexcept { _splitImpulseEnabled = enabled; }
	bool IsSplitImpulseEnabled() const noexcept { return _splitImpulseEnabled; }

	// Speculative CCD (predict contacts before penetration occurs)
	void SetSpeculativeCcdEnabled(bool enabled) noexcept { _speculativeCcdEnabled = enabled; }
	bool IsSpeculativeCcdEnabled() const noexcept { return _speculativeCcdEnabled; }

	// Havok-style TOI-based CCD (Time of Impact with backstep)
	void SetHavokCcdEnabled(bool enabled) noexcept { _havokCcdEnabled = enabled; }
	bool IsHavokCcdEnabled() const noexcept { return _havokCcdEnabled; }

	// Interpolation: call after Update() to compute interpolated transforms
	// alpha = accumulator / fixedDeltaTime (fraction of pending sub-step)
	void ComputeInterpolation() noexcept;
	float InterpolationAlpha() const noexcept;

private:
	void StepSimulation(float stepDt);
	void IntegrateBodies(float stepDt);
	void UpdateSleepState(PhysicsBody* body, float stepDt);
	void ApplyBodyConstraints(PhysicsBody* body) const;
	PhysicsBody* FindBodyByOwner(GameObject* owner) const;

	// --- ルックアップキャッシュ ---
	void BuildLookupCaches();
	PhysicsBody* CachedFindBody(GameObject* owner) const;
	Collider* CachedFindCollider(GameObject* owner) const;
	std::unordered_map<GameObject*, PhysicsBody*> _bodyByOwner{};
	std::unordered_map<GameObject*, Collider*>    _colliderByOwner{};

	// --- SoA キャッシュ (IntegrateBodies 高速化用) ---
	// AoS (PhysicsBody*) を連続メモリの SoA に展開し、
	// ParallelFor のキャッシュヒット率を向上させる。
	struct BodySoA {
		std::vector<VECTOR> position;
		std::vector<VECTOR> velocity;
		std::vector<VECTOR> angularVelocity;
		std::vector<VECTOR> force;
		std::vector<VECTOR> torque;
		std::vector<float>  inverseMass;
		std::vector<float>  linearDamping;
		std::vector<float>  angularDamping;
		std::vector<float>  gravityScale;
		std::vector<uint8_t> flags; // BodyFlag を OR したビット集合 (BodyFlag を参照)

		// SoA フラグ用ビット定義 (BitOperation と組み合わせて使う)
		struct BodyFlag {
			static constexpr uint8_t Active      = 1 << 0; // _enabled && owner active
			static constexpr uint8_t Kinematic   = 1 << 1;
			static constexpr uint8_t Sleeping    = 1 << 2;
			static constexpr uint8_t UseGravity  = 1 << 3;
			static constexpr uint8_t FreezeRot   = 1 << 4;
			static constexpr uint8_t Ccd         = 1 << 5;
		};

		void Resize(size_t n) {
			position.resize(n); velocity.resize(n); angularVelocity.resize(n);
			force.resize(n); torque.resize(n);
			inverseMass.resize(n); linearDamping.resize(n);
			angularDamping.resize(n); gravityScale.resize(n);
			flags.resize(n);
		}
	};
	BodySoA _bodySoA{};
	void GatherBodySoA();
	void ScatterBodySoA(float stepDt);

	// --- ソルバーコンタクト ---
	// ColliderManager の生 Contact から SolverContact を構築し、
	// 前フレームの累積インパルスを照合してウォームスタート。
	void BuildSolverContacts(float stepDt);
	void WarmStart();
	void SolveIsland(const PhysicsIsland& island, float stepDt);
	void SolveRollingFriction(SolverContact& sc);
	void SolveAllIslands(float stepDt);
	void PositionalCorrection(float stepDt, float depthThreshold = 0.005f, float biasScale = 1.0f);
	void SplitImpulseCorrection(float stepDt);
	void GenerateSpeculativeContacts(float stepDt);
	void ResolveToiEvents(float stepDt);
	void PropagateIslandSleep();

	std::vector<SolverContact> _solverContacts{};
	std::vector<SolverContact> _prevSolverContacts{}; // 前フレーム（マッチング用）

	// --- アイランド ---
	void BuildIslands();
	void SplitLargeIsland(int islandIdx, int maxBodiesPerSplit);
	void BuildConstraintBatches(PhysicsIsland& island);
	std::vector<PhysicsIsland> _islands{};
	std::unordered_map<PhysicsBody*, int> _bodyIslandMap{}; // body → islandId

	// Island 再利用プール: スロー再構築時、破棄せずバッファを温存
	std::vector<PhysicsIsland> _islandPool{};
	int AcquireIsland();          // _islands に 1 スロット追加（プール優先）
	void RecycleAllIslands();     // _islands の中身をプールへ退避し _islands を空に

	// Union-Find with rank for O(α(n)) amortized merges
	std::vector<int> _ufParent{};
	std::vector<int> _ufRank{};
	int UFFind(int x) noexcept;
	void UFUnite(int a, int b) noexcept;

	// BuildIslands の変化検出キャッシュ（前フレームとグラフが同じなら再構築スキップ）
	size_t _prevIslandBodyCount = 0;
	size_t _prevContactHash     = ~size_t(0); // 初回は必ず再構築

	// Island splitting threshold: islands larger than this are split for parallel PGS
	// With ~50 objects, splitting small islands wastes more time than it saves.
	static constexpr int kIslandSplitThreshold = 64;
	// Constraint batching threshold: islands with more contacts get graph-colored batches
	static constexpr int kBatchingThreshold = 32;
	// 小バッチのシリアル実行閾値: これ未満のバッチは ParallelForBarrier を使わずシリアル解
	// バリアのセットアップコスト（notify_all + spin）が並列効果を上回る下限
	static constexpr int kBatchParallelThreshold = 48;

private:
	std::atomic_bool _shuttingDown{ false };
	mutable std::mutex _mtx;
	std::vector<PhysicsController*> _controllers{};
	std::vector<PhysicsBody*> _bodies{};

	// Gravity (arbitrary vector)
	VECTOR _gravity = VGet(0.0f, -9.8f, 0.0f);

public:
	void SetGravity(const VECTOR& g) noexcept { _gravity = g; }
	VECTOR GetGravity() const noexcept { return _gravity; }

	// Adaptive solver iterations (4..16)
	void SetAdaptiveIterationRange(int minIter, int maxIter) noexcept {
		_minSolverIterations = (minIter >= 1) ? minIter : 1;
		_maxSolverIterations = (maxIter >= _minSolverIterations) ? maxIter : _minSolverIterations;
	}

private:
	int _minSolverIterations = 4;
	int _maxSolverIterations = 16;
	int ComputeAdaptiveIterations() const noexcept;

	bool _groundPlaneEnabled = false;
	VECTOR _groundPlaneNormal = VGet(0.0f, 1.0f, 0.0f);
	float _groundPlaneD = 0.0f; // n . x = d
	bool _splitImpulseEnabled = true;
	bool _speculativeCcdEnabled = true;
	bool _havokCcdEnabled = true;

	float _fixedDeltaTime = 1.0f / 120.0f;
	int _maxSubSteps = 8;
	int _solverIterations = 10;
	float _accumulator = 0.0f;

public:


	//  Async physics mode (double-buffered)

	//  When enabled, StepSimulation runs on a background thread.
	//  Main thread uses the previous frame's interpolated transforms
	//  while the physics thread computes the current frame.
	//  Call WaitForPhysics() before reading physics state if needed.
	void SetAsyncEnabled(bool enabled) noexcept { _asyncEnabled = enabled; }
	bool IsAsyncEnabled() const noexcept { return _asyncEnabled; }
	void WaitForPhysics();

private:
	bool _asyncEnabled = false;
	std::future<void> _asyncFuture{};
	void RunAsyncStep(float dt, int maxSubSteps);

	// ============================================================
	//  永続バッファ（毎フレームの heap 確保を抑制）
	//  clear()/resize() のみ行い capacity は保持する。
	// ============================================================
	// BuildSolverContacts 用
	struct PrevKey {
		Collider* a; Collider* b;
		bool operator==(const PrevKey& o) const noexcept { return a == o.a && b == o.b; }
	};
	struct PrevHash {
		size_t operator()(const PrevKey& k) const noexcept {
			return (reinterpret_cast<size_t>(k.a) >> 4) ^ (reinterpret_cast<size_t>(k.b) << 1);
		}
	};
	struct TaggedContact { SolverContact sc; bool valid = false; };
	// ResolveToiEvents 用
	struct ToiEvent {
		PhysicsBody* body = nullptr;
		float toi = 1.0f;
		VECTOR toiPosition{};
		VECTOR clampedVelocity{};
	};

	std::vector<PhysicsController*>                       _ctrlSnapshotBuf;
	std::unordered_multimap<PrevKey, size_t, PrevHash>    _prevContactMapBuf;
	std::vector<TaggedContact>                            _buildContactResultsBuf;
	std::vector<size_t>                                   _islandOrderBuf;
	std::unordered_map<PhysicsBody*, int>                 _bodyIndexBuf;
	std::unordered_map<int, int>                          _rootToIslandBuf;
	std::vector<VECTOR>                                   _pseudoVelBuf;
	std::vector<VECTOR>                                   _posCorrectionBuf;
	std::unordered_map<PhysicsBody*, size_t>              _bodyIdxBuf;
	std::vector<ToiEvent>                                 _toiEventsBuf;
	std::unordered_set<PhysicsBody*>                      _specCoveredBodiesBuf;
	std::vector<SolverContact>                            _specContactsBuf;

	// ============================================================
	//  SplitLargeIsland 永続バッファ（毎呼出の heap 確保を排除）
	// ============================================================
	std::unordered_map<PhysicsBody*, int> _splitLocalIdxBuf;
	std::vector<std::vector<int>>         _splitAdjBuf;
	std::vector<int>                      _splitColorBuf;
	std::vector<int>                      _splitQueueBuf;
	std::vector<PhysicsBody*>             _splitOrigBodiesBuf;
	std::vector<int>                      _splitOrigContactsBuf;

	// ============================================================
	//  BuildConstraintBatches 永続バッファ（毎呼出の heap 確保を排除）
	// ============================================================
	std::unordered_map<PhysicsBody*, std::vector<int>> _batchBodyToContactsBuf;
	std::vector<int>                                   _batchContactColorBuf;
	std::vector<int>                                   _batchUsedColorEpochBuf;

	// Island sorting cache (to avoid re-sorting when island count hasn't changed)
	size_t _prevIslandCount = 0;
};
