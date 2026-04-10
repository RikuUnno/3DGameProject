#pragma once
#pragma once

#include <vector>
#include <atomic>
#include <mutex>
#include <unordered_map>
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
	// 識別
	Collider* colA = nullptr;
	Collider* colB = nullptr;

	// 幾何
	VECTOR normal   = VGet(0,1,0);
	VECTOR point    = VGet(0,0,0);
	VECTOR rA       = VGet(0,0,0); // point - centerA
	VECTOR rB       = VGet(0,0,0); // point - centerB
	float  penetration = 0.0f;

	// キャッシュ済みソルバー定数（BuildSolverContacts で計算）
	float effectiveInvMassN  = 0.0f; // 法線方向
	float effectiveInvMassT1 = 0.0f; // 摩擦方向1
	float effectiveInvMassT2 = 0.0f; // 摩擦方向2
	float restitution = 0.0f;
	float friction    = 0.0f;
	float staticFriction = 0.0f; // static friction (for static/kinetic transition)
	VECTOR tangent1 = VGet(1,0,0);
	VECTOR tangent2 = VGet(0,0,1);
	float normalBias = 0.0f; // Baumgarte velocity bias

	// Body ポインタ（毎フレーム解決）
	PhysicsBody* bodyA = nullptr;
	PhysicsBody* bodyB = nullptr;
	float invA = 0.0f;
	float invB = 0.0f;

	// 累積インパルス（warm-start / クランプ用）
	float normalLambda   = 0.0f;
	float frictionLambda1 = 0.0f;
	float frictionLambda2 = 0.0f;

	// Split impulse (position correction without affecting velocity)
	float splitNormalLambda = 0.0f;
	float splitBias = 0.0f;

	// 接触点のローカル座標（マッチング用）
	VECTOR localA = VGet(0,0,0); // colA ローカル
	VECTOR localB = VGet(0,0,0); // colB ローカル

	// アイランドID（BuildIslands で割り当て）
	int islandId = -1;

	// Speculative CCD: true if this contact was generated speculatively
	// (penetration was predicted, not measured)
	bool speculative = false;
};

// ============================================================
//  Island ? 接触グラフから導出した独立ボディ群
// ============================================================
struct PhysicsIsland {
	std::vector<PhysicsBody*> bodies;
	std::vector<int> contactIndices; // _solverContacts へのインデックス
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

	// --- ソルバーコンタクト ---
	// ColliderManager の生 Contact から SolverContact を構築し、
	// 前フレームの累積インパルスを照合してウォームスタート。
	void BuildSolverContacts(float stepDt);
	void WarmStart();
	void SolveIsland(const PhysicsIsland& island, float stepDt);
	void SolveAllIslands(float stepDt);
	void PositionalCorrection(float stepDt);
	void SplitImpulseCorrection(float stepDt);
	void GenerateSpeculativeContacts(float stepDt);
	void ResolveToiEvents(float stepDt);
	void PropagateIslandSleep();

	std::vector<SolverContact> _solverContacts{};
	std::vector<SolverContact> _prevSolverContacts{}; // 前フレーム（マッチング用）

	// --- アイランド ---
	void BuildIslands();
	std::vector<PhysicsIsland> _islands{};
	std::unordered_map<PhysicsBody*, int> _bodyIslandMap{}; // body → islandId

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

	float _fixedDeltaTime = 1.0f / 120.0f;
	int _maxSubSteps = 8;
	int _solverIterations = 6;
	float _accumulator = 0.0f;
};
