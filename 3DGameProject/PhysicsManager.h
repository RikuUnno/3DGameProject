#pragma once

#include <vector>
#include <atomic>
#include <mutex>

#include "DxLib.h"
#include "Manager.h"

class PhysicsController;
class PhysicsBody;
class GameObject;

// PhysicsManager
// -物理(剛体/積分/衝突解決)の全体管理を担当
// - Controller は操作(外部からの設定/デバッグ操作等)を担当
class PhysicsManager : public Manager
{
private:
	// シングルトン
	PhysicsManager() = default;
	virtual ~PhysicsManager() = default;

public:
	// シングルトンインスタンス取得
	static PhysicsManager& Instance() {
		static PhysicsManager inst;
		return inst;
	}

	// copy禁止
	PhysicsManager(const PhysicsManager&) = delete;
	PhysicsManager& operator=(const PhysicsManager&) = delete;

	// Manager
	void Initialize() override {}
	void Shutdown() override; 			// 明示的終了（Main終了時の安全化）
	void Update(float dt) override; // 毎フレーム呼ばれる

	// 終了処理ガード
	bool IsShuttingDown() const noexcept { return _shuttingDown.load(std::memory_order_relaxed); } 

	// Controller 登録/解除（必要になったら複数対応）
	void Register(PhysicsController* controller); 		// Controller を登録する。Update で呼ばれる
	void Unregister(PhysicsController* controller); 		// Controller を解除する。Update で呼ばれる

	// PhysicsBody 登録/解除（GameObject派生が保持するコンポーネントを登録する）
	void RegisterBody(PhysicsBody* body); 		// PhysicsBody を登録する。Update で呼ばれる
	void UnregisterBody(PhysicsBody* body); 		// PhysicsBody を解除する。Update で呼ばれる

	// 簡易地面（ワールドY平面）
	void SetGroundPlaneEnabled(bool enabled) noexcept { _groundPlaneEnabled = enabled; }
	bool IsGroundPlaneEnabled() const noexcept { return _groundPlaneEnabled; }
	void SetGroundPlaneY(float y) noexcept { _groundPlaneY = y; }
	float GroundPlaneY() const noexcept { return _groundPlaneY; }

	void SetFixedDeltaTime(float fixedDeltaTime) noexcept;
	float FixedDeltaTime() const noexcept { return _fixedDeltaTime; }
	void SetMaxSubSteps(int maxSubSteps) noexcept;
	int MaxSubSteps() const noexcept { return _maxSubSteps; }
	void SetSolverIterations(int solverIterations) noexcept;
	int SolverIterations() const noexcept { return _solverIterations; }

private:
	void StepSimulation(float stepDt);
	void IntegrateBodies(float stepDt);
	void SolveContacts(float stepDt);
	void UpdateSleepState(PhysicsBody* body, float stepDt);
	void ApplyBodyConstraints(PhysicsBody* body) const;
	PhysicsBody* FindBodyByOwner(GameObject* owner) const;

private:
	std::atomic_bool _shuttingDown{ false }; 		// 終了処理ガード：終了中は Update/Register/Unregister を no-op にする
	mutable std::mutex _mtx;						// スレッド安全用ミューテックス
	std::vector<PhysicsController*> _controllers{}; 	// 登録された Controller のリスト
	std::vector<PhysicsBody*> _bodies{}; 			// 登録された PhysicsBody のリスト

	// 重力（ワールド）
	float _gravityY = -9.8f;

	// 簡易地面（ワールドY平面）
	bool _groundPlaneEnabled = false;
	float _groundPlaneY = 0.0f;

	float _fixedDeltaTime = 1.0f / 120.0f;
	int _maxSubSteps = 8;
	int _solverIterations = 6;
	float _accumulator = 0.0f;
};
