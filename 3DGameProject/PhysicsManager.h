#pragma once

#include <vector>
#include <atomic>

#include "Manager.h"

class PhysicsController;
class PhysicsBody;

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
	void Initialize() override {}	// 初期化
	void Shutdown() override; 			// 明示的終了（Main終了時の安全化）
	void Update() override {} 			// dt付き更新を基本にするため、ここはno-op

	// dt付き更新
	void Update(float dt); // 毎フレーム呼ばれる

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

private:
	std::atomic_bool _shuttingDown{ false }; 		// 終了処理ガード：終了中は Update/Register/Unregister を no-op にする
	std::vector<PhysicsController*> _controllers{}; 	// 登録された Controller のリスト
	std::vector<PhysicsBody*> _bodies{}; 			// 登録された PhysicsBody のリスト

	// 重力（ワールド）
	float _gravityY = -9.8f;

	// --- temporary stabilization ---
	// ColliderManager の押し戻しは Transform を直すが、速度は直さないため
	// とりあえず落下しっぱなしを防ぐ簡易地面を用意する。
	// デフォルトでは無効にして、接触情報に基づく速度修正を行う。
	bool _groundPlaneEnabled = false;
	float _groundPlaneY =0.0f;
};
