#pragma once

#include <deque>
#include <string>
#include <cstdint>
#include <vector>

#include "SceneTpl.h"
#include "CameraController.h"

class PachinkoSensor;

// パチンコゲームのステージシーン
class PachinkoGame_StageScene : public SceneTpl<PachinkoGame_StageScene>
{
public:
	// シーン名
	static std::string StaticName() { return "PachinkoGame_StageScene"; }

	void Start() override;
	void Update(float dtSec) override;
	void Draw() override;
	void End() override;

private:
	// スタック判定用の球ごとの追跡データ
	struct BallTrack {
		GameObject* ball = nullptr;
		VECTOR      lastSnapPos = {};       // 最後のスナップショット位置
		float       accumSec = 0.0f;        // スナップショット間隔の累積時間
		int         stuckCount = 0;         // 変位不足が連続した回数
	};

	// スタック判定パラメータ
	static constexpr float kStuckSnapInterval   = 0.5f;   // スナップショット間隔（秒）
	static constexpr float kStuckMinDisplacement = 0.05f; // この距離未満なら「動いていない」と判定
	static constexpr int   kStuckCountThreshold  = 4;     // 何回連続で変位不足ならスタックとみなすか

	// センサー登録ヘルパ
	void SpawnSensors_();

	// フラグ
	bool _returningToMenu = false; // メニューへの遷移が開始済みか（多重遷移防止）

	// Ball関連
	float _ballSpawnInterval = 0.3f; // 鉄球の自動生成間隔（秒）
	int _ballCount = 0;              // 生成シリアル
	int _activeBallCount = 0;        // 現在アクティブな鉄球数
	std::deque<GameObject*> _liveBalls; // 追跡中の鉄球（Pool返却管理）
	std::deque<BallTrack>   _ballTracks; // _liveBalls と 1:1 対応

	// センサー関連
	std::vector<PachinkoSensor*> _sensors;  // 配置済みセンサー一覧
	int _totalScore = 0;                    // 累計入賞スコア

#ifdef _DEBUG
	// (チートモード)
	bool _isCheatMode = false; // チートモードかどうか
#endif // _DEBUG

	// カメラ関連
	std::uint32_t _cameraId = 0;        // カメラID
	CameraController _cameraController; // カメラコントローラ
	bool _freeCameraMode = false;       // フリームーブカメラモードかどうか
	// 固定カメラの位置と注視点(初期値)
	VECTOR _fixedCameraEye    = VGet(0.0f, 7.0f, 12.3f);
	VECTOR _fixedCameraTarget = VGet(0.0f, 7.0f, 0.0f);
};