#include "DxLib.h"
#include "Info.h"
#include "Assert.h"
#include "SceneManager.h"
#include "TitleScene.h"
#include "Time.h"
#include "KeyInput.h"
#include "ObjectManager.h"
#include "CameraManager.h"
#include "ColliderManager.h"
#include "PhysicsManager.h"
#include "SeManager.h"
#include "BgmManager.h"
#include "SceneTransition.h"
#include "PerformanceMonitor.h"
#include "PhysicsMonitor.h"

void LightingInit() {
	SetUseZBuffer3D(TRUE);
	SetWriteZBuffer3D(TRUE);
}

// プログラムは WinMainから始まります
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	// VSync無効
	SetWaitVSyncFlag(FALSE);

	// ウィンドウモードに設定
	ChangeWindowMode(TRUE);

	// 画面モードの設定
	SetGraphMode(WINDOW_WIDTH, WINDOW_HEIGHT,32);

	SetWindowText("3D_GAME_Project");

	// DxLibの初期化
	if (DxLib_Init() == -1) {
		ASSERT_MSG(false, "DxLib_Init()に失敗しました");
		return -1;
	}

	// lightingの初期化
	LightingInit();

	// 各種マネージャの初期化
	Time::Instance().Reset();

	// キー入力の初期化とリピート設定
	KeyInput::Instance().BeginKeyInput();
	KeyInput::Instance().SetInputRepeatedTime(KEY_INPUT_RETURN, 0.2);

	// 最初のシーンをタイトルシーンに設定
	SceneManager::Instance().ChangeScene(std::make_unique<TitleScene>()); // 最初のシーンをタイトルシーンに設定

	// シーンIDを ObjectManager に通知（シーン切替ごとに更新する必要がある）
	ObjectManager::Instance().SetCurrentSceneId(SceneManager::Instance().CurrentSceneId());

	// 最初のカメラを作成（シーンIDはタイトルシーンのもの）
	CameraManager::Instance().CreateCamera(SceneManager::Instance().CurrentSceneId());

	// モニター自動保存設定
	PerformanceMonitor::Instance().EnableDetailedLogging(true);
	PerformanceMonitor::Instance().SetAutoSaveInterval(5.0f); // 5秒ごと
	PhysicsMonitor::Instance().EnableAutoSave(true);
	PhysicsMonitor::Instance().SetAutoSaveInterval(5.0f); // 5秒ごと

	constexpr double kPoolTrimIntervalSec =1.0; // プールトリム間隔（秒）
	constexpr double kPoolMaxIdleSec =10.0;     // プール最大アイドル時間（秒）	
	double poolTrimAccumSec = 0.0;				// トリム間隔の累積時間

	// メインループ
	while (ProcessMessage() ==0 && CheckHitKey(KEY_INPUT_ESCAPE) ==0)
	{
		Time::Instance().Update();
		const float dt = static_cast<float>(Time::Instance().GetDeltaTime());

		// パフォーマンスモニタ：フレーム開始
		PerformanceMonitor::Instance().BeginFrame();

		// キー入力の更新
		KeyInput::Instance().Update(dt);

		// シーンの更新（遷移中は SceneTransition::Update() 内で更新する）
		SceneManager::Instance().Update(dt);

		//物理(重力・速度積分)
		PhysicsManager::Instance().Update(dt);

		// シーン遷移の更新（遷移中は SceneManager::Update() を呼ばない）
		SceneTransition::Instance().Update(dt);

		// カメラ更新（Blend中は BlendのUpdate内で更新する）
		CameraManager::Instance().Update(dt);

		// リスナー位置の更新（カメラに追従）
		if (auto* cam = CameraManager::Instance().Render()) {
			SeManager::Instance().SetListener(&cam->transform);
		}

		// SE/BGM 更新
		SeManager::Instance().Update(dt);
		BgmManager::Instance().Update(dt);

		// パフォーマンスモニタ更新
		PerformanceMonitor::Instance().Update();

		// 物理モニタ更新
		PhysicsMonitor::Instance().Update(dt);

		// プールの定期トリム
		poolTrimAccumSec += dt;
		if (poolTrimAccumSec >= kPoolTrimIntervalSec) {
			ObjectManager::Instance().TrimAllPoolsUnused(kPoolMaxIdleSec);
			poolTrimAccumSec =0.0;
		}

		ClearDrawScreen();

		{
			int width =0, height =0;
			GetDrawScreenSize(&width, &height);
			CameraManager::Instance().ApplyRenderCameraToDxLib(width, height);
		}

		SceneTransition::Instance().Draw();

#ifdef _DEBUG
		{
			int width =0, height =0;
			GetDrawScreenSize(&width, &height);
			const int x =10;
			const int y = height -140;
			ObjectManager::Instance().DebugDraw(x, y);
		}

		// パフォーマンスモニタ描画 (F3 で ON/OFF)
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_F3)) {
			PerformanceMonitor::Instance().ToggleVisible();
		}
		PerformanceMonitor::Instance().Draw(10, 10);

		// 物理モニタ描画 (F4 で ON/OFF)
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_F4)) {
			PhysicsMonitor::Instance().ToggleVisible();
		}
		PhysicsMonitor::Instance().Draw(500, 10);

		// ログ保存 (F5: 物理、F6: パフォーマンス)
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_F5)) {
			PhysicsMonitor::Instance().SaveDetailedLog();
		}
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_F6)) {
			PerformanceMonitor::Instance().SaveDetailedLog();
		}
#endif

		ScreenFlip();

		SceneManager::Instance().ProcessPendingChanges();
	}

	// 各種マネージャの終了処理
	ColliderManager::Instance().Shutdown();
	PhysicsManager::Instance().Shutdown();

	// キー入力の終了処理
	KeyInput::Instance().EndKeyInput();

	DxLib_End();
	return 0;
}
