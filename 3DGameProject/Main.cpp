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

void LightingInit() {
	SetUseZBuffer3D(TRUE);
	SetWriteZBuffer3D(TRUE);
}

// プログラムは WinMainから始まります
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	ChangeWindowMode(TRUE); // ウインドウモードで起動

	SetGraphMode(WINDOW_WIDTH, WINDOW_HEIGHT,32); //画面サイズのセット

	SetWindowText("3D_GAME_Project"); // ウィンドウの名前（現在は仮）

	// DXライブラリの初期化処理
	if (DxLib_Init() == -1) {
		ASSERT_MSG(false, "DxLib_Init()に失敗しました");
		return -1; // エラーが発生した場合は -1 を返して終了
	}

	// ライティングの初期化
	LightingInit();

	// Time の初期化（カウント開始）
	Time::Instance().Reset();

	// KeyInput を有効化
	KeyInput::Instance().BeginKeyInput();
	// Enter のリピート間隔を0.2 秒に設定
	KeyInput::Instance().SetInputRepeatedTime(KEY_INPUT_RETURN,0.2);

	// 最初のシーンをセット
	SceneManager::Instance().ChangeScene(std::make_unique<TitleScene>());

	// ObjectManager 側に現在シーンIDを反映（初期シーンに紐づけて Spawnできるようにする）
	ObjectManager::Instance().SetCurrentSceneId(SceneManager::Instance().CurrentSceneId());

	// デフォルトのカメラを1台作成（シーンIDに紐づける）
	CameraManager::Instance().CreateCamera(SceneManager::Instance().CurrentSceneId());

	// ---- Pool auto-trim settings ----
	constexpr double kPoolTrimIntervalSec =1.0; //何秒ごとに掃除するか
	constexpr double kPoolMaxIdleSec =10.0; //何秒未使用なら削除するか
	double poolTrimAccumSec =0.0;				 // 経過時間蓄積用

	// メインループ
	while (ProcessMessage() ==0 && CheckHitKey(KEY_INPUT_ESCAPE) ==0)
	{
		// 時間更新（必ず最初）
		Time::Instance().Update();

		// 入力
		KeyInput::Instance(); // インスタンス取得で状態更新

		// シーン更新
		SceneManager::Instance().Update();

		// カメラ補間更新（Renderのブレンドなど）
		CameraManager::Instance().Update((float)Time::Instance().GetDeltaTime());

		// ---- Pool auto-trim ----
		poolTrimAccumSec += Time::Instance().GetDeltaTime();
		if (poolTrimAccumSec >= kPoolTrimIntervalSec) {
			ObjectManager::Instance().TrimAllPoolsUnused(kPoolMaxIdleSec);
			poolTrimAccumSec =0.0;
		}
		// 描画
		ClearDrawScreen();					//画面クリア

		// レンダーカメラを適用（A/BのB: Render Camera）
		{
			int w =0, h =0;
			GetDrawScreenSize(&w, &h);
			CameraManager::Instance().ApplyRenderCameraToDxLib(w, h);
		}

		// シーン描画
		SceneManager::Instance().Draw();

		// デバッグ表示
#ifdef _DEBUG
		{
			int w =0, h =0;
			GetDrawScreenSize(&w, &h);
			const int x =10;
			const int y = h -140; //だいたい左下（行数が増えたら調整）
			ObjectManager::Instance().DebugDraw(x, y);
		}
#endif

		ScreenFlip();						// 入れ替え

		// フレーム末に保留中の遷移を処理
		SceneManager::Instance().ProcessPendingChanges();
	}

	// 終了処理
	// 終了時の静的デストラクタ（atexit）から ColliderManager に触られても落ちないように
	//先に Shutdownしてコンテナを安全化する。
	ColliderManager::GetInstance().Shutdown();

	KeyInput::Instance().EndKeyInput();
	// DXライブラリの終了処理
	DxLib_End();
	return 0;
}
