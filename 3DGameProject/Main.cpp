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

void LightingInit() {
	SetUseZBuffer3D(TRUE);
	SetWriteZBuffer3D(TRUE);
}

// プログラムは WinMainから始まります
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	ChangeWindowMode(TRUE);

	SetGraphMode(WINDOW_WIDTH, WINDOW_HEIGHT,32);

	SetWindowText("3D_GAME_Project");

	if (DxLib_Init() == -1) {
		ASSERT_MSG(false, "DxLib_Init()に失敗しました");
		return -1;
	}

	LightingInit();

	Time::Instance().Reset();

	KeyInput::Instance().BeginKeyInput();
	KeyInput::Instance().SetInputRepeatedTime(KEY_INPUT_RETURN,0.2);

	SceneManager::Instance().ChangeScene(std::make_unique<TitleScene>());

	ObjectManager::Instance().SetCurrentSceneId(SceneManager::Instance().CurrentSceneId());

	CameraManager::Instance().CreateCamera(SceneManager::Instance().CurrentSceneId());

	constexpr double kPoolTrimIntervalSec =1.0;
	constexpr double kPoolMaxIdleSec =10.0;
	double poolTrimAccumSec =0.0;

	while (ProcessMessage() ==0 && CheckHitKey(KEY_INPUT_ESCAPE) ==0)
	{
		Time::Instance().Update();
		const float dt = static_cast<float>(Time::Instance().GetDeltaTime());

		KeyInput::Instance();

		SceneManager::Instance().Update();

		//物理(重力・速度積分)
		PhysicsManager::Instance().Update(dt);

		SceneTransition::Instance().Update(Time::Instance().GetDeltaTime());

		CameraManager::Instance().Update(dt);

		if (auto* cam = CameraManager::Instance().Render()) {
			SeManager::Instance().SetListener(&cam->transform);
		}

		SeManager::Instance().Update();

		poolTrimAccumSec += Time::Instance().GetDeltaTime();
		if (poolTrimAccumSec >= kPoolTrimIntervalSec) {
			ObjectManager::Instance().TrimAllPoolsUnused(kPoolMaxIdleSec);
			poolTrimAccumSec =0.0;
		}

		ClearDrawScreen();

		{
			int w =0, h =0;
			GetDrawScreenSize(&w, &h);
			CameraManager::Instance().ApplyRenderCameraToDxLib(w, h);
		}

		SceneTransition::Instance().Draw();

#ifdef _DEBUG
		{
			int w =0, h =0;
			GetDrawScreenSize(&w, &h);
			const int x =10;
			const int y = h -140;
			ObjectManager::Instance().DebugDraw(x, y);
		}
#endif

		ScreenFlip();

		SceneManager::Instance().ProcessPendingChanges();
	}

	ColliderManager::GetInstance().Shutdown();
	PhysicsManager::Instance().Shutdown();

	KeyInput::Instance().EndKeyInput();

	DxLib_End();
	return 0;
}
