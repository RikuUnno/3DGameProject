#include "MenuScene.h"
#include "TitleScene.h"
#include "DxLib.h"
#include "KeyInput.h"
#include "SceneManager.h"

#include "ObjectFactory.h"
#include "ObjectController.h"
#include "ObjectManager.h"
#include "GameObject.h"

#include "CameraController.h"
#include "CameraManager.h"
#include "CameraTags.h"

#include <memory>
#include <string>

namespace {
	// このシーン内だけで動く「生成/プール」デモ用の簡易 GameObject
	class DemoObject final : public GameObject {
	public:
		void OnAcquire(const VariantMap& params) override {
			label_ = "DemoObject";
			auto it = params.find("label");
			if (it != params.end()) label_ = it->second;
		}

		void OnRelease() override {
			label_.clear();
		}

		void Draw() override {
			DrawString(10,40, label_.c_str(), GetColor(200,255,200));
		}

	private:
		std::string label_;
	};

	ObjectController g_controller;
	int g_demoCreateCount =0;

	// カメラデバッグ用
	CameraController g_camCtrl;
	CameraController::CameraId g_debugCamId =0;
	CameraController::CameraId g_gameCamId =0;

	// 切替デモ用のフラグ
	bool g_useBlend = true;
	float g_blendSec =0.4f;
	CameraController::CameraId g_currentCamId =0;

	//3Dデバッグ用の簡易描画
	void DrawSimple3DDebug() {
		const int half =10;
		const float step =1.0f;
		const unsigned int colGrid = GetColor(60,60,60);
		for (int i = -half; i <= half; ++i) {
			const float x = i * step;
			DrawLine3D(VGet(x,0.0f, -half * step), VGet(x,0.0f, half * step), colGrid);
			const float z = i * step;
			DrawLine3D(VGet(-half * step,0.0f, z), VGet(half * step,0.0f, z), colGrid);
		}

		DrawLine3D(VGet(0,0,0), VGet(2,0,0), GetColor(255,80,80));
		DrawLine3D(VGet(0,0,0), VGet(0,2,0), GetColor(80,255,80));
		DrawLine3D(VGet(0,0,0), VGet(0,0,2), GetColor(80,80,255));

		DrawSphere3D(VGet(0.0f,0.5f,3.5f),0.5f,16, GetColor(200,200,80), GetColor(200,200,80), TRUE);
		DrawSphere3D(VGet(2.5f,1.0f, -1.5f),0.6f,16, GetColor(80,200,200), GetColor(80,200,200), TRUE);
	}
}

void MenuScene::Start() {
	g_controller.SpawnAuto(
		"DemoObject",
		[](const VariantMap&) {
			++g_demoCreateCount;
			return std::make_unique<DemoObject>();
		},
		8,
		{ {"label", "DemoObject生成（シーン開始）"} }
	);

	auto& camMgr = CameraManager::Instance();
	const int sceneId = SceneManager::Instance().CurrentSceneId();

	// Debug camera
	if (g_debugCamId == 0 || camMgr.Get(g_debugCamId) == nullptr) {
		g_debugCamId = g_camCtrl.SpawnAuto(
			sceneId,
			CameraTag::Debug,
			VGet(0.0f, 2.0f, -8.0f),
			VGet(0.0f, 0.0f, 0.0f)
		);
	}

	// Game camera（tmp をやめる）
	if (g_gameCamId == 0 || camMgr.Get(g_gameCamId) == nullptr) {
		// いったん操作対象を Game に切り替えて Spawn
		g_camCtrl.SetCamera(0);
		g_gameCamId = g_camCtrl.SpawnAuto(
			sceneId,
			CameraTag::Game,
			VGet(6.0f, 3.0f, -6.0f),
			VGet(0.0f, 0.8f, 0.0f)
		);
	}

	g_currentCamId = g_debugCamId;
	camMgr.SetRender(g_currentCamId);
}

void MenuScene::Update() {
	g_controller.UpdateAll();

	// カメラ操作（デバッグカメラのみ動かす）
	g_camCtrl.SetCamera(g_debugCamId);
	g_camCtrl.UpdateFreeMoveQuat(6.0f,1.6f);

	// 切替モード切替
	// B: ブレンドON/OFF
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_B)) {
		g_useBlend = !g_useBlend;
	}

	//1: Debugカメラへ
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_1)) {
		g_currentCamId = g_debugCamId;
		if (g_useBlend) {
			CameraManager::Instance().BlendRenderTo(g_currentCamId, g_blendSec);
		} else {
			CameraManager::Instance().SetRender(g_currentCamId);
		}
	}
	//2: Gameカメラへ
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_2)) {
		g_currentCamId = g_gameCamId;
		if (g_useBlend) {
			CameraManager::Instance().BlendRenderTo(g_currentCamId, g_blendSec);
		} else {
			CameraManager::Instance().SetRender(g_currentCamId);
		}
	}

	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_SPACE)) {
		g_controller.ReleaseAll();
		SceneManager::Instance().ChangeScene(std::make_unique<TitleScene>());
	}
}

void MenuScene::Draw() {
	DrawSimple3DDebug();

	DrawString(10,10, "メニューシーン - Spaceで戻る", GetColor(255,255,255));
	DrawString(10,25, "[デモ] ObjectController: Spawn/Update/Draw/ReleaseAll", GetColor(180,255,180));
	DrawString(10,70, "[カメラ]1:Debug2:Game B:Blend ON/OFF", GetColor(180,180,255));
	DrawFormatString(10,90, GetColor(180,180,255), "[カメラ] Blend: %s sec=%.2f", g_useBlend?"ON":"OFF", g_blendSec);
	DrawString(10,110, "[操作] 矢印:回転 / WASD+QE:移動（Debugカメラのみ）", GetColor(180,180,255));
	DrawFormatString(10,130, GetColor(180,180,255), "[Render] current=%d blending=%s", (int)g_currentCamId, CameraManager::Instance().IsBlending()?"YES":"NO");
	DrawFormatString(10,50, GetColor(255,255,0), "[デモ] Factory new回数: %d", g_demoCreateCount);

	g_controller.DrawAll();
}