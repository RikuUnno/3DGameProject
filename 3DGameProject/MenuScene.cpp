#include "MenuScene.h"
#include "TitleScene.h"
#include "DxLib.h"
#include "KeyInput.h"
#include "SceneManager.h"
#include "SceneTransition.h"

#include "ObjectFactory.h"
#include "ObjectManager.h"

#include "CameraController.h"
#include "CameraManager.h"
#include "CameraTags.h"

#include "ColliderManager.h"
#include "Debug Class.h"

#include <memory>

namespace {
	// カメラ
	CameraController g_camCtrl;
	CameraController::CameraId g_debugCamId =0;
	CameraController::CameraId g_gameCamId =0;
	CameraController::CameraId g_currentCamId =0;
	bool g_useBlend = true;
	float g_blendSec =0.4f;

	// プールからスポーンするデバッグ用オブジェクト
	DebugPlayer* g_debugPlayer = nullptr;
	DebugEnemy* g_debugEnemy = nullptr;
	DebugHat* g_debugHat = nullptr;
	DebugGround* g_debugGround = nullptr;

	// グリッド床を描画
	void DrawGridFloor(float y, int halfCells, float step) {
		const unsigned int colGrid = GetColor(60,60,60);
		for (int i = -halfCells; i <= halfCells; ++i) {
			const float x = i * step;
			DrawLine3D(VGet(x, y, -halfCells * step), VGet(x, y, halfCells * step), colGrid);
			const float z = i * step;
			DrawLine3D(VGet(-halfCells * step, y, z), VGet(halfCells * step, y, z), colGrid);
		}
		// axis
		DrawLine3D(VGet(0, y,0), VGet(2, y,0), GetColor(255,80,80));
		DrawLine3D(VGet(0, y,0), VGet(0, y +2,0), GetColor(80,255,80));
		DrawLine3D(VGet(0, y,0), VGet(0, y,2), GetColor(80,80,255));
	}
}

void MenuScene::Start() {
	auto& camMgr = CameraManager::Instance();
	const int sceneId = SceneManager::Instance().CurrentSceneId();

	// 自由カメラ（デバッグ用）
	if (g_debugCamId ==0 || camMgr.Get(g_debugCamId) == nullptr) {
		g_debugCamId = g_camCtrl.SpawnAuto(sceneId, CameraTag::Debug, VGet(0.0f,2.0f, -8.0f), VGet(0.0f,0.0f,0.0f));
	}
	// 固定カメラ（ゲーム用）
	if (g_gameCamId ==0 || camMgr.Get(g_gameCamId) == nullptr) {
		g_camCtrl.SetCamera(0);
		g_gameCamId = g_camCtrl.SpawnAuto(sceneId, CameraTag::Game, VGet(6.0f,3.0f, -6.0f), VGet(0.0f,0.8f,0.0f));
	}

	g_currentCamId = g_debugCamId;
	camMgr.SetRender(g_currentCamId);

	// 登録（最初の一回だけ）
	static bool s_registered = false;
	if (!s_registered) {
		ObjectFactory::Instance().RegisterCreator("DebugPlayer", [](const VariantMap&) { return std::make_unique<DebugPlayer>(); });
		ObjectFactory::Instance().RegisterCreator("DebugEnemy", [](const VariantMap&) { return std::make_unique<DebugEnemy>(); });
		ObjectFactory::Instance().RegisterCreator("DebugHat", [](const VariantMap&) { return std::make_unique<DebugHat>(); });
		ObjectFactory::Instance().RegisterCreator("DebugGround", [](const VariantMap&) { return std::make_unique<DebugGround>(); });
		ObjectManager::Instance().RegisterPool("DebugPlayer",8);
		ObjectManager::Instance().RegisterPool("DebugEnemy",8);
		ObjectManager::Instance().RegisterPool("DebugHat",8);
		ObjectManager::Instance().RegisterPool("DebugGround",2);
		s_registered = true;
	}

	// DebugPlayer / DebugEnemy / Hat / Ground をスポーン
	g_debugPlayer = dynamic_cast<DebugPlayer*>(ObjectManager::Instance().Spawn("DebugPlayer"));
	g_debugEnemy = dynamic_cast<DebugEnemy*>(ObjectManager::Instance().Spawn("DebugEnemy"));
	g_debugHat = dynamic_cast<DebugHat*>(ObjectManager::Instance().Spawn("DebugHat"));
	g_debugGround = dynamic_cast<DebugGround*>(ObjectManager::Instance().Spawn("DebugGround"));

	if (g_debugPlayer) g_debugPlayer->transform.SetLocalPosition(VGet(-1.5f,1.0f,2.0f));
	if (g_debugEnemy) g_debugEnemy->transform.SetLocalPosition(VGet(1.5f,1.0f,2.0f));

	// グリッド(0)の下に床を置く
	if (g_debugGround) g_debugGround->transform.SetLocalPosition(VGet(0.0f,-0.6f,0.0f));

	// 親子付け（親=Player）
	if (g_debugPlayer && g_debugHat) {
		g_debugHat->transform.SetParent(&g_debugPlayer->transform);
		g_debugHat->transform.SetLocalPosition(VGet(0.0f,1.0f,0.0f));
	}
}

void MenuScene::Update() {
	// ObjectManager 配下の更新
	ObjectManager::Instance().UpdateAll();

	// カメラ操作（デバッグカメラのみ入力）
	g_camCtrl.SetCamera(g_debugCamId);
	g_camCtrl.UpdateFreeMoveMouse(8.0f,0.4f,10.0f);

	// DebugPlayer 移動
	if (g_debugPlayer) {
		VECTOR p = g_debugPlayer->transform.LocalPosition();
		const float spd =0.08f;
		if (CheckHitKey(KEY_INPUT_J)) p.x -= spd;
		if (CheckHitKey(KEY_INPUT_L)) p.x += spd;
		if (CheckHitKey(KEY_INPUT_I)) p.z += spd;
		if (CheckHitKey(KEY_INPUT_K)) p.z -= spd;
		if (CheckHitKey(KEY_INPUT_U)) p.y += spd;
		if (CheckHitKey(KEY_INPUT_O)) p.y -= spd;
		g_debugPlayer->transform.SetLocalPosition(p);
	}

	// カメラ切替入力
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_B)) {    // ブレンドON/OFF切替
		g_useBlend = !g_useBlend;
	}
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_1)) {    // デバッグカメラ
		g_currentCamId = g_debugCamId;
		if (g_useBlend) CameraManager::Instance().BlendRenderTo(g_currentCamId, g_blendSec);
		else CameraManager::Instance().SetRender(g_currentCamId);
	}
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_2)) {    // ゲームカメラ
		g_currentCamId = g_gameCamId;
		if (g_useBlend) CameraManager::Instance().BlendRenderTo(g_currentCamId, g_blendSec);
		else CameraManager::Instance().SetRender(g_currentCamId);
	}

	// シーン切替入力
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_SPACE)) {
		if (g_debugHat) {
			g_debugHat->transform.SetParent(nullptr);
			ObjectManager::Instance().Release(g_debugHat);
			g_debugHat = nullptr;
		}
		if (g_debugGround) { ObjectManager::Instance().Release(g_debugGround); g_debugGround = nullptr; }
		if (g_debugPlayer) { ObjectManager::Instance().Release(g_debugPlayer); g_debugPlayer = nullptr; }
		if (g_debugEnemy) { ObjectManager::Instance().Release(g_debugEnemy); g_debugEnemy = nullptr; }

		SceneTransition::Params p;
		p.mode = SceneTransition::Mode::MaskImage;
		p.durationSec =0.6;
		p.maskGraphPath = "Data/Transition/mask.png";
		p.pixelShaderPath = "Data/Transition/mask_transition.pso";

		SceneTransition::Instance().Start(std::make_unique<TitleScene>(), p,0.5f);
	}
}

void MenuScene::Draw() {
	DrawGridFloor(0.0f, 10, 1.0f);	// グリッド床描画

	ObjectManager::Instance().DrawAll();	// 全オブジェクト描画

	DrawString(10, 10, "MenuScene - Spaceで戻る", GetColor(255, 255, 255));
	DrawString(10, 30, "[操作] J/L:I/K:U/Oで DebugPlayer を移動", GetColor(255, 255, 120));
	DrawString(10, 50, "[カメラ]1:Debug2:Game B:Blend ON/OFF", GetColor(180, 180, 255));
}