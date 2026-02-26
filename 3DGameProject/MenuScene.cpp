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
	CameraController g_camCtrl;						// カメラコントローラ
	CameraController::CameraId g_debugCamId = 0;	// デバッグ用操作カメラ
	CameraController::CameraId g_gameCamId = 0;		// ゲーム用固定カメラ
	CameraController::CameraId g_currentCamId = 0;	// 現在レンダー中カメラ
	bool g_useBlend = true;							// カメラ切替をブレンドするか
	float g_blendSec = 0.4f;						// ブレンド時間

	// プールからスポーンしたデバッグ用オブジェクト
	DebugPlayer* g_debugPlayer = nullptr;	// デバッグ用プレイヤー
	DebugEnemy* g_debugEnemy = nullptr;		// デバッグ用エネミー

	// グリッド床描画
	void DrawGridFloor(float y, int halfCells, float step) {
		const unsigned int colGrid = GetColor(60, 60, 60);
		for (int i = -halfCells; i <= halfCells; ++i) {
			const float x = i * step;
			DrawLine3D(VGet(x, y, -halfCells * step), VGet(x, y, halfCells * step), colGrid);
			const float z = i * step;
			DrawLine3D(VGet(-halfCells * step, y, z), VGet(halfCells * step, y, z), colGrid);
		}
		// axis
		DrawLine3D(VGet(0, y, 0), VGet(2, y, 0), GetColor(255, 80, 80));
		DrawLine3D(VGet(0, y, 0), VGet(0, y + 2, 0), GetColor(80, 255, 80));
		DrawLine3D(VGet(0, y, 0), VGet(0, y, 2), GetColor(80, 80, 255));
	}
}

void MenuScene::Start() {
	auto& camMgr = CameraManager::Instance();						// カメラマネージャ取得
	const int sceneId = SceneManager::Instance().CurrentSceneId();	// 現在シーンID取得

	// 操作カメラ（デバッグ用）
	if (g_debugCamId == 0 || camMgr.Get(g_debugCamId) == nullptr) {
		g_debugCamId = g_camCtrl.SpawnAuto(sceneId, CameraTag::Debug, VGet(0.0f, 2.0f, -8.0f), VGet(0.0f, 0.0f, 0.0f));
	}
	// 固定カメラ（ゲーム用）
	if (g_gameCamId == 0 || camMgr.Get(g_gameCamId) == nullptr) {
		g_camCtrl.SetCamera(0);
		g_gameCamId = g_camCtrl.SpawnAuto(sceneId, CameraTag::Game, VGet(6.0f, 3.0f, -6.0f), VGet(0.0f, 0.8f, 0.0f));
	}

	g_currentCamId = g_debugCamId;
	camMgr.SetRender(g_currentCamId);

	// 登録（最初の一回だけ）
	static bool s_registered = false;
	if (!s_registered) {
		ObjectFactory::Instance().RegisterCreator("DebugPlayer", [](const VariantMap&) { return std::make_unique<DebugPlayer>(); });	// ラムダ式で生成関数を登録
		ObjectFactory::Instance().RegisterCreator("DebugEnemy", [](const VariantMap&) { return std::make_unique<DebugEnemy>(); });		// ラムダ式で生成関数を登録
		ObjectManager::Instance().RegisterPool("DebugPlayer", 8);	// プール登録
		ObjectManager::Instance().RegisterPool("DebugEnemy", 8);	// プール登録
		s_registered = true;
	}

	// デモ用に DebugPlayer / DebugEnemy をスポーン
	g_debugPlayer = dynamic_cast<DebugPlayer*>(ObjectManager::Instance().Spawn("DebugPlayer"));	// pool から取得
	g_debugEnemy = dynamic_cast<DebugEnemy*>(ObjectManager::Instance().Spawn("DebugEnemy"));	// pool から取得
	if (g_debugPlayer) g_debugPlayer->transform.SetLocalPosition(VGet(-1.5f, 1.0f, 2.0f));		// 少し離す
	if (g_debugEnemy) g_debugEnemy->transform.SetLocalPosition(VGet(1.5f, 1.0f, 2.0f));			// 少し離す
}

void MenuScene::Update() {
	// ObjectManager 配下の更新
	ObjectManager::Instance().UpdateAll();

	// カメラ操作（デバッグカメラのみ動かす）
	g_camCtrl.SetCamera(g_debugCamId);
	g_camCtrl.UpdateFreeMoveMouse(8.0f,0.4f,10.0f);

	// DebugPlayer 移動入力
	if (g_debugPlayer) {
		VECTOR p = g_debugPlayer->transform.LocalPosition();	// 現在位置取得
		const float spd = 0.08f;								// 移動速度
		if (CheckHitKey(KEY_INPUT_J)) p.x -= spd;				// 左
		if (CheckHitKey(KEY_INPUT_L)) p.x += spd;				// 右
		if (CheckHitKey(KEY_INPUT_I)) p.z += spd;				// 前
		if (CheckHitKey(KEY_INPUT_K)) p.z -= spd;				// 後
		if (CheckHitKey(KEY_INPUT_U)) p.y += spd;				// 上
		if (CheckHitKey(KEY_INPUT_O)) p.y -= spd;				// 下
		g_debugPlayer->transform.SetLocalPosition(p);			// 位置更新
	}

	// 当たり更新
	ColliderManager::GetInstance().Update();

	// カメラ切替入力
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_B)) {	// ブレンドON/OFF切替
		g_useBlend = !g_useBlend;
	}
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_1)) {	// デバッグカメラ
		g_currentCamId = g_debugCamId;
		if (g_useBlend) CameraManager::Instance().BlendRenderTo(g_currentCamId, g_blendSec);
		else CameraManager::Instance().SetRender(g_currentCamId);
	}
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_2)) {	// ゲームカメラ
		g_currentCamId = g_gameCamId;
		if (g_useBlend) CameraManager::Instance().BlendRenderTo(g_currentCamId, g_blendSec);
		else CameraManager::Instance().SetRender(g_currentCamId);
	}

	// シーン切替入力
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_SPACE)) {
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