#include "MenuScene.h"
#include "TitleScene.h"
#include "PhysicsScene.h"
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
		// 真上だと回転追従が分かりにくいので、少し前方右寄りにずらす
		g_debugHat->transform.SetLocalPosition(VGet(0.35f,1.0f,0.35f));
	}
}

void MenuScene::Update(float dtSec) {
	// ObjectManager 配下の更新
	ObjectManager::Instance().UpdateAll(dtSec);

	// カメラ操作（デバッグカメラのみ入力）
	g_camCtrl.SetCamera(g_debugCamId);
	g_camCtrl.UpdateFreeMoveMouse(8.0f,0.4f,10.0f, dtSec);

	// DebugPlayer 移動
	if (g_debugPlayer) {
		PhysicsBody* body = g_debugPlayer->GetPhysicsBody();
		if (body) {
			const float moveSpeed =4.8f;
			const float verticalSpeed =4.0f;
			VECTOR input = VGet(0.0f,0.0f,0.0f);
			if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_J)) input.x -= 1.0f;
			if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_L)) input.x += 1.0f;
			if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_I)) input.z += 1.0f;
			if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_K)) input.z -= 1.0f;
			if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_U)) input.y += 1.0f;
			if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_O)) input.y -= 1.0f;

			const VECTOR horizontalInput = VGet(input.x,0.0f,input.z);
			const float horizontalLenSq = horizontalInput.x * horizontalInput.x + horizontalInput.z * horizontalInput.z;

			VECTOR newVelocity = body->_velocity;
			newVelocity.x = 0.0f;
			newVelocity.z = 0.0f;
			if (horizontalLenSq > 1e-6f) {
				const float invLen = 1.0f / std::sqrt(horizontalLenSq);
				newVelocity.x = horizontalInput.x * invLen * moveSpeed;
				newVelocity.z = horizontalInput.z * invLen * moveSpeed;

				// 水平方向の入力がある時は進行方向を向かせる。
				// 子オブジェクト(DebugHat)が親の回転に追従しているか確認しやすくするため。
				const float yaw = std::atan2(horizontalInput.x, horizontalInput.z);
				g_debugPlayer->transform.SetLocalEulerRad(VGet(0.0f, yaw, 0.0f));
			}

			// y は入力がある時だけ速度を与える。
			// 入力が無い時に 0 を入れると、重力で増えた落下速度まで打ち消してしまう。
			if (input.y != 0.0f) {
				newVelocity.y = input.y * verticalSpeed;
			}
			body->_velocity = newVelocity;
		}
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
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_P)) {
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
		p.durationSec = 0.6;
		p.maskGraphPath = "Data/Transition/mask.png";
		p.pixelShaderPath = "Data/Transition/mask_transition.pso";

		SceneTransition::Instance().Start(std::make_unique<PhysicsScene>(), p, 0.5f);
	}
}

void MenuScene::Draw() {
	DrawGridFloor(0.0f, 10, 1.0f);	// グリッド床描画

	ObjectManager::Instance().DrawAll();	// 全オブジェクト描画

	DrawString(10, 10, "MenuScene - Spaceで戻る", GetColor(255, 255, 255));
	DrawString(10, 30, "[操作] J/L:I/K:U/Oで DebugPlayer を移動", GetColor(255, 255, 120));
	DrawString(10, 50, "       入力は PhysicsBody の速度へ反映", GetColor(255, 220, 120));
	DrawString(10, 70, "       斜め移動は正規化 / 帽子は斜め前に配置", GetColor(255, 220, 120));
	DrawString(10, 90, "       水平移動時は進行方向へ自動回転", GetColor(255, 220, 120));
	DrawString(10, 110, "[カメラ]1:Debug2:Game B:Blend ON/OFF", GetColor(180, 180, 255));
	DrawString(10, 130, "P : Physics Scene", GetColor(180, 255, 180));
}