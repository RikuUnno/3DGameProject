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

#include "ColliderManager.h"
#include "LayerMask.h"
#include "Debug Class.h"

#include "BoxCollider.h"

#include <memory>
#include <string>

namespace {
	DebugClass::MenuCollisionDebugObjects g_colDbgObjs;

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
			DrawString(10, 40, label_.c_str(), GetColor(200, 255, 200));
		}

	private:
		std::string label_;
	};

	ObjectController g_controller;					// デモ用オブジェクトコントローラ
	int g_demoCreateCount = 0;						// デモ用生成カウンタ

	// カメラデバッグ用
	CameraController g_camCtrl;						// カメラコントローラ
	CameraController::CameraId g_debugCamId = 0;	// デバッグカメラID
	CameraController::CameraId g_gameCamId = 0;		// ゲームカメラID

	// 切替デモ用のフラグ
	bool g_useBlend = true;							// ブレンド切替を使うかどうか
	float g_blendSec = 0.4f;						// ブレンド時間
	CameraController::CameraId g_currentCamId = 0;	// 現在レンダー中のカメラID

	//3Dデバッグ用の簡易描画
	void DrawSimple3DDebug() {
		const int half = 10;
		const float step = 1.0f;
		const unsigned int colGrid = GetColor(60, 60, 60);
		for (int i = -half; i <= half; ++i) {
			const float x = i * step;
			DrawLine3D(VGet(x, 0.0f, -half * step), VGet(x, 0.0f, half * step), colGrid);
			const float z = i * step;
			DrawLine3D(VGet(-half * step, 0.0f, z), VGet(half * step, 0.0f, z), colGrid);
		}

		DrawLine3D(VGet(0, 0, 0), VGet(2, 0, 0), GetColor(255, 80, 80));
		DrawLine3D(VGet(0, 0, 0), VGet(0, 2, 0), GetColor(80, 255, 80));
		DrawLine3D(VGet(0, 0, 0), VGet(0, 0, 2), GetColor(80, 80, 255));
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

	// --- collision debug harness ---
	g_colDbgObjs = {};
	g_colDbgObjs.state.enabled = true;

	// レイヤー確認用：PLAYERがENEMYに当たる設定
	g_colDbgObjs.sphereA = std::make_unique<DebugSphereObject>(0.5f, layerMask::PLAYER, mask::PLAYER);						// プレイヤーレイヤー
	g_colDbgObjs.sphereB = std::make_unique<DebugSphereObject>(0.5f, layerMask::ENEMY, mask::ENEMY);						// エネミーレイヤーレイヤー
	g_colDbgObjs.box = std::make_unique<DebugBoxObject>(VGet(0.8f, 0.8f, 0.8f), layerMask::ENVIRONMENT, mask::ENVIRONMENT);	// 環境レイヤー

	// トリガー確認用
	if (g_colDbgObjs.box && g_colDbgObjs.box->GetCollider()) {
		g_colDbgObjs.box->GetCollider()->isTrigger = true;
	}

	// 位置調整
	g_colDbgObjs.sphereA->transform.SetLocalPosition(VGet(-1.5f, 0.5f, 2.5f));	// 動かすやつ
	g_colDbgObjs.sphereB->transform.SetLocalPosition(VGet(1.5f, 0.5f, 2.5f));	// 動かさないやつ
	g_colDbgObjs.box->transform.SetLocalPosition(VGet(0.0f, 0.8f, 4.5f));		// 動かさないやつ
	g_colDbgObjs.box->transform.SetLocalEulerRad(VGet(0.0f, 0.7f, 0.0f));		// 回転

	auto& camMgr = CameraManager::Instance();						// カメラマネージャ取得
	const int sceneId = SceneManager::Instance().CurrentSceneId();	// シーンID取得

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
	g_camCtrl.UpdateFreeMoveQuat(6.0f, 1.6f);

	// Collision debug object move
	if (g_colDbgObjs.state.enabled && g_colDbgObjs.sphereA) {
		VECTOR p = g_colDbgObjs.sphereA->transform.LocalPosition();
		const float spd = 0.08f;
		if (CheckHitKey(KEY_INPUT_J)) p.x -= spd;
		if (CheckHitKey(KEY_INPUT_L)) p.x += spd;
		if (CheckHitKey(KEY_INPUT_I)) p.z += spd;
		if (CheckHitKey(KEY_INPUT_K)) p.z -= spd;
		if (CheckHitKey(KEY_INPUT_U)) p.y += spd;
		if (CheckHitKey(KEY_INPUT_O)) p.y -= spd;

		g_colDbgObjs.sphereA->transform.SetLocalPosition(p);
	}

	// 当たり更新（MenuSceneで明示的に呼ぶ）
	ColliderManager::GetInstance().Update();

	// 切替モード切替
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_B)) {
		g_useBlend = !g_useBlend;
	}

	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_1)) {
		g_currentCamId = g_debugCamId;
		if (g_useBlend) CameraManager::Instance().BlendRenderTo(g_currentCamId, g_blendSec);
		else CameraManager::Instance().SetRender(g_currentCamId);
	}
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_2)) {
		g_currentCamId = g_gameCamId;
		if (g_useBlend) CameraManager::Instance().BlendRenderTo(g_currentCamId, g_blendSec);
		else CameraManager::Instance().SetRender(g_currentCamId);
	}

	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_SPACE)) {
		if (g_colDbgObjs.sphereA) g_colDbgObjs.sphereA->SetActive(false);
		if (g_colDbgObjs.sphereB) g_colDbgObjs.sphereB->SetActive(false);
		if (g_colDbgObjs.box) g_colDbgObjs.box->SetActive(false);

		g_controller.ReleaseAll();
		SceneManager::Instance().ChangeScene(std::make_unique<TitleScene>());
	}
}

void MenuScene::Draw() {
	DebugClass::DrawSimple3DDebug();

	if (g_colDbgObjs.sphereA) g_colDbgObjs.sphereA->Draw();
	if (g_colDbgObjs.sphereB) g_colDbgObjs.sphereB->Draw();
	if (g_colDbgObjs.box) g_colDbgObjs.box->Draw();

	DrawString(10, 10, "メニューシーン - Spaceで戻る", GetColor(255, 255, 255));
	DrawString(10, 25, "[デモ] ObjectController: Spawn/Update/Draw/ReleaseAll", GetColor(180, 255, 180));
	DrawString(10, 70, "[カメラ]1:Debug2:Game B:Blend ON/OFF", GetColor(180, 180, 255));
	DrawFormatString(10, 90, GetColor(180, 180, 255), "[カメラ] Blend: %s sec=%.2f", g_useBlend ? "ON" : "OFF", g_blendSec);
	DrawString(10, 110, "[操作] 矢印:回転 / WASD+QE:移動（Debugカメラのみ）", GetColor(180, 180, 255));
	DrawFormatString(10, 130, GetColor(180, 180, 255), "[Render] current=%d blending=%s", (int)g_currentCamId, CameraManager::Instance().IsBlending() ? "YES" : "NO");

	// collision debug HUD
	DrawString(10, 160, "[CollisionDebug] J/L:I/K:U/O move sphereA", GetColor(255, 255, 120));
	if (g_colDbgObjs.sphereA) {
		const VECTOR p = g_colDbgObjs.sphereA->transform.WorldPosition();
		DrawFormatString(10, 180, GetColor(255, 255, 120), "sphereA pos=(%.2f,%.2f,%.2f) hit=%s trig=%s", p.x, p.y, p.z,
			g_colDbgObjs.sphereA->IsColliding() ? "YES" : "NO",
			g_colDbgObjs.sphereA->IsTriggering() ? "YES" : "NO");
	}
	if (g_colDbgObjs.sphereB) {
		const VECTOR p = g_colDbgObjs.sphereB->transform.WorldPosition();
		DrawFormatString(10, 200, GetColor(255, 255, 120), "sphereB pos=(%.2f,%.2f,%.2f) hit=%s trig=%s", p.x, p.y, p.z,
			g_colDbgObjs.sphereB->IsColliding() ? "YES" : "NO",
			g_colDbgObjs.sphereB->IsTriggering() ? "YES" : "NO");
	}
	if (g_colDbgObjs.box) {
		const VECTOR p = g_colDbgObjs.box->transform.WorldPosition();
		DrawFormatString(10, 220, GetColor(255, 255, 120), "box pos=(%.2f,%.2f,%.2f) hit=%s trig=%s", p.x, p.y, p.z,
			g_colDbgObjs.box->IsColliding() ? "YES" : "NO",
			g_colDbgObjs.box->IsTriggering() ? "YES" : "NO");
	}

	g_controller.DrawAll();
}