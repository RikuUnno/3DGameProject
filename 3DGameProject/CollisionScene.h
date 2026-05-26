#pragma once

#include <cmath>
#include <memory>
#include <string>

#include "CameraController.h"
#include "CameraManager.h"
#include "CameraTags.h"
#include "CollisionDebugClass.h"
#include "Debug Class.h"
#include "DxLib.h"
#include "KeyInput.h"
#include "ObjectFactory.h"
#include "ObjectManager.h"
#include "SceneManager.h"
#include "SceneTpl.h"
#include "SceneTransition.h"
#include "TitleScene.h"

// CollisionScene - 衝突テストシーン
class CollisionScene : public SceneTpl<CollisionScene> {
public:
	static std::string StaticName() { return "CollisionScene"; }	// シーン名

	// カメラコントローラ
	void Start() override {
		// カメラマネージャーとシーン ID を取得
		auto& camMgr = CameraManager::Instance();
		const int sceneId = SceneManager::Instance().CurrentSceneId();

		// デモ用オブジェクトのプール登録と生成
		RegisterPools_();
		SpawnDemoObjects_();
		CollisionDebugClass::ResetEventText();

		// カメラ生成（既に生成されている場合は再利用）
		if (_debugCamId == 0 || camMgr.Get(_debugCamId) == nullptr) {
			_debugCamId = _debugCamCtrl.SpawnAuto(sceneId, CameraTag::Debug, VGet(0.0f, 5.5f, -14.0f), VGet(0.10f, 0.0f, 0.0f));
		}
		if (_gameCamId == 0 || camMgr.Get(_gameCamId) == nullptr) {
			_gameCamId = _gameCamCtrl.SpawnAuto(sceneId, CameraTag::Game, VGet(6.0f, 4.0f, -8.0f), VGet(0.0f, 0.0f, 0.0f));
		}

		// デバッグカメラをレンダリング対象に設定
		_currentCamId = _debugCamId;
		camMgr.SetRender(_currentCamId);
	}

	// シーン終了前のクリーンアップ
	void End() override {
		ReleaseDemoObjects_();	// デモ用オブジェクトの解放
	}

	// デモ用オブジェクトの解放
	void Update(float dtSec) override {
		// 全オブジェクトの更新処理を呼び出す（衝突判定やその他のロジックを更新）
		ObjectManager::Instance().UpdateAll(dtSec);

		// デバッグカメラの操作更新
		_debugCamCtrl.SetCamera(_debugCamId);							// デバッグカメラを操作対象に設定
		_debugCamCtrl.UpdateFreeMoveMouse(8.0f, 0.4f, 10.0f, dtSec);	// デバッグカメラの更新処理を呼び出す（右クリック＋WASDQEでのフリームーブ操作を処理）

		UpdateControlledPlayer_(dtSec);	// プレイヤーの操作更新
		UpdateGameCamera_();			// ゲームカメラの位置をプレイヤーに追従させる

		// カメラ切替とシーン遷移の入力処理
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_B)) {	// ブレンド切替
			_useBlend = !_useBlend;
		}
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_1)) {	// デバッグカメラに切替
			_currentCamId = _debugCamId;
			if (_useBlend) CameraManager::Instance().BlendRenderTo(_currentCamId, _blendSec);
			else CameraManager::Instance().SetRender(_currentCamId);
		}
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_2)) {	// ゲームカメラに切替
			_currentCamId = _gameCamId;
			if (_useBlend) CameraManager::Instance().BlendRenderTo(_currentCamId, _blendSec);
			else CameraManager::Instance().SetRender(_currentCamId);
		}
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_R)) {	// シーンリセット
			ReleaseDemoObjects_();
			SceneManager::Instance().RequestChange(std::make_unique<CollisionScene>());
		}
		if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_T)) {	// タイトルへ遷移
			ReleaseDemoObjects_();
			SceneTransition::Params p;
			p.mode = SceneTransition::Mode::MaskImage;
			p.durationSec = 0.4f;
			p.maskGraphPath = "Data/Transition/mask.png";
			p.pixelShaderPath = "Data/Transition/mask_transition.pso";
			SceneTransition::Instance().Start(std::make_unique<TitleScene>(), p, 0.5f);
		}
	}

	// 描画処理
	void Draw() override {
		DrawGridFloor_(0.0f, 14, 1.0f);
		ObjectManager::Instance().DrawAll();

		DrawString(10, 10, "CollisionScene - R:リセット T:タイトル", GetColor(255, 255, 255));
		DrawString(10, 30, "[操作] J/L:I/K:U/O でプレイヤー移動", GetColor(255, 255, 120));
		DrawString(10, 50, "       既存MenuSceneベースで衝突応答を確認", GetColor(255, 220, 120));
		DrawString(10, 70, "[カメラ] 1:Debug 2:Game  B:Blend ON/OFF", GetColor(180, 180, 255));
		DrawString(10, 90, "赤系: Collision  青系: Trigger", GetColor(255, 180, 180));
		DrawString(10, 110, CollisionDebugClass::LastEventText().c_str(), GetColor(180, 255, 180));	// 最後の衝突イベントを表示
		DrawString(10, 130, "Box / Sphere / Capsule / Trigger を配置", GetColor(220, 220, 220));
	}

private:
	// デモ用オブジェクトのプール登録
	void RegisterPools_() {
		static bool s_registered = false;	// 登録は一度だけで十分
		if (s_registered) return;			// すでに登録されていれば何もしない

		// Factory に生成関数を登録
		ObjectFactory::Instance().RegisterCreator("DebugPlayer", [](const VariantMap&) { return std::make_unique<DebugPlayer>(); });									// "DebugPlayer" というキーで DebugPlayer オブジェクトを生成する関数を登録
		ObjectFactory::Instance().RegisterCreator("DebugEnemy", [](const VariantMap&) { return std::make_unique<DebugEnemy>(); });										// "DebugEnemy" というキーで DebugEnemy オブジェクトを生成する関数を登録
		ObjectFactory::Instance().RegisterCreator("DebugHat", [](const VariantMap&) { return std::make_unique<DebugHat>(); });											// "DebugHat" というキーで DebugHat オブジェクトを生成する関数を登録
		ObjectFactory::Instance().RegisterCreator("DebugGround", [](const VariantMap&) { return std::make_unique<DebugGround>(); });									// "DebugGround" というキーで DebugGround オブジェクトを生成する関数を登録
		ObjectFactory::Instance().RegisterCreator(CollisionDebugClass::StaticPoolKey(), [](const VariantMap&) { return std::make_unique<CollisionDebugClass>(); });		// CollisionDebugClass::StaticPoolKey() というキーで CollisionDebugClass オブジェクトを生成する関数を登録

		ObjectManager::Instance().RegisterPool("DebugPlayer", 4);	// "DebugPlayer" というキーでオブジェクトプールを登録（最大4つのオブジェクトをプールできるように設定）
		ObjectManager::Instance().RegisterPool("DebugEnemy", 4);	// "DebugEnemy" というキーでオブジェクトプールを登録（最大4つのオブジェクトをプールできるように設定）
		ObjectManager::Instance().RegisterPool("DebugHat", 4);		// "DebugHat" というキーでオブジェクトプールを登録（最大4つのオブジェクトをプールできるように設定）
		ObjectManager::Instance().RegisterPool("DebugGround", 2);	// "DebugGround" というキーでオブジェクトプールを登録（最大2つのオブジェクトをプールできるように設定）
		ObjectManager::Instance().RegisterPool(CollisionDebugClass::StaticPoolKey(), 16);	// CollisionDebugClass::StaticPoolKey() というキーでオブジェクトプールを登録（最大16つのオブジェクトをプールできるように設定）
		s_registered = true;
	}

	// デモ用オブジェクトの生成
	void SpawnDemoObjects_() {
		_player = dynamic_cast<DebugPlayer*>(ObjectManager::Instance().Spawn("DebugPlayer"));	// "DebugPlayer" というキーでオブジェクトを生成し、_player ポインタにキャストして保存
		_enemy = dynamic_cast<DebugEnemy*>(ObjectManager::Instance().Spawn("DebugEnemy"));		// "DebugEnemy" というキーでオブジェクトを生成し、_enemy ポインタにキャストして保存
		_hat = dynamic_cast<DebugHat*>(ObjectManager::Instance().Spawn("DebugHat"));			// "DebugHat" というキーでオブジェクトを生成し、_hat ポインタにキャストして保存
		_ground = dynamic_cast<DebugGround*>(ObjectManager::Instance().Spawn("DebugGround"));	// "DebugGround" というキーでオブジェクトを生成し、_ground ポインタにキャストして保存

		// 衝突オブジェクトの生成（SpawnCollisionObject_ はこのシーン内で定義されたユーティリティ関数で、VariantMap を受け取って衝突オブジェクトを生成する）
		_boxObstacle = SpawnCollisionObject_({	// ボックス型の障害物を生成
			{"name", "SolidBox"},
			{"shape", "box"},
			{"static", "true"},
			{"px", "0.0"}, {"py", "1.0"}, {"pz", "5.0"},
			{"hx", "0.9"}, {"hy", "0.9"}, {"hz", "0.9"},
			{"color", std::to_string(GetColor(220, 220, 220))}
		});
		_sphereObstacle = SpawnCollisionObject_({	// 球型の障害物を生成
			{"name", "SolidSphere"},
			{"shape", "sphere"},
			{"static", "true"},
			{"px", "-4.5"}, {"py", "1.0"}, {"pz", "4.5"},
			{"radius", "0.8"},
			{"color", std::to_string(GetColor(255, 220, 120))}
		});
		_capsuleObstacle = SpawnCollisionObject_({	// カプセル型の障害物を生成
			{"name", "SolidCapsule"},
			{"shape", "capsule"},
			{"static", "true"},
			{"px", "4.5"}, {"py", "1.2"}, {"pz", "4.0"},
			{"radius", "0.45"}, {"halfHeight", "0.95"},
			{"yaw", "0.35"},
			{"color", std::to_string(GetColor(180, 255, 180))}
		});
		_triggerSphere = SpawnCollisionObject_({	// トリガー用の球型オブジェクトを生成
			{"name", "TriggerSphere"},
			{"shape", "sphere"},
			{"trigger", "true"},
			{"static", "true"},
			{"layer", std::to_string(layerMask::TRIGGER)},
			{"px", "0.0"}, {"py", "1.2"}, {"pz", "-3.5"},
			{"radius", "1.4"},
			{"color", std::to_string(GetColor(120, 140, 255))},
			{"triggerColor", std::to_string(GetColor(80, 180, 255))}
		});

		// 位置や親子関係の設定
		if (_player) {	// プレイヤーオブジェクトの初期位置を設定
			_player->transform.SetLocalPosition(VGet(0.0f, 1.0f, 0.0f));
			if (PhysicsBody* body = _player->GetPhysicsBody()) {
				body->_enabled = false;
				body->_velocity = VGet(0.0f, 0.0f, 0.0f);
			}
		}
		if (_enemy) {	// 敵オブジェクトの初期位置を設定
			_enemy->transform.SetLocalPosition(VGet(3.5f, 1.0f, -1.5f));
			_enemy->isStatic = true;
		}
		if (_ground) {	// 地面オブジェクトの初期位置を設定
			_ground->transform.SetLocalPosition(VGet(0.0f, -0.6f, 0.0f));
		}
		if (_hat && _player) {	// 帽子オブジェクトをプレイヤーの子オブジェクトとして設定し、相対位置を調整
			_hat->transform.SetParent(&_player->transform);
			_hat->transform.SetLocalPosition(VGet(0.35f, 1.0f, 0.35f));
		}
	}

	// デモ用オブジェクトの解放
	void ReleaseDemoObjects_() {
		// 生成したオブジェクトを ObjectManager を通じて解放し、ポインタを nullptr にリセット
		if (_hat) {
			_hat->transform.SetParent(nullptr);
			ObjectManager::Instance().Release(_hat);
			_hat = nullptr;
		}
		// それぞれのオブジェクトが nullptr でない場合は ObjectManager を通じて解放し、ポインタを nullptr にリセット
		if (_ground) { ObjectManager::Instance().Release(_ground); _ground = nullptr; }
		if (_player) { ObjectManager::Instance().Release(_player); _player = nullptr; }
		if (_enemy) { ObjectManager::Instance().Release(_enemy); _enemy = nullptr; }
		if (_boxObstacle) { ObjectManager::Instance().Release(_boxObstacle); _boxObstacle = nullptr; }
		if (_sphereObstacle) { ObjectManager::Instance().Release(_sphereObstacle); _sphereObstacle = nullptr; }
		if (_capsuleObstacle) { ObjectManager::Instance().Release(_capsuleObstacle); _capsuleObstacle = nullptr; }
		if (_triggerSphere) { ObjectManager::Instance().Release(_triggerSphere); _triggerSphere = nullptr; }
	}

	// プレイヤーの操作更新
	void UpdateControlledPlayer_(float dtSec) {
		if (!_player) return;

		// 入力に基づいてプレイヤーの移動を更新する。J/L で左右、I/K で前後、U/O で上下の移動を制御。
		const float moveSpeed = 4.8f;		// 水平方向の移動速度を定義
		const float verticalSpeed = 4.0f;	// 垂直方向の移動速度を定義
		VECTOR input = VGet(0.0f, 0.0f, 0.0f);
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_J)) input.x -= 1.0f;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_L)) input.x += 1.0f;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_I)) input.z += 1.0f;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_K)) input.z -= 1.0f;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_U)) input.y += 1.0f;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_O)) input.y -= 1.0f;

		// 水平方向の入力ベクトルを計算し、その長さの二乗を求める。これにより、入力がほとんどない場合の処理を簡略化できる。
		const VECTOR horizontalInput = VGet(input.x, 0.0f, input.z);													// 水平方向の入力ベクトルを作成（y成分は0に固定）
		const float horizontalLenSq = horizontalInput.x * horizontalInput.x + horizontalInput.z * horizontalInput.z;	// 水平方向の入力ベクトルの長さの二乗を計算

		// プレイヤーの新しい位置を計算する。水平方向の入力がある場合は、正規化して移動速度を掛ける。垂直方向の入力も同様に処理する。
		VECTOR newPosition = _player->transform.LocalPosition();
		if (horizontalLenSq > 1e-6f) {	// 水平方向の入力がほとんどない場合は移動しない（ゼロ除算を防止）
			const float invLen = 1.0f / std::sqrt(horizontalLenSq);
			newPosition.x += horizontalInput.x * invLen * moveSpeed * dtSec;
			newPosition.z += horizontalInput.z * invLen * moveSpeed * dtSec;

			const float yaw = std::atan2(horizontalInput.x, horizontalInput.z);
			_player->transform.SetLocalEulerRad(VGet(0.0f, yaw, 0.0f));
		}
		if (input.y != 0.0f) {			// 垂直方向の入力がある場合は移動する
			newPosition.y += input.y * verticalSpeed * dtSec;
		}
		_player->transform.SetLocalPosition(newPosition);
	}

	// ゲームカメラの位置をプレイヤーに追従させる
	void UpdateGameCamera_() {
		auto* cam = CameraManager::Instance().Get(_gameCamId);		// ゲームカメラを CameraManager から取得
		if (!cam || !_player) return;								// カメラやプレイヤーが存在しない場合は何もしない
		const VECTOR target = _player->transform.LocalPosition();	// プレイヤーの位置をターゲットとして取得
		cam->LookAt(VAdd(target, VGet(6.0f, 4.0f, -8.0f)), VAdd(target, VGet(0.0f, 1.0f, 0.0f)));	// ゲームカメラの位置をプレイヤーの位置からオフセットした位置に設定し、プレイヤーを見つめるようにする
	}

	// 衝突オブジェクトの生成ユーティリティ関数
	static CollisionDebugClass* SpawnCollisionObject_(const VariantMap& params) {
		return dynamic_cast<CollisionDebugClass*>(ObjectManager::Instance().Spawn(CollisionDebugClass::StaticPoolKey(), params));	// CollisionDebugClass::StaticPoolKey() というキーでオブジェクトを生成し、CollisionDebugClass 型にキャストして返す
	}

	// グリッド床の描画ユーティリティ関数
	static void DrawGridFloor_(float y, int halfCells, float step) {
		const unsigned int colGrid = GetColor(60, 60, 60);
		for (int i = -halfCells; i <= halfCells; ++i) {
			const float x = i * step;
			DrawLine3D(VGet(x, y, -halfCells * step), VGet(x, y, halfCells * step), colGrid);
			const float z = i * step;
			DrawLine3D(VGet(-halfCells * step, y, z), VGet(halfCells * step, y, z), colGrid);
		}
		DrawLine3D(VGet(0, y, 0), VGet(2, y, 0), GetColor(255, 80, 80));
		DrawLine3D(VGet(0, y, 0), VGet(0, y + 2, 0), GetColor(80, 255, 80));
		DrawLine3D(VGet(0, y, 0), VGet(0, y, 2), GetColor(80, 80, 255));
	}

private:
	// カメラコントローラとカメラIDの管理
	CameraController _debugCamCtrl;					// デバッグカメラのコントローラを管理するためのメンバ変数
	CameraController _gameCamCtrl;					// ゲームカメラのコントローラを管理するためのメンバ変数
	CameraController::CameraId _debugCamId = 0;		// デバッグカメラのIDを管理するためのメンバ変数（初期値は0で、生成されたカメラのIDが格納される）
	CameraController::CameraId _gameCamId = 0;		// ゲームカメラのIDを管理するためのメンバ変数（初期値は0で、生成されたカメラのIDが格納される）
	CameraController::CameraId _currentCamId = 0;	// 現在レンダリング対象のカメラIDを管理するためのメンバ変数（初期値は0で、Start() 内でデバッグカメラのIDが設定される）
	bool _useBlend = true;							// カメラ切替時にブレンドを使用するかどうかを管理するためのフラグ（初期値はtrueで、Bキーで切り替え可能）
	float _blendSec = 0.4f;							// カメラ切替時のブレンド時間を管理するためのメンバ変数（初期値は0.4秒）

	// デモ用オブジェクトのポインタ（生成されたオブジェクトを管理するためのメンバ変数）
	DebugPlayer* _player = nullptr;						// プレイヤーオブジェクトのポインタ
	DebugEnemy* _enemy = nullptr;						// 敵オブジェクトのポインタ
	DebugHat* _hat = nullptr;							// 帽子オブジェクトのポインタ
	DebugGround* _ground = nullptr;						// 地面オブジェクトのポインタ
	CollisionDebugClass* _boxObstacle = nullptr;		// ボックス型障害物のポインタ
	CollisionDebugClass* _sphereObstacle = nullptr;		// 球型障害物のポインタ
	CollisionDebugClass* _capsuleObstacle = nullptr;	// カプセル型障害物のポインタ
	CollisionDebugClass* _triggerSphere = nullptr;		// トリガー用の球型オブジェクトのポインタ
};