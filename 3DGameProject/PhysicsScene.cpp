//  PhysicsScene - 物理デバッグシーン
// 
//  物理エンジンのテスト用シーン。様々な形状・マテリアルの組み合わせで
//  衝突、摩擦、反発、CCD（連続衝突検出）等をテストできる。
//
//  【操作】
//  R: リセット / T: タイトル / 右クリック+WASDQE: カメラ移動
//  1: 木ボックス投下 / 2: 金属球投下 / 3: ゴムカプセル投下
//  F: 高速弾発射（CCDテスト）

#include "PhysicsScene.h"

#include <memory>
#include <string>
#include <deque>

#include "CameraController.h"
#include "CameraManager.h"
#include "CameraTags.h"
#include "DxLib.h"
#include "KeyInput.h"
#include "TitleScene.h"
#include "ObjectFactory.h"
#include "ObjectManager.h"
#include "PhysicsDebugClass.h"
#include "PhysicsMaterial.h"
#include "PhysicsManager.h"
#include "SceneManager.h"
#include "SceneTransition.h"

namespace {
    // カメラコントローラ
    CameraController _cameraController;
    CameraController::CameraId _cameraId = 0;
    bool _registered = false;  // ファクトリ登録済みフラグ
    bool _isSpawningArena = false;  // アリーナ生成中はオブジェクト追跡しない

	// 編集対象の斜面
	PhysicsDebugClass* _rampLeftWall = nullptr;   // キー8
	PhysicsDebugClass* _rampCorner = nullptr;     // キー9
	PhysicsDebugClass* _rampBackWall = nullptr;   // キー0
	enum class RampSelection { Left, Corner, Back };
	RampSelection _selectedRamp = RampSelection::Left;

	PhysicsDebugClass* GetSelectedRamp_() {
		switch (_selectedRamp) {
		case RampSelection::Left: return _rampLeftWall;
		case RampSelection::Corner: return _rampCorner;
		case RampSelection::Back: return _rampBackWall;
		default: return nullptr;
		}
	}

	// 動的オブジェクト数制限（古い順に削除）
	constexpr size_t _maxDynamicBoxCount = 30;         // ボックスはやや多めに許可（スタッキングテスト用）
	constexpr size_t _maxDynamicSphereCount = 40;      // 球は多めに許可（転がりテスト用）
    constexpr size_t _maxDynamicCapsuleCount = 20;    // カプセルは少なめに許可（転倒テスト用）

    // プレイヤー生成オブジェクトの追跡キュー
	std::deque<PhysicsDebugClass*> _dynamicBoxes;       // 動的ボックスの追跡キュー
	std::deque<PhysicsDebugClass*> _dynamicSpheres;     // 動的球の追跡キュー
	std::deque<PhysicsDebugClass*> _dynamicCapsules;    // 動的カプセルの追跡キュー

    // 古いオブジェクトの削除（上限管理）
    void ReleaseOldestIfNeeded_(std::deque<PhysicsDebugClass*>& objects, size_t maxCount) {
        while (objects.size() >= maxCount && !objects.empty()) {
            PhysicsDebugClass* oldest = objects.front();
            objects.pop_front();
            if (!oldest) continue;
            ObjectManager::Instance().Release(oldest);
        }
    }

	// 新しいオブジェクトの登録（上限管理）
    void RegisterDynamicObject_(PhysicsDebugClass* obj, std::deque<PhysicsDebugClass*>& objects, size_t maxCount) {
        if (!obj) return;
        if (_isSpawningArena) return;
        ReleaseOldestIfNeeded_(objects, maxCount);
        objects.push_back(obj);
    }

	// 動的オブジェクトの一括削除（シーンリセット用）
    void ClearDynamicTracking_() {
        _dynamicBoxes.clear();
        _dynamicSpheres.clear();
        _dynamicCapsules.clear();
    }

    // オブジェクト生成ヘルパー（ObjectManagerから生成）
    PhysicsDebugClass* SpawnPhysicsObject(const std::string& key, const VariantMap& params) {
        return dynamic_cast<PhysicsDebugClass*>(ObjectManager::Instance().Spawn(key, params));
    }

	// Box生成（自動的にCCD有効化）
	// パラメータ: px/py/pz(位置), hx/hy/hz(ハーフサイズ), material(材質), static(静的フラグ)
	PhysicsDebugClass* SpawnPhysicsBox(const VariantMap& params) {
		auto* obj = SpawnPhysicsObject(PhysicsDebugBox::StaticPoolKey(), params);   // ObjectManager から生成
		RegisterDynamicObject_(obj, _dynamicBoxes, _maxDynamicBoxCount);            // 追跡登録（上限管理）
		return obj;
	}

	// 球生成（自動的にCCD有効化）
	// パラメータ: px/py/pz(位置), radius(半径), material(材質)
	PhysicsDebugClass* SpawnPhysicsSphere(const VariantMap& params) {
		auto* obj = SpawnPhysicsObject(PhysicsDebugSphere::StaticPoolKey(), params);    // ObjectManager から生成
		RegisterDynamicObject_(obj, _dynamicSpheres, _maxDynamicSphereCount);           // 追跡登録（上限管理）
		return obj;
	}

	// カプセル生成（自動的にCCD有効化）
	// パラメータ: px/py/pz(位置), radius(半径), halfHeight(半分の高さ), material(材質)
	PhysicsDebugClass* SpawnPhysicsCapsule(const VariantMap& params) {
		auto* obj = SpawnPhysicsObject(PhysicsDebugCapsule::StaticPoolKey(), params);
		RegisterDynamicObject_(obj, _dynamicCapsules, _maxDynamicCapsuleCount);
		return obj;
	}

    //  アリーナ生成（横長床 + 左壁 + 奥壁 + 斜めスロープ）
    void SpawnArena() {
		_isSpawningArena = true;

		const unsigned int colFloor   = GetColor(170, 130, 85);
		const unsigned int colWall    = GetColor(160, 140, 120);
		const unsigned int colRamp    = GetColor(170, 175, 185);
		const unsigned int colWood    = GetColor(190, 150, 90);
		const unsigned int colMetal   = GetColor(180, 185, 195);
		const unsigned int colCapsule = GetColor(220, 120, 120);

		// 地面はウッド（横長）
		SpawnPhysicsBox({
			{"static", "true"},
			{"px", "0"}, {"py", "-1.0"}, {"pz", "0"},
			{"hx", "15.0"}, {"hy", "1.0"}, {"hz", "9.0"},
			{"material", "wood"},
			{"color", std::to_string(colFloor)}
		});

		// 左壁
		SpawnPhysicsBox({
			{"static", "true"},
			{"px", "-15.5"}, {"py", "3.0"}, {"pz", "0.0"},
			{"hx", "0.5"}, {"hy", "4.0"}, {"hz", "9.0"},
			{"material", "wood"},
			{"color", std::to_string(colWall)}
		});

		// 右壁
		SpawnPhysicsBox({
			{"static", "true"},
			{"px", "15.5"}, {"py", "3.0"}, {"pz", "0.0"},
			{"hx", "0.5"}, {"hy", "4.0"}, {"hz", "9.0"},
			{"material", "wood"},
			{"color", std::to_string(colWall)}
		});

		// 奥壁
		SpawnPhysicsBox({
			{"static", "true"},
			{"px", "0.0"}, {"py", "3.0"}, {"pz", "9.5"},
			{"hx", "15.0"}, {"hy", "4.0"}, {"hz", "0.5"},
			{"material", "wood"},
			{"color", std::to_string(colWall)}
		});

		// 手前壁
		SpawnPhysicsBox({
			{"static", "true"},
			{"px", "0.0"}, {"py", "3.0"}, {"pz", "-9.5"},
			{"hx", "15.0"}, {"hy", "4.0"}, {"hz", "0.5"},
			{"material", "wood"},
			{"color", std::to_string(colWall)}
		});

		// 左上コーナーに「3枚の斜面」構成（赤・黄・青）
		// 赤: 左手前から立ち上がる斜面
		auto* rampRed = SpawnPhysicsBox({
			{"static", "true"},
			{"px", "-12.8"}, {"py", "1.0"}, {"pz", "-1.0"},
			{"hx", "3.8"}, {"hy", "0.22"}, {"hz", "4.0"},
			{"material", "metal"},
			{"color", std::to_string(GetColor(230, 90, 90))}
		});
		_rampLeftWall = rampRed;
		if (rampRed) {
			// 左: x0 y0 z-20 (deg)
			rampRed->transform.SetLocalEulerRad(VGet(
				DX_PI_F * (0.0f / 180.0f),
				DX_PI_F * (0.0f / 180.0f),
				-DX_PI_F * (20.0f / 180.0f)
			));
		}

		// 黄: 中央接続の斜面
		auto* rampYellow = SpawnPhysicsBox({
			{"static", "true"},
			{"px", "-9.6"}, {"py", "1.0"}, {"pz", "3.2"},
			{"hx", "3.0"}, {"hy", "0.22"}, {"hz", "4.2"},
			{"material", "metal"},
			{"color", std::to_string(GetColor(235, 225, 90))}
		});
		_rampCorner = rampYellow;
		if (rampYellow) {
			// 真ん中: x-25 y35 z-35 (deg)
			rampYellow->transform.SetLocalEulerRad(VGet(
				-DX_PI_F * (25.0f / 180.0f),
				 DX_PI_F * (35.0f / 180.0f),
				-DX_PI_F * (35.0f / 180.0f)
			));
		}

		// 青: 奥壁寄りの受け側斜面
		auto* rampBlue = SpawnPhysicsBox({
			{"static", "true"},
			{"px", "-4.2"}, {"py", "1.0"}, {"pz", "5.1"},
			{"hx", "7.8"}, {"hy", "0.22"}, {"hz", "3.6"},
			{"material", "metal"},
			{"color", std::to_string(GetColor(90, 140, 235))}
		});
		_rampBackWall = rampBlue;
		if (rampBlue) {
			// 奥: x-20 y0 z0 (deg)
			rampBlue->transform.SetLocalEulerRad(VGet(
				-DX_PI_F * (20.0f / 180.0f),
				 DX_PI_F * (0.0f / 180.0f),
				 DX_PI_F * (0.0f / 180.0f)
			));
		}

		// 球オブジェクト（右側に縦配置）
		for (int i = 0; i < 3; ++i) {
			SpawnPhysicsSphere({
				{"px", "8.0"}, {"py", std::to_string(1.2f + i * 1.0f)}, {"pz", std::to_string(2.0f + i * 0.2f)},
				{"radius", "0.45"},
				{"material", "metal"},
				{"ccd", "true"},
				{"ccdThreshold", "2.0"},
				{"color", std::to_string(colMetal)}
			});
		}

		// Boxオブジェクト（中央下付近）
		for (int i = 0; i < 2; ++i) {
			SpawnPhysicsBox({
				{"px", std::to_string(-0.8f + i * 1.2f)}, {"py", "0.6"}, {"pz", "-3.0"},
				{"hx", "0.6"}, {"hy", "0.45"}, {"hz", "0.45"},
				{"material", "wood"},
				{"ccd", "true"},
				{"ccdThreshold", "2.0"},
				{"color", std::to_string(colWood)}
			});
		}

		// 初期配置を増やす: 追加ボックススタック
		for (int row = 0; row < 3; ++row) {
			SpawnPhysicsBox({
				{"px", "3.0"}, {"py", std::to_string(0.6f + row * 0.95f)}, {"pz", "-1.5"},
				{"hx", "0.5"}, {"hy", "0.45"}, {"hz", "0.45"},
				{"material", "wood"},
				{"ccd", "true"},
				{"ccdThreshold", "2.0"},
				{"color", std::to_string(colWood)}
			});
		}

		// カプセルオブジェクト（左下付近）
		for (int i = 0; i < 2; ++i) {
			SpawnPhysicsCapsule({
				{"px", std::to_string(-8.0f + i * 1.8f)}, {"py", "0.9"}, {"pz", "-4.5"},
				{"radius", "0.35"},
				{"halfHeight", "0.65"},
				{"material", "rubber"},
				{"ccd", "true"},
				{"ccdThreshold", "2.0"},
				{"color", std::to_string(colCapsule)}
			});
		}

		_isSpawningArena = false;
    }

    // カメラ前方にオブジェクトを落とす（type: 1=Box, 2=Sphere, 3=Capsule）
	void SpawnDropObject(int type) {    // type: 1=Box, 2=Sphere, 3=Capsule
		auto* cam = CameraManager::Instance().Get(_cameraId);   // カメラ取得
        if (!cam) return;

		const VECTOR forward = cam->transform.Forward();            // カメラの前方ベクトルを取得
		const VECTOR eye = cam->transform.LocalPosition();          // カメラの位置を取得
		const VECTOR spawnPos = VAdd(eye, VScale(forward, 3.0f));   // 位置（カメラの前方3mの位置にスポーン）
		VariantMap params;                          // パラメータマップを作成
		params["px"] = std::to_string(spawnPos.x);  // 位置（カメラの前方3mの位置）
		params["py"] = std::to_string(spawnPos.y);  // 位置（カメラの前方3mの位置）
		params["pz"] = std::to_string(spawnPos.z);  // 位置（カメラの前方3mの位置）
		params["ccd"] = "true";                     // CCD有効化
		params["ccdThreshold"] = "2.0";             // CCD閾値設定（小さな値で厳密なCCDを有効化）

		PhysicsDebugClass* obj = nullptr;           // 生成するオブジェクトのポインタ
		if (type == 1) {                            // Box
			params["hx"] = "0.5";                   // ハーフサイズ（0.5mの立方体）
			params["hy"] = "0.5";                   // ハーフサイズ（0.5mの立方体）
			params["hz"] = "0.5";                   // ハーフサイズ（0.5mの立方体）
			params["material"] = "wood";            // 材質(wood)
			obj = SpawnPhysicsBox(params);          // Box生成
        }
		else if (type == 2) {                       // Sphere
			params["radius"] = "0.45";              // 半径（0.45mの球）
			params["material"] = "metal";           // 材質(metal)
			obj = SpawnPhysicsSphere(params);       // 球生成
        }
		else {                                      // Capsule
			params["radius"] = "0.38";              // 半径（0.38mのカプセル）
			params["halfHeight"] = "0.75";          // 半分の高さ（0.75mのカプセル）
			params["material"] = "rubber";          // 材質(rubber(ゴムのような材質))
			obj = SpawnPhysicsCapsule(params);      // カプセル生成
        }

		// 生成後すぐに前方に軽く押し出す（衝突テスト用）
        if (obj) {
			obj->GetPhysicsBody()->AddVelocityChange(VScale(forward, 3.0f));    // 軽く前方に押し出す（3m/s程度）
        }
    }

	// 高速弾発射（CCDテスト用、40m/s）
	void FireProjectile() {
		auto* cam = CameraManager::Instance().Get(_cameraId);   // カメラ取得
		if (!cam) return;                                       // カメラの前方に高速で飛ぶ金属球を生成して発射する（CCDテスト用）

		const VECTOR forward = cam->transform.Forward();            // カメラの前方ベクトルを取得
		const VECTOR eye = cam->transform.LocalPosition();          // カメラの位置を取得
		const VECTOR spawnPos = VAdd(eye, VScale(forward, 2.0f));   // 位置（カメラの前方2mの位置にスポーン）
		auto* obj = SpawnPhysicsSphere({                            // パラメータを直接指定して球を生成
			{"px", std::to_string(spawnPos.x)},                     // 位置（カメラの前方2mの位置）
			{"py", std::to_string(spawnPos.y)},                     // 位置（カメラの前方2mの位置）
			{"pz", std::to_string(spawnPos.z)},                     // 位置（カメラの前方2mの位置）
			{"radius", "0.35"},                                     // 半径（0.35mの球）
			{"material", "metal"},                                  // 材質(metal)
			{"ccd", "true"},                                        // CCD有効化
			{"ccdThreshold", "1.0"},                                // CCD閾値設定（小さな値で厳密なCCDを有効化）
			{"maxLinearSpeed", "200.0"},                            // 最大線形速度設定（高速で飛ばすために十分な値を設定）
			{"color", std::to_string(GetColor(255, 100, 100))}      // 色設定（赤系）
		});
		// 生成後すぐに前方に高速で押し出す（40m/s程度）
		if (obj) {
			auto* body = obj->GetPhysicsBody();                                     // 物理本体取得
			if (body && !body->_isKinematic) {                                      // 動的オブジェクトならCCD設定
				body->_ccdQuality = CcdQuality::Bullet;                             // 最も高品質なCCDモードを設定
				body->_allowedPenetrationDepth = 0.01f;                             // 小さな値で厳密なCCDを有効化
			}
			obj->GetPhysicsBody()->AddVelocityChange(VScale(forward, 40.0f));   // 高速で飛ばす（40m/s程度）
		}
	}
}


//  PhysicsScene - 物理デバッグシーン

// シーン開始（カメラ、ファクトリ登録、アリーナ生成）
void PhysicsScene::Start() {	// シーン開始時の初期化処理
	auto& cameraManager = CameraManager::Instance();                // カメラマネージャーのインスタンスを取得
	const int sceneId = SceneManager::Instance().CurrentSceneId();  // 現在のシーンIDを取得
	ClearDynamicTracking_();                                        // 動的オブジェクトの追跡をクリア（リセット時などに古いオブジェクトの追跡をリセット）

	auto& physics = PhysicsManager::Instance();
	physics.SetSolverIterations(12);
	physics.SetAdaptiveIterationRange(8, 28);
	physics.SetFixedDeltaTime(1.0f / 120.0f);
	physics.SetMaxSubSteps(8);
	physics.SetSplitImpulseEnabled(true);

	// カメラ生成（既に生成されている場合は再利用）
	if (_cameraId == 0 || cameraManager.Get(_cameraId) == nullptr) {    // カメラが未生成または既に削除されている場合は新規生成
		_cameraId = _cameraController.SpawnAuto(                        // カメラコントローラーを使用して自動生成
			sceneId, CameraTag::Debug,                                  // シーンIDとカメラタグを指定
			VGet(0.0f, 10.0f, -25.0f),                                  // 位置（やや高い位置から斜め前方を見下ろす位置）
			VGet(0.30f, 0.0f, 0.0f));                                   // 回転（やや下向きの角度）
    }
	cameraManager.SetRender(_cameraId);                                 // カメラをレンダリング対象に設定

	// ファクトリ登録（初回のみ）
	if (!_registered) {     // まだ登録されていない場合のみ登録処理を行う
		ObjectFactory::Instance().RegisterCreator(                                          // 物理デバッグ用のオブジェクトをファクトリに登録
			PhysicsDebugBox::StaticPoolKey(),                                               // ボックスの生成関数を登録
			[](const VariantMap&) { return std::make_unique<PhysicsDebugBox>(); });         // ラムダ関数でPhysicsDebugBoxのインスタンスを生成して返す
		ObjectFactory::Instance().RegisterCreator(                                          // 球の生成関数を登録
			PhysicsDebugSphere::StaticPoolKey(),											// 球の生成関数を登録
			[](const VariantMap&) { return std::make_unique<PhysicsDebugSphere>(); });		// ラムダ関数でPhysicsDebugSphereのインスタンスを生成して返す
		ObjectFactory::Instance().RegisterCreator(											// カプセルの生成関数を登録
			PhysicsDebugCapsule::StaticPoolKey(),											// カプセルの生成関数を登録
			[](const VariantMap&) { return std::make_unique<PhysicsDebugCapsule>(); });		// ラムダ関数でPhysicsDebugCapsuleのインスタンスを生成して返す
		ObjectManager::Instance().RegisterPool(PhysicsDebugBox::StaticPoolKey(), 160);		// ボックスはやや多めに登録（スタッキングテスト用）
		ObjectManager::Instance().RegisterPool(PhysicsDebugSphere::StaticPoolKey(), 160);	// 球は多めに登録（転がりテスト用）
		ObjectManager::Instance().RegisterPool(PhysicsDebugCapsule::StaticPoolKey(), 64);	// カプセルは少なめに登録（転倒テスト用）
		_registered = true;																	// 登録フラグを立てる（これ以降は登録済みとみなす）
    }

	// アリーナ生成（地面、壁、スロープ、テストオブジェクト群）
    SpawnArena();
}

// 毎フレーム更新（カメラ操作、オブジェクト生成、シーン遷移）
void PhysicsScene::Update(float dtSec) {	// 毎フレームの更新処理
	ObjectManager::Instance().UpdateAll(dtSec);										// 全オブジェクトの更新処理を呼び出す（物理シミュレーションやその他のロジックを更新）

	_cameraController.SetCamera(_cameraId);											// カメラコントローラーにカメラIDをセットして、操作対象のカメラを指定する
	_cameraController.UpdateFreeMoveMouse(10.0f, 0.45f, 8.0f, dtSec);				// カメラコントローラーの更新処理を呼び出す（右クリック＋WASDQEでのフリームーブ操作を処理）

	// 斜面選択: 8=左壁, 9=角, 0=奥壁
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_8)) _selectedRamp = RampSelection::Left;
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_9)) _selectedRamp = RampSelection::Corner;
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_0)) _selectedRamp = RampSelection::Back;

	// 選択中斜面の位置・回転調整
	if (auto* sel = GetSelectedRamp_()) {
		VECTOR p = sel->transform.LocalPosition();
		VECTOR e = sel->transform.LocalEulerRad();
		const float moveSpeed = 4.0f;
		const float rotSpeed = DX_PI_F * 0.25f; // 45deg/sec

		// 位置（矢印: XZ, PgUp/PgDn: Y）
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_LEFT))  p.x -= moveSpeed * dtSec;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_RIGHT)) p.x += moveSpeed * dtSec;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_UP))    p.z += moveSpeed * dtSec;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_DOWN))  p.z -= moveSpeed * dtSec;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_PGUP))  p.y += moveSpeed * dtSec;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_PGDN))  p.y -= moveSpeed * dtSec;

		// 回転（I/K: X, J/L: Y, U/O: Z）
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_I)) e.x += rotSpeed * dtSec;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_K)) e.x -= rotSpeed * dtSec;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_J)) e.y -= rotSpeed * dtSec;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_L)) e.y += rotSpeed * dtSec;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_U)) e.z += rotSpeed * dtSec;
		if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_O)) e.z -= rotSpeed * dtSec;

		sel->transform.SetLocalPosition(p);
		sel->transform.SetLocalEulerRad(e);
	}

	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_1)) SpawnDropObject(1);
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_2)) SpawnDropObject(2);
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_3)) SpawnDropObject(3);
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_F)) FireProjectile();
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_R)) {
		SceneManager::Instance().RequestChange(std::make_unique<PhysicsScene>());
	}

	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_T)) {
		SceneTransition::Params params;
		params.mode = SceneTransition::Mode::MaskImage;
		params.durationSec = 0.4;
		params.maskGraphPath = "Data/Transition/mask.png";
		params.pixelShaderPath = "Data/Transition/mask_transition.pso";
		SceneTransition::Instance().Start(std::make_unique<TitleScene>(), params, 0.5f);
	}
}

// 描画（オブジェクト描画 + UI表示）
void PhysicsScene::Draw() {	// 描画処理
	// 地面グリッドを描画
	const float gridY = 0.02f;
	const int halfCells = 15;
	const float step = 1.0f;
	const unsigned int gridCol = GetColor(80, 80, 80);
	for (int i = -halfCells; i <= halfCells; ++i) {
		const float x = i * step;
		DrawLine3D(VGet(x, gridY, -halfCells * step), VGet(x, gridY, halfCells * step), gridCol);
		const float z = i * step;
		DrawLine3D(VGet(-halfCells * step, gridY, z), VGet(halfCells * step, gridY, z), gridCol);
	}

	ObjectManager::Instance().DrawAll();					// 全オブジェクトの描画処理を呼び出す（物理オブジェクトやその他のオブジェクトを描画）

	const unsigned int white  = GetColor(255, 255, 255);	// 白色（UIテキスト用）
	const unsigned int blue	  = GetColor(200, 220, 255);	// 青色（UIテキスト用）
    const unsigned int yellow = GetColor(255, 220, 140);	// 黄色（UIテキスト用）
    const unsigned int red    = GetColor(255, 180, 180);	// 赤色（UIテキスト用）
    const unsigned int green  = GetColor(180, 255, 180);	// 緑色（UIテキスト用）

	DrawString(10, 10, "PhysicsScene  R: リセット  T: タイトル", white);									// タイトルと基本操作説明
	DrawString(10, 30, "右クリック + WASDQE : フリーカメラ", blue);									// カメラ操作説明
    DrawString(10, 50, "1: Box  2: Sphere  3: Capsule", yellow);			// オブジェクト生成説明
	DrawString(10, 70, "F : 球を発射", red);											// 高速弾発射説明

}
