// ============================================================
//  PhysicsScene - 物理デバッグシーン
// ============================================================
//  物理エンジンのテスト用シーン。様々な形状・マテリアルの組み合わせで
//  衝突、摩擦、反発、CCD（連続衝突検出）等をテストできる。
//
//  【操作】
//  R: リセット / T: タイトル / 右クリック+WASDQE: カメラ移動
//  1: 木ボックス投下 / 2: 金属球投下 / 3: ゴムカプセル投下
//  F: 高速弾発射（CCDテスト）
// ============================================================

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

    // ================================================================
    //  アリーナ生成（地面、壁、スロープ、テストオブジェクト群）
    // ================================================================
    //  - Floor: Stone材質（36x36m）
    //  - Walls: Wood材質（4面）
    //  - Ramp: Metal材質（傾斜）
    //  - Stacked Boxes: Wood材質（3列x4段、スタッキングテスト）
    //  - Pyramid: Wood材質（崩壊テスト）
    //  - Metal Spheres: スロープ転がりテスト
    //  - Bouncy Balls: 高反発テスト
    //  - Capsules: 転倒テスト
    //  - Ice Blocks: 低摩擦テスト
    // ================================================================
    void SpawnArena() {
		_isSpawningArena = true;    // アリーナ生成中フラグを立てる（生成中は動的オブジェクトの追跡をスキップ）
        
		// カラー定数
        const unsigned int colFloor   = GetColor(200, 200, 200);
        const unsigned int colWall    = GetColor(160, 140, 120);
        const unsigned int colRamp    = GetColor(170, 175, 185);
        const unsigned int colWood    = GetColor(190, 150, 90);
        const unsigned int colMetal   = GetColor(180, 185, 195);
        const unsigned int colBouncy  = GetColor(100, 220, 100);
        const unsigned int colIce     = GetColor(200, 235, 255);

		// Floor: Stone (36x36) - 大きめの床で転がりやスタッキングのテストをしやすくする 
        SpawnPhysicsBox({
			{"static", "true"},                             // 静的オブジェクトフラグ
			{"px", "0"}, {"py", "-1.0"}, {"pz", "0"},       // 位置
			{"hx", "18.0"}, {"hy", "1.0"}, {"hz", "18.0"},  // ハーフサイズ（18m x 18m x 1mの床）
			{"material", "block"},                          // 材質(block)
			{"color", std::to_string(colFloor)}             // 色設定（グレー系）
        });

		// Walls: Wood (4 sides) - 床より少し高めの壁で、転がりやスタッキングのテストをしやすくする
		SpawnPhysicsBox({       // 前後の壁
			{"static", "true"},                             // 静的オブジェクトフラグ
			{"px", "0"}, {"py", "3.0"}, {"pz", "18.5"},     // 位置（床より3m高く、前後に18.5m）
			{"hx", "19.0"}, {"hy", "4.0"}, {"hz", "0.5"},   // ハーフサイズ（19m x 4m x 0.5mの壁）
			{"material", "wood"},                           // 材質(wood)
			{"color", std::to_string(colWall)}              // 色設定（茶色系）
        });
		SpawnPhysicsBox({                                   
			{"static", "true"},                             // 静的オブジェクトフラグ
			{"px", "0"}, {"py", "3.0"}, {"pz", "-18.5"},    // 位置（床より3m高く、前後に18.5m）
			{"hx", "19.0"}, {"hy", "4.0"}, {"hz", "0.5"},   // ハーフサイズ（19m x 4m x 0.5mの壁）
			{"material", "wood"},                           // 材質(wood)
			{"color", std::to_string(colWall)}              // 色設定（茶色系）
        });
		SpawnPhysicsBox({       // 左右の壁
			{"static", "true"},                             // 静的オブジェクトフラグ
			{"px", "18.5"}, {"py", "3.0"}, {"pz", "0"},     // 位置（床より3m高く、左右に18.5m）
			{"hx", "0.5"}, {"hy", "4.0"}, {"hz", "18.0"},   // ハーフサイズ（0.5m x 4m x 18mの壁）
			{"material", "wood"},                           // 材質(wood)
			{"color", std::to_string(colWall)}              // 色設定（茶色系）
        });
        SpawnPhysicsBox({
			{"static", "true"},                             // 静的オブジェクトフラグ
			{"px", "-18.5"}, {"py", "3.0"}, {"pz", "0"},    // 位置（床より3m高く、左右に18.5m）
			{"hx", "0.5"}, {"hy", "4.0"}, {"hz", "18.0"},   // ハーフサイズ（0.5m x 4m x 18mの壁）
			{"material", "wood"},                           // 材質(wood)
			{"color", std::to_string(colWall)}              // 色設定（茶色系）
        });

		// Ramp: Metal - 床より少し高めのスロープで、転がりやスタッキングのテストをしやすくする
        auto* ramp = SpawnPhysicsBox({
			{"static", "true"},                             // 静的オブジェクトフラグ
			{"px", "-10.0"}, {"py", "1.0"}, {"pz", "-6.0"}, // 位置（床より1m高く、左前に配置）
			{"hx", "5.0"}, {"hy", "0.25"}, {"hz", "4.0"},   // ハーフサイズ（5m x 0.25m x 4mのスロープ）
			{"material", "metal"},                          // 材質(metal)
			{"color", std::to_string(colRamp)}              // 色設定（灰色系）
        });
		// スロープを約17度傾ける（tan(17°) ≈ 0.3）
        if (ramp) {
			ramp->transform.SetLocalEulerRad(VGet(0.0f, 0.0f, -0.3f)); // Z軸回転で傾ける（右肩下がりのスロープ）
        }

		// Stacked Wood Boxes: 3 cols x 4 rows - スタッキングとCCDのテスト
		for (int row = 0; row < 4; ++row) {         // 4段積み
			for (int col = 0; col < 3; ++col) {     // 3列
                SpawnPhysicsBox({
					{"px", std::to_string(5.0f + col * 1.05f)},     // 位置（右前に配置、列ごとに1.05m間隔）
					{"py", std::to_string(0.5f + row * 1.02f)},     // 位置（段ごとに1.02m間隔で積む）
					{"pz", "6.0"},                                  // 位置（前方に配置）
					{"hx", "0.5"}, {"hy", "0.5"}, {"hz", "0.5"},    // ハーフサイズ（0.5mの立方体）
					{"material", "wood"},                           // 材質(wood)
					{"ccd", "true"},                                // CCD有効化
					{"ccdThreshold", "2.0"},                        // CCD閾値設定（小さな値で厳密なCCDを有効化）
					{"color", std::to_string(colWood)}              // 色設定（茶色系）
                });
            }
        }

		// Pyramid Boxes - ピラミッド状に積む（崩壊テスト）
        for (int row = 0; row < 3; ++row) {         // 3段積み
            const int count = 3 - row;
            for (int col = 0; col < count; ++col) { // 各段の列数
                SpawnPhysicsBox({
                    {"px", std::to_string(12.0f + col * 1.05f)},    // 位置（右前に配置、列ごとに1.05m間隔）
                    {"py", std::to_string(0.5f + row * 1.02f)},     // 位置（段ごとに1.02m間隔で積む）
                    {"pz", "-5.0"},                                 // 位置（前方に配置）
                    {"hx", "0.5"}, {"hy", "0.5"}, {"hz", "0.5"},    // ハーフサイズ（0.5mの立方体）
                    {"material", "wood"},                           // 材質(wood)
                    {"ccd", "true"},                                // CCD有効化
                    {"ccdThreshold", "2.0"},                        // CCD閾値設定（小さな値で厳密なCCDを有効化）
					{"color", std::to_string(colWood)}              // 色設定（茶色系）
                });
            }
        }

		// Metal Spheres: On ramp - スロープ上に金属球を配置して転がりとCCDのテスト
		SpawnPhysicsSphere({        // 位置（スロープの上に配置）
			{"px", "-12.0"}, {"py", "3.0"}, {"pz", "-6.0"},     // 位置（スロープの上に配置）
			{"radius", "0.45"},                                 // 半径（0.45mの球）
			{"material", "metal"},                              // 材質(metal)
            {"ccd", "true"},                                    // CCD有効化
			{"ccdThreshold", "3.0"},                            // CCD閾値設定（小さな値で厳密なCCDを有効化）
			{"color", std::to_string(colMetal)}                 // 色設定（灰色系）
        });
		SpawnPhysicsSphere({        // 位置（スロープの上に配置）
			{"px", "-11.0"}, {"py", "2.5"}, {"pz", "-5.0"},     // 位置（スロープの上に配置）
			{"radius", "0.4"},                                  // 半径（0.4mの球）
			{"material", "metal"},                              // 材質(metal)
			{"ccd", "true"},                                    // CCD有効化
			{"ccdThreshold", "3.0"},                            // CCD閾値設定（小さな値で厳密なCCDを有効化）
			{"color", std::to_string(colMetal)}                 // 色設定（灰色系）
        });

		// Bouncy Balls: High drop - 高い位置から落とす高反発球で、反発テストとCCDのテスト
		SpawnPhysicsSphere({        // 位置（高い位置から落とす）
			{"px", "0"}, {"py", "8.0"}, {"pz", "0"},            // 位置（中央の高い位置から落とす）
			{"radius", "0.5"},                                  // 半径（0.5mの球）
			{"material", "bouncy"},                             // 材質(bouncy(ゴムのような高反発材質))
            {"ccd", "true"},                                    // CCD有効化
            {"ccdThreshold", "3.0"},                            // CCD閾値設定（小さな値で厳密なCCDを有効化）
            {"color", std::to_string(colBouncy)}                // 色設定（赤系）
        });
		SpawnPhysicsSphere({        // 位置（高い位置から落とす）
			{"px", "2.0"}, {"py", "10.0"}, {"pz", "-2.0"},      // 位置（やや右のさらに高い位置から落とす）
			{"radius", "0.35"},                                 // 半径（0.35mの球）
			{"material", "bouncy"},                             // 材質(bouncy(ゴムのような高反発材質))
			{"ccd", "true"},                                    // CCD有効化
			{"ccdThreshold", "3.0"},                            // CCD閾値設定（小さな値で厳密なCCDを有効化）
			{"color", std::to_string(colBouncy)}                // 色設定（赤系）
        });

		// Capsules: Topple test - 転倒テスト用のカプセルを配置
		SpawnPhysicsCapsule({       // 位置（やや左の位置に配置）
			{"px", "-4.0"}, {"py", "1.5"}, {"pz", "0"},         // 位置（やや左の位置に配置）
			{"radius", "0.4"},                                  // 半径（0.4mのカプセル）
            {"halfHeight", "0.8"},                              // 半分の高さ（0.8mのカプセル）
            {"material", "rubber"},                             // 材質(rubber(ゴムのような材質))
            {"color", std::to_string(GetColor(220, 120, 120))}  // 色設定（赤系）
        });
		SpawnPhysicsCapsule({       // 位置（さらに左の位置に配置）
			{"px", "-6.0"}, {"py", "1.5"}, {"pz", "2.0"},       // 位置（さらに左の位置に配置）
			{"radius", "0.35"},                                 // 半径（0.35mのカプセル）
			{"halfHeight", "0.7"},                              // 半分の高さ（0.7mのカプセル）
			{"material", "wood"},                               // 材質(wood)
			{"color", std::to_string(colWood)}                  // 色設定（茶色系）
        });

		// Ice Blocks: Low-friction test - 低摩擦テスト用の氷ブロックを配置
		for (int i = 0; i < 3; ++i) {   // 3つの氷ブロックを配置
			SpawnPhysicsBox({       // 位置（やや左の位置に配置、ブロックごとに1.1m間隔で配置）
				{"px", std::to_string(-3.0f + i * 1.1f)},       // 位置（やや左の位置に配置、ブロックごとに1.1m間隔で配置）
				{"py", "0.5"},                                  // 位置（床から0.5mの高さに配置）
				{"pz", "-10.0"},                                // 位置（前方に配置）
				{"hx", "0.5"}, {"hy", "0.5"}, {"hz", "0.5"},    // ハーフサイズ（0.5mの立方体）
				{"material", "ice"},                            // 材質(ice(氷のような低摩擦材質))
				{"color", std::to_string(colIce)}               // 色設定（水色系）
            });
        }

		_isSpawningArena = false;   // アリーナ生成完了フラグを下ろす（これ以降は動的オブジェクトの追跡を有効化）
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
	physics.SetSolverIterations(8);
	physics.SetAdaptiveIterationRange(6, 20);
	physics.SetFixedDeltaTime(1.0f / 60.0f);
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

	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_1)) SpawnDropObject(1);	// キー入力1でBoxをスポーン
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_2)) SpawnDropObject(2);	// キー入力2でSphereをスポーン
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_3)) SpawnDropObject(3);	// キー入力3でCapsuleをスポーン
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_F)) FireProjectile();		// キー入力Fで高速弾を発射
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_R)) {						// キー入力Rでシーンリセット
		SceneManager::Instance().RequestChange(std::make_unique<PhysicsScene>());	// 新しいPhysicsSceneを生成してシーン遷移をリクエスト（これによりシーンがリセットされる）
    }

	// シーン遷移（キー入力Tでタイトルシーンへ遷移、マスク画像を使用したエフェクト付き）
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_T)) {								// キー入力Tでタイトルシーンへ遷移
		SceneTransition::Params params;														// シーン遷移のパラメータを設定
		params.mode = SceneTransition::Mode::MaskImage;										// マスク画像を使用した遷移モードを指定
		params.durationSec = 0.4;															// 遷移時間を0.4秒に設定
		params.maskGraphPath = "Data/Transition/mask.png";									// マスク画像のパスを指定（この画像の白い部分が遷移の中心になる）
		params.pixelShaderPath = "Data/Transition/mask_transition.pso";						// ピクセルシェーダーのパスを指定（マスク画像を使用した遷移エフェクトを実装したシェーダー）
		SceneTransition::Instance().Start(std::make_unique<TitleScene>(), params, 0.5f);	// タイトルシーンへの遷移を開始（新しいTitleSceneを生成して遷移を開始、遷移エフェクトのパラメータも渡す）
    }
}

// 描画（オブジェクト描画 + UI表示）
void PhysicsScene::Draw() {	// 描画処理
	ObjectManager::Instance().DrawAll();					// 全オブジェクトの描画処理を呼び出す（物理オブジェクトやその他のオブジェクトを描画）

	const unsigned int white  = GetColor(255, 255, 255);	// 白色（UIテキスト用）
	const unsigned int blue	  = GetColor(200, 220, 255);	// 青色（UIテキスト用）
    const unsigned int yellow = GetColor(255, 220, 140);	// 黄色（UIテキスト用）
    const unsigned int red    = GetColor(255, 180, 180);	// 赤色（UIテキスト用）
    const unsigned int green  = GetColor(180, 255, 180);	// 緑色（UIテキスト用）

	DrawString(10, 10, "PhysicsScene  R: Reset  T: Title", white);												// タイトルと基本操作説明
	DrawString(10, 30, "RightClick + WASDQE : Free Camera", blue);												// カメラ操作説明
    DrawString(10, 50,  "1: Wood Box  2: Metal Sphere  3: Rubber Capsule  (drop forward)", yellow);				// オブジェクト生成説明
	DrawString(10, 70, "F : Fire Metal Ball", red);																// 高速弾発射説明
	DrawString(10, 90, "Floor: Stone / Wall: Wood / Ramp: Metal / Stack: Wood / Ball: Bouncy+Metal", green);	// アリーナのオブジェクト説明
}
