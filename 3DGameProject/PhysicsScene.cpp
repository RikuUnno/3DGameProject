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
#include "SceneManager.h"
#include "SceneTransition.h"

namespace {
    CameraController _cameraController;
    CameraController::CameraId _cameraId = 0;
    bool _registered = false;
    bool _isSpawningArena = false;

    constexpr size_t _maxDynamicBoxCount = 30;
    constexpr size_t _maxDynamicSphereCount = 40;
    constexpr size_t _maxDynamicCapsuleCount = 20;

    std::deque<PhysicsDebugClass*> _dynamicBoxes;
    std::deque<PhysicsDebugClass*> _dynamicSpheres;
    std::deque<PhysicsDebugClass*> _dynamicCapsules;

    void ReleaseOldestIfNeeded_(std::deque<PhysicsDebugClass*>& objects, size_t maxCount) {
        while (objects.size() >= maxCount && !objects.empty()) {
            PhysicsDebugClass* oldest = objects.front();
            objects.pop_front();
            if (!oldest) continue;
            ObjectManager::Instance().Release(oldest);
        }
    }

    void RegisterDynamicObject_(PhysicsDebugClass* obj, std::deque<PhysicsDebugClass*>& objects, size_t maxCount) {
        if (!obj) return;
        if (_isSpawningArena) return;
        ReleaseOldestIfNeeded_(objects, maxCount);
        objects.push_back(obj);
    }

    void ClearDynamicTracking_() {
        _dynamicBoxes.clear();
        _dynamicSpheres.clear();
        _dynamicCapsules.clear();
    }

    PhysicsDebugClass* SpawnPhysicsObject(const std::string& key, const VariantMap& params) {
        return dynamic_cast<PhysicsDebugClass*>(ObjectManager::Instance().Spawn(key, params));
    }

    PhysicsDebugClass* SpawnPhysicsBox(const VariantMap& params) {
        auto* obj = SpawnPhysicsObject(PhysicsDebugBox::StaticPoolKey(), params);
        RegisterDynamicObject_(obj, _dynamicBoxes, _maxDynamicBoxCount);
        return obj;
    }

    PhysicsDebugClass* SpawnPhysicsSphere(const VariantMap& params) {
        auto* obj = SpawnPhysicsObject(PhysicsDebugSphere::StaticPoolKey(), params);
        RegisterDynamicObject_(obj, _dynamicSpheres, _maxDynamicSphereCount);
        return obj;
    }

    PhysicsDebugClass* SpawnPhysicsCapsule(const VariantMap& params) {
        auto* obj = SpawnPhysicsObject(PhysicsDebugCapsule::StaticPoolKey(), params);
        RegisterDynamicObject_(obj, _dynamicCapsules, _maxDynamicCapsuleCount);
        return obj;
    }

    // ================================================================
    //  アリーナ構築
    // ================================================================
    void SpawnArena() {
        _isSpawningArena = true;

        // ========== 床: 半分ゴム(+X側, 赤), 半分氷(-X側, 水色) ==========
        SpawnPhysicsBox({
            {"static", "true"},
            {"px", "9.0"}, {"py", "-1.0"}, {"pz", "0"},
            {"hx", "9.0"}, {"hy", "1.0"}, {"hz", "18.0"},
            {"material", "rubber"},
            {"color", std::to_string(GetColor(220, 100, 100))}
        });
        SpawnPhysicsBox({
            {"static", "true"},
            {"px", "-9.0"}, {"py", "-1.0"}, {"pz", "0"},
            {"hx", "9.0"}, {"hy", "1.0"}, {"hz", "18.0"},
            {"material", "ice"},
            {"color", std::to_string(GetColor(180, 230, 255))}
        });

        // ========== 壁: 木（4面） ==========
        const unsigned int wallColor = GetColor(180, 140, 80);
        SpawnPhysicsBox({
            {"static", "true"},
            {"px", "0"}, {"py", "4.0"}, {"pz", "18.0"},
            {"hx", "18.0"}, {"hy", "5.0"}, {"hz", "1.0"},
            {"material", "wood"},
            {"color", std::to_string(wallColor)}
        });
        SpawnPhysicsBox({
            {"static", "true"},
            {"px", "0"}, {"py", "4.0"}, {"pz", "-18.0"},
            {"hx", "18.0"}, {"hy", "5.0"}, {"hz", "1.0"},
            {"material", "wood"},
            {"color", std::to_string(wallColor)}
        });
        SpawnPhysicsBox({
            {"static", "true"},
            {"px", "18.0"}, {"py", "4.0"}, {"pz", "0"},
            {"hx", "1.0"}, {"hy", "5.0"}, {"hz", "18.0"},
            {"material", "wood"},
            {"color", std::to_string(wallColor)}
        });
        SpawnPhysicsBox({
            {"static", "true"},
            {"px", "-18.0"}, {"py", "4.0"}, {"pz", "0"},
            {"hx", "1.0"}, {"hy", "5.0"}, {"hz", "18.0"},
            {"material", "wood"},
            {"color", std::to_string(wallColor)}
        });

        // ========== 斜め坂: 鉄 ==========
        const unsigned int rampColor = GetColor(160, 170, 180);
        auto* ramp1 = SpawnPhysicsBox({
            {"static", "true"},
            {"px", "-8.0"}, {"py", "0.6"}, {"pz", "-6.0"},
            {"hx", "4.0"}, {"hy", "0.3"}, {"hz", "5.0"},
            {"material", "metal"},
            {"color", std::to_string(rampColor)}
        });
        if (ramp1) {
            ramp1->transform.SetLocalEulerRad(VGet(0.0f, 0.0f, -0.35f));
        }
        auto* ramp2 = SpawnPhysicsBox({
            {"static", "true"},
            {"px", "-10.0"}, {"py", "0.6"}, {"pz", "8.0"},
            {"hx", "3.5"}, {"hy", "0.3"}, {"hz", "4.0"},
            {"material", "metal"},
            {"color", std::to_string(rampColor)}
        });
        if (ramp2) {
            ramp2->transform.SetLocalEulerRad(VGet(-0.30f, 0.4f, -0.25f));
        }

        // ========== 積み重ねBox: 氷（ゴム床側に配置） ==========
        const unsigned int iceBoxColor = GetColor(200, 240, 255);
        for (int y = 0; y < 5; ++y) {
            for (int x = 0; x < 3; ++x) {
                SpawnPhysicsBox({
                    {"px", std::to_string(4.0f + x * 1.05f)},
                    {"py", std::to_string(0.5f + y * 1.01f)},
                    {"pz", "5.0"},
                    {"hx", "0.5"}, {"hy", "0.5"}, {"hz", "0.5"},
                    {"material", "ice"},
                    {"ccd", "true"},
                    {"ccdThreshold", "2.0"},
                    {"color", std::to_string(iceBoxColor)}
                });
            }
        }
        for (int y = 0; y < 4; ++y) {
            for (int x = 0; x < 2; ++x) {
                SpawnPhysicsBox({
                    {"px", std::to_string(10.0f + x * 1.05f)},
                    {"py", std::to_string(0.5f + y * 1.01f)},
                    {"pz", "-4.0"},
                    {"hx", "0.5"}, {"hy", "0.5"}, {"hz", "0.5"},
                    {"material", "ice"},
                    {"ccd", "true"},
                    {"ccdThreshold", "2.0"},
                    {"color", std::to_string(iceBoxColor)}
                });
            }
        }

        // ========== 球: 鉄（複数） ==========
        const unsigned int metalBallColor = GetColor(180, 185, 195);
        // ゴム床側（重くてグリップする）
        for (int i = 0; i < 4; ++i) {
            SpawnPhysicsSphere({
                {"px", std::to_string(6.0f + i * 1.8f)},
                {"py", "1.5"},
                {"pz", std::to_string(-2.0f + i * 0.5f)},
                {"radius", "0.5"},
                {"material", "metal"},
                {"ccd", "true"},
                {"ccdThreshold", "3.0"},
                {"color", std::to_string(metalBallColor)}
            });
        }
        // 氷床側（滑って転がる）
        for (int i = 0; i < 4; ++i) {
            SpawnPhysicsSphere({
                {"px", std::to_string(-6.0f - i * 1.8f)},
                {"py", "1.5"},
                {"pz", std::to_string(2.0f - i * 0.5f)},
                {"radius", "0.45"},
                {"material", "metal"},
                {"ccd", "true"},
                {"ccdThreshold", "3.0"},
                {"color", std::to_string(metalBallColor)}
            });
        }
        // 坂の上（転がり落ちる）
        SpawnPhysicsSphere({
            {"px", "-9.0"}, {"py", "2.5"}, {"pz", "-6.0"},
            {"radius", "0.4"},
            {"material", "metal"},
            {"ccd", "true"},
            {"ccdThreshold", "3.0"},
            {"color", std::to_string(metalBallColor)}
        });
        SpawnPhysicsSphere({
            {"px", "-10.5"}, {"py", "2.5"}, {"pz", "8.0"},
            {"radius", "0.4"},
            {"material", "metal"},
            {"ccd", "true"},
            {"ccdThreshold", "3.0"},
            {"color", std::to_string(metalBallColor)}
        });

        _isSpawningArena = false;
    }

    // カメラ前方にオブジェクトを落とす
    void SpawnDropObject(int type) {
        auto* cam = CameraManager::Instance().Get(_cameraId);
        if (!cam) return;

        const VECTOR forward = cam->transform.Forward();
        const VECTOR eye = cam->transform.LocalPosition();
        const VECTOR spawnPos = VAdd(eye, VScale(forward, 3.0f));
        VariantMap params;
        params["px"] = std::to_string(spawnPos.x);
        params["py"] = std::to_string(spawnPos.y);
        params["pz"] = std::to_string(spawnPos.z);
        params["ccd"] = "true";
        params["ccdThreshold"] = "2.0";

        PhysicsDebugClass* obj = nullptr;
        if (type == 1) {
            params["hx"] = "0.5";
            params["hy"] = "0.5";
            params["hz"] = "0.5";
            params["material"] = "wood";
            obj = SpawnPhysicsBox(params);
        }
        else if (type == 2) {
            params["radius"] = "0.45";
            params["material"] = "metal";
            obj = SpawnPhysicsSphere(params);
        }
        else {
            params["radius"] = "0.38";
            params["halfHeight"] = "0.75";
            params["material"] = "rubber";
            obj = SpawnPhysicsCapsule(params);
        }

        if (obj) {
            // AddVelocityChange で直接速度を設定（質量に依存しない）
            obj->GetPhysicsBody()->AddVelocityChange(VScale(forward, 3.0f));
        }
    }

    // F: 鉄のボールを高速発射（CCD閾値を極限まで低く設定してすり抜け防止）
    void FireProjectile() {
        auto* cam = CameraManager::Instance().Get(_cameraId);
        if (!cam) return;

        const VECTOR forward = cam->transform.Forward();
        const VECTOR eye = cam->transform.LocalPosition();
        const VECTOR spawnPos = VAdd(eye, VScale(forward, 2.0f));
        auto* obj = SpawnPhysicsSphere({
            {"px", std::to_string(spawnPos.x)},
            {"py", std::to_string(spawnPos.y)},
            {"pz", std::to_string(spawnPos.z)},
            {"radius", "0.35"},
            {"material", "metal"},
            {"ccd", "true"},
            {"ccdThreshold", "1.0"},
            {"maxLinearSpeed", "200.0"},
        });
        if (obj) {
            // AddVelocityChange で直接速度を設定（質量に依存しない）
            obj->GetPhysicsBody()->AddVelocityChange(VScale(forward, 40.0f));
        }
    }
}

void PhysicsScene::Start() {
    auto& cameraManager = CameraManager::Instance();
    const int sceneId = SceneManager::Instance().CurrentSceneId();
    ClearDynamicTracking_();

    if (_cameraId == 0 || cameraManager.Get(_cameraId) == nullptr) {
        _cameraId = _cameraController.SpawnAuto(sceneId, CameraTag::Debug, VGet(0.0f, 8.0f, -22.0f), VGet(0.22f, 0.0f, 0.0f));
    }
    cameraManager.SetRender(_cameraId);

    if (!_registered) {
        ObjectFactory::Instance().RegisterCreator(PhysicsDebugBox::StaticPoolKey(), [](const VariantMap&) { return std::make_unique<PhysicsDebugBox>(); });
        ObjectFactory::Instance().RegisterCreator(PhysicsDebugSphere::StaticPoolKey(), [](const VariantMap&) { return std::make_unique<PhysicsDebugSphere>(); });
        ObjectFactory::Instance().RegisterCreator(PhysicsDebugCapsule::StaticPoolKey(), [](const VariantMap&) { return std::make_unique<PhysicsDebugCapsule>(); });
        ObjectManager::Instance().RegisterPool(PhysicsDebugBox::StaticPoolKey(), 160);
        ObjectManager::Instance().RegisterPool(PhysicsDebugSphere::StaticPoolKey(), 160);
        ObjectManager::Instance().RegisterPool(PhysicsDebugCapsule::StaticPoolKey(), 64);
        _registered = true;
    }

    SpawnArena();
}

void PhysicsScene::Update(float dtSec) {
    ObjectManager::Instance().UpdateAll(dtSec);

    _cameraController.SetCamera(_cameraId);
    _cameraController.UpdateFreeMoveMouse(10.0f, 0.45f, 8.0f, dtSec);

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

void PhysicsScene::Draw() {
    ObjectManager::Instance().DrawAll();

    DrawString(10, 10, "PhysicsScene - R:リセット T:タイトル", GetColor(255, 255, 255));
    DrawString(10, 30, "右クリック+WASDQE : フリーカメラ操作", GetColor(200, 220, 255));
    DrawString(10, 50, "1:木の箱  2:鉄の球  3:ゴムカプセル を前方に落とす", GetColor(255, 220, 140));
    DrawString(10, 70, "F : 鉄のボールを高速発射", GetColor(255, 180, 180));
    DrawString(10, 90, "床: 右=ゴム(赤) 左=氷(水色) / 壁:木 / 坂:鉄 / 積Box:氷 / 球:鉄", GetColor(180, 255, 180));
}
