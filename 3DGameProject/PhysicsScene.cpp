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
        // CCD有効化とスリープ設定
        if (obj) {
            auto* body = obj->GetPhysicsBody();
            if (body && !body->_isKinematic) {
                body->_ccdQuality = CcdQuality::Bullet;
                body->_allowedPenetrationDepth = 0.01f;
                body->_sleepLinearThreshold = 0.01f;
                body->_sleepAngularThreshold = 0.01f;
                body->_sleepTimeThreshold = 0.3f;
            }
        }
        return obj;
    }

    PhysicsDebugClass* SpawnPhysicsSphere(const VariantMap& params) {
        auto* obj = SpawnPhysicsObject(PhysicsDebugSphere::StaticPoolKey(), params);
        RegisterDynamicObject_(obj, _dynamicSpheres, _maxDynamicSphereCount);
        // CCD有効化とスリープ設定
        if (obj) {
            auto* body = obj->GetPhysicsBody();
            if (body && !body->_isKinematic) {
                body->_ccdQuality = CcdQuality::Bullet;
                body->_allowedPenetrationDepth = 0.01f;
                body->_sleepLinearThreshold = 0.01f;
                body->_sleepAngularThreshold = 0.01f;
                body->_sleepTimeThreshold = 0.3f;
            }
        }
        return obj;
    }

    PhysicsDebugClass* SpawnPhysicsCapsule(const VariantMap& params) {
        auto* obj = SpawnPhysicsObject(PhysicsDebugCapsule::StaticPoolKey(), params);
        RegisterDynamicObject_(obj, _dynamicCapsules, _maxDynamicCapsuleCount);
        // CCD有効化とスリープ設定
        if (obj) {
            auto* body = obj->GetPhysicsBody();
            if (body && !body->_isKinematic) {
                body->_ccdQuality = CcdQuality::Bullet;
                body->_allowedPenetrationDepth = 0.01f;
                body->_sleepLinearThreshold = 0.01f;
                body->_sleepAngularThreshold = 0.01f;
                body->_sleepTimeThreshold = 0.3f;
            }
        }
        return obj;
    }

    // ================================================================
    //  Arena
    // ================================================================
    void SpawnArena() {
        _isSpawningArena = true;

        const unsigned int colFloor   = GetColor(200, 200, 200);
        const unsigned int colWall    = GetColor(160, 140, 120);
        const unsigned int colRamp    = GetColor(170, 175, 185);
        const unsigned int colWood    = GetColor(190, 150, 90);
        const unsigned int colMetal   = GetColor(180, 185, 195);
        const unsigned int colBouncy  = GetColor(100, 220, 100);
        const unsigned int colIce     = GetColor(200, 235, 255);

        // ========== Floor: Stone (36x36) ==========
        SpawnPhysicsBox({
            {"static", "true"},
            {"px", "0"}, {"py", "-1.0"}, {"pz", "0"},
            {"hx", "18.0"}, {"hy", "1.0"}, {"hz", "18.0"},
            {"material", "stone"},
            {"color", std::to_string(colFloor)}
        });

        // ========== Walls: Wood (4 sides) ==========
        SpawnPhysicsBox({
            {"static", "true"},
            {"px", "0"}, {"py", "3.0"}, {"pz", "18.5"},
            {"hx", "19.0"}, {"hy", "4.0"}, {"hz", "0.5"},
            {"material", "wood"},
            {"color", std::to_string(colWall)}
        });
        SpawnPhysicsBox({
            {"static", "true"},
            {"px", "0"}, {"py", "3.0"}, {"pz", "-18.5"},
            {"hx", "19.0"}, {"hy", "4.0"}, {"hz", "0.5"},
            {"material", "wood"},
            {"color", std::to_string(colWall)}
        });
        SpawnPhysicsBox({
            {"static", "true"},
            {"px", "18.5"}, {"py", "3.0"}, {"pz", "0"},
            {"hx", "0.5"}, {"hy", "4.0"}, {"hz", "18.0"},
            {"material", "wood"},
            {"color", std::to_string(colWall)}
        });
        SpawnPhysicsBox({
            {"static", "true"},
            {"px", "-18.5"}, {"py", "3.0"}, {"pz", "0"},
            {"hx", "0.5"}, {"hy", "4.0"}, {"hz", "18.0"},
            {"material", "wood"},
            {"color", std::to_string(colWall)}
        });

        // ========== Ramp: Metal ==========
        auto* ramp = SpawnPhysicsBox({
            {"static", "true"},
            {"px", "-10.0"}, {"py", "1.0"}, {"pz", "-6.0"},
            {"hx", "5.0"}, {"hy", "0.25"}, {"hz", "4.0"},
            {"material", "metal"},
            {"color", std::to_string(colRamp)}
        });
        if (ramp) {
            ramp->transform.SetLocalEulerRad(VGet(0.0f, 0.0f, -0.3f));
        }

        // ========== Stacked Wood Boxes: 3 cols x 4 rows ==========
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 3; ++col) {
                SpawnPhysicsBox({
                    {"px", std::to_string(5.0f + col * 1.05f)},
                    {"py", std::to_string(0.5f + row * 1.02f)},
                    {"pz", "6.0"},
                    {"hx", "0.5"}, {"hy", "0.5"}, {"hz", "0.5"},
                    {"material", "wood"},
                    {"ccd", "true"},
                    {"ccdThreshold", "2.0"},
                    {"color", std::to_string(colWood)}
                });
            }
        }

        // ========== Pyramid Boxes (far right) ==========
        for (int row = 0; row < 3; ++row) {
            const int count = 3 - row;
            for (int col = 0; col < count; ++col) {
                SpawnPhysicsBox({
                    {"px", std::to_string(12.0f + col * 1.05f)},
                    {"py", std::to_string(0.5f + row * 1.02f)},
                    {"pz", "-5.0"},
                    {"hx", "0.5"}, {"hy", "0.5"}, {"hz", "0.5"},
                    {"material", "wood"},
                    {"ccd", "true"},
                    {"ccdThreshold", "2.0"},
                    {"color", std::to_string(colWood)}
                });
            }
        }

        // ========== Metal Spheres: On ramp ==========
        SpawnPhysicsSphere({
            {"px", "-12.0"}, {"py", "3.0"}, {"pz", "-6.0"},
            {"radius", "0.45"},
            {"material", "metal"},
            {"ccd", "true"},
            {"ccdThreshold", "3.0"},
            {"color", std::to_string(colMetal)}
        });
        SpawnPhysicsSphere({
            {"px", "-11.0"}, {"py", "2.5"}, {"pz", "-5.0"},
            {"radius", "0.4"},
            {"material", "metal"},
            {"ccd", "true"},
            {"ccdThreshold", "3.0"},
            {"color", std::to_string(colMetal)}
        });

        // ========== Bouncy Balls: High drop ==========
        SpawnPhysicsSphere({
            {"px", "0"}, {"py", "8.0"}, {"pz", "0"},
            {"radius", "0.5"},
            {"material", "bouncy"},
            {"ccd", "true"},
            {"ccdThreshold", "3.0"},
            {"color", std::to_string(colBouncy)}
        });
        SpawnPhysicsSphere({
            {"px", "2.0"}, {"py", "10.0"}, {"pz", "-2.0"},
            {"radius", "0.35"},
            {"material", "bouncy"},
            {"ccd", "true"},
            {"ccdThreshold", "3.0"},
            {"color", std::to_string(colBouncy)}
        });

        // ========== Capsules: Topple test ==========
        SpawnPhysicsCapsule({
            {"px", "-4.0"}, {"py", "1.5"}, {"pz", "0"},
            {"radius", "0.4"},
            {"halfHeight", "0.8"},
            {"material", "rubber"},
            {"color", std::to_string(GetColor(220, 120, 120))}
        });
        SpawnPhysicsCapsule({
            {"px", "-6.0"}, {"py", "1.5"}, {"pz", "2.0"},
            {"radius", "0.35"},
            {"halfHeight", "0.7"},
            {"material", "wood"},
            {"color", std::to_string(colWood)}
        });

        // ========== Ice Blocks: Low-friction test ==========
        for (int i = 0; i < 3; ++i) {
            SpawnPhysicsBox({
                {"px", std::to_string(-3.0f + i * 1.1f)},
                {"py", "0.5"},
                {"pz", "-10.0"},
                {"hx", "0.5"}, {"hy", "0.5"}, {"hz", "0.5"},
                {"material", "ice"},
                {"color", std::to_string(colIce)}
            });
        }

        _isSpawningArena = false;
    }

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
            obj->GetPhysicsBody()->AddVelocityChange(VScale(forward, 3.0f));
        }
    }

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
            obj->GetPhysicsBody()->AddVelocityChange(VScale(forward, 40.0f));
        }
    }
}

void PhysicsScene::Start() {
    auto& cameraManager = CameraManager::Instance();
    const int sceneId = SceneManager::Instance().CurrentSceneId();
    ClearDynamicTracking_();

    if (_cameraId == 0 || cameraManager.Get(_cameraId) == nullptr) {
        _cameraId = _cameraController.SpawnAuto(
            sceneId, CameraTag::Debug,
            VGet(0.0f, 10.0f, -25.0f),
            VGet(0.30f, 0.0f, 0.0f));
    }
    cameraManager.SetRender(_cameraId);

    if (!_registered) {
        ObjectFactory::Instance().RegisterCreator(
            PhysicsDebugBox::StaticPoolKey(),
            [](const VariantMap&) { return std::make_unique<PhysicsDebugBox>(); });
        ObjectFactory::Instance().RegisterCreator(
            PhysicsDebugSphere::StaticPoolKey(),
            [](const VariantMap&) { return std::make_unique<PhysicsDebugSphere>(); });
        ObjectFactory::Instance().RegisterCreator(
            PhysicsDebugCapsule::StaticPoolKey(),
            [](const VariantMap&) { return std::make_unique<PhysicsDebugCapsule>(); });
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

    const unsigned int white  = GetColor(255, 255, 255);
    const unsigned int blue   = GetColor(200, 220, 255);
    const unsigned int yellow = GetColor(255, 220, 140);
    const unsigned int red    = GetColor(255, 180, 180);
    const unsigned int green  = GetColor(180, 255, 180);

    DrawString(10, 10,  "PhysicsScene  R: Reset  T: Title", white);
    DrawString(10, 30,  "RightClick + WASDQE : Free Camera", blue);
    DrawString(10, 50,  "1: Wood Box  2: Metal Sphere  3: Rubber Capsule  (drop forward)", yellow);
    DrawString(10, 70,  "F : Fire Metal Ball", red);
    DrawString(10, 90,  "Floor: Stone / Wall: Wood / Ramp: Metal / Stack: Wood / Ball: Bouncy+Metal", green);
}
