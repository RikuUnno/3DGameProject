#include "CollisionScene.h"

#include <cmath>
#include <memory>
#include <string>

#include "CameraManager.h"
#include "CollisionDebugClass.h"
#include "Debug Class.h"
#include "DxLib.h"
#include "KeyInput.h"
#include "ObjectFactory.h"
#include "ObjectManager.h"
#include "SceneManager.h"
#include "SceneTransition.h"
#include "DemoMenuScene.h"

void CollisionScene::Start()
{
    auto& camMgr = CameraManager::Instance();
    const int sceneId = SceneManager::Instance().CurrentSceneId();
    RegisterPools_();
    SpawnDemoObjects_();
    CollisionDebugClass::ResetEventText();

    if (_debugCamId == 0 || camMgr.Get(_debugCamId) == nullptr)
        _debugCamId = _debugCamCtrl.SpawnAuto(sceneId, CameraTag::Debug, VGet(0.0f, 5.5f, -14.0f), VGet(0.10f, 0.0f, 0.0f));
    if (_gameCamId == 0 || camMgr.Get(_gameCamId) == nullptr)
        _gameCamId = _gameCamCtrl.SpawnAuto(sceneId, CameraTag::Game, VGet(6.0f, 4.0f, -8.0f), VGet(0.0f, 0.0f, 0.0f));

    _currentCamId = _debugCamId;
    camMgr.SetRender(_currentCamId);
}

void CollisionScene::End()
{
    ReleaseDemoObjects_();
}

void CollisionScene::Update(float dtSec)
{
    ObjectManager::Instance().UpdateAll(dtSec);
    _debugCamCtrl.SetCamera(_debugCamId);
    _debugCamCtrl.UpdateFreeMoveMouse(8.0f, 0.4f, 10.0f, dtSec);
    UpdateControlledPlayer_(dtSec);
    UpdateGameCamera_();

    auto& key = KeyInput::Instance();
    if (key.IsKeyInputTrigger(KEY_INPUT_B)) _useBlend = !_useBlend;
    if (key.IsKeyInputTrigger(KEY_INPUT_1)) {
        _currentCamId = _debugCamId;
        if (_useBlend) CameraManager::Instance().BlendRenderTo(_currentCamId, _blendSec);
        else           CameraManager::Instance().SetRender(_currentCamId);
    }
    if (key.IsKeyInputTrigger(KEY_INPUT_2)) {
        _currentCamId = _gameCamId;
        if (_useBlend) CameraManager::Instance().BlendRenderTo(_currentCamId, _blendSec);
        else           CameraManager::Instance().SetRender(_currentCamId);
    }
    if (key.IsKeyInputTrigger(KEY_INPUT_R)) {
        ReleaseDemoObjects_();
        SceneManager::Instance().RequestChange(std::make_unique<CollisionScene>());
    }
    if (key.IsKeyInputTrigger(KEY_INPUT_ESCAPE)) {
        ReleaseDemoObjects_();
        SceneTransition::Params p;
        p.mode = SceneTransition::Mode::MaskImage;
        p.durationSec = 0.4f;
        p.maskGraphPath = "Data/Transition/mask.png";
        p.pixelShaderPath = "Data/Transition/mask_transition.pso";
        SceneTransition::Instance().Start(std::make_unique<DemoMenuScene>(), p, 0.5f);
    }
}

void CollisionScene::Draw()
{
    DrawGridFloor_(0.0f, 14, 1.0f);
    ObjectManager::Instance().DrawAll();
    DrawString(10, 10,  "CollisionScene - R:リセット ESC:メニュー", GetColor(255, 255, 255));
    DrawString(10, 30,  "[操作] J/L:I/K:U/O でプレイヤー移動", GetColor(255, 255, 120));
    DrawString(10, 50,  "       Collision/Trigger 判定確認", GetColor(255, 220, 120));
    DrawString(10, 70,  "[カメラ] 1:Debug 2:Game  B:Blend ON/OFF", GetColor(180, 180, 255));
    DrawString(10, 90,  "赤系: Collision  青系: Trigger", GetColor(255, 180, 180));
    DrawString(10, 110, CollisionDebugClass::LastEventText().c_str(), GetColor(180, 255, 180));
    DrawString(10, 130, "Box / Sphere / Capsule / Trigger を配置", GetColor(220, 220, 220));
}

void CollisionScene::RegisterPools_()
{
    static bool s_registered = false;
    if (s_registered) return;
    ObjectFactory::Instance().RegisterCreator("DebugPlayer",  [](const VariantMap&) { return std::make_unique<DebugPlayer>(); });
    ObjectFactory::Instance().RegisterCreator("DebugEnemy",   [](const VariantMap&) { return std::make_unique<DebugEnemy>(); });
    ObjectFactory::Instance().RegisterCreator("DebugHat",     [](const VariantMap&) { return std::make_unique<DebugHat>(); });
    ObjectFactory::Instance().RegisterCreator("DebugGround",  [](const VariantMap&) { return std::make_unique<DebugGround>(); });
    ObjectFactory::Instance().RegisterCreator(CollisionDebugClass::StaticPoolKey(), [](const VariantMap&) { return std::make_unique<CollisionDebugClass>(); });
    ObjectManager::Instance().RegisterPool("DebugPlayer",  4);
    ObjectManager::Instance().RegisterPool("DebugEnemy",   4);
    ObjectManager::Instance().RegisterPool("DebugHat",     4);
    ObjectManager::Instance().RegisterPool("DebugGround",  2);
    ObjectManager::Instance().RegisterPool(CollisionDebugClass::StaticPoolKey(), 16);
    s_registered = true;
}

void CollisionScene::SpawnDemoObjects_()
{
    _player = dynamic_cast<DebugPlayer*>(ObjectManager::Instance().Spawn("DebugPlayer"));
    _enemy  = dynamic_cast<DebugEnemy*>(ObjectManager::Instance().Spawn("DebugEnemy"));
    _hat    = dynamic_cast<DebugHat*>(ObjectManager::Instance().Spawn("DebugHat"));
    _ground = dynamic_cast<DebugGround*>(ObjectManager::Instance().Spawn("DebugGround"));

    _boxObstacle     = SpawnCollisionObject_({ {"name","SolidBox"},    {"shape","box"},     {"static","true"}, {"px","0.0"},  {"py","1.0"}, {"pz","5.0"},  {"hx","0.9"},{"hy","0.9"},{"hz","0.9"}, {"color",std::to_string(GetColor(220,220,220))} });
    _sphereObstacle  = SpawnCollisionObject_({ {"name","SolidSphere"}, {"shape","sphere"},  {"static","true"}, {"px","-4.5"},{"py","1.0"}, {"pz","4.5"},  {"radius","0.8"}, {"color",std::to_string(GetColor(255,220,120))} });
    _capsuleObstacle = SpawnCollisionObject_({ {"name","SolidCapsule"},{"shape","capsule"}, {"static","true"}, {"px","4.5"}, {"py","1.2"}, {"pz","4.0"},  {"radius","0.45"},{"halfHeight","0.95"},{"yaw","0.35"}, {"color",std::to_string(GetColor(180,255,180))} });
    _triggerSphere   = SpawnCollisionObject_({ {"name","TriggerSphere"},{"shape","sphere"},  {"trigger","true"},{"static","true"}, {"layer",std::to_string(layerMask::TRIGGER)}, {"px","0.0"},{"py","1.2"},{"pz","-3.5"}, {"radius","1.4"}, {"color",std::to_string(GetColor(120,140,255))}, {"triggerColor",std::to_string(GetColor(80,180,255))} });

    if (_player) {
        _player->transform.SetLocalPosition(VGet(0.0f, 1.0f, 0.0f));
        if (PhysicsBody* body = _player->GetPhysicsBody()) {
            body->_enabled  = false;
            body->_velocity = VGet(0.0f, 0.0f, 0.0f);
        }
    }
    if (_enemy)  { _enemy->transform.SetLocalPosition(VGet(3.5f, 1.0f, -1.5f)); _enemy->isStatic = true; }
    if (_ground) { _ground->transform.SetLocalPosition(VGet(0.0f, -0.6f, 0.0f)); }
    if (_hat && _player) {
        _hat->transform.SetParent(&_player->transform);
        _hat->transform.SetLocalPosition(VGet(0.35f, 1.0f, 0.35f));
    }
}

void CollisionScene::ReleaseDemoObjects_()
{
    if (_hat)             { _hat->transform.SetParent(nullptr); ObjectManager::Instance().Release(_hat); _hat = nullptr; }
    if (_ground)          { ObjectManager::Instance().Release(_ground); _ground = nullptr; }
    if (_player)          { ObjectManager::Instance().Release(_player); _player = nullptr; }
    if (_enemy)           { ObjectManager::Instance().Release(_enemy);  _enemy  = nullptr; }
    if (_boxObstacle)     { ObjectManager::Instance().Release(_boxObstacle);     _boxObstacle     = nullptr; }
    if (_sphereObstacle)  { ObjectManager::Instance().Release(_sphereObstacle);  _sphereObstacle  = nullptr; }
    if (_capsuleObstacle) { ObjectManager::Instance().Release(_capsuleObstacle); _capsuleObstacle = nullptr; }
    if (_triggerSphere)   { ObjectManager::Instance().Release(_triggerSphere);   _triggerSphere   = nullptr; }
}

void CollisionScene::UpdateControlledPlayer_(float dtSec)
{
    if (!_player) return;
    const float moveSpeed    = 4.8f;
    const float verticalSpeed = 4.0f;
    VECTOR input = VGet(0.0f, 0.0f, 0.0f);
    auto& key = KeyInput::Instance();
    if (key.IsKeyInputHeld(KEY_INPUT_J)) input.x -= 1.0f;
    if (key.IsKeyInputHeld(KEY_INPUT_L)) input.x += 1.0f;
    if (key.IsKeyInputHeld(KEY_INPUT_I)) input.z += 1.0f;
    if (key.IsKeyInputHeld(KEY_INPUT_K)) input.z -= 1.0f;
    if (key.IsKeyInputHeld(KEY_INPUT_U)) input.y += 1.0f;
    if (key.IsKeyInputHeld(KEY_INPUT_O)) input.y -= 1.0f;

    const VECTOR hInput = VGet(input.x, 0.0f, input.z);
    const float hLenSq = hInput.x * hInput.x + hInput.z * hInput.z;
    VECTOR pos = _player->transform.LocalPosition();
    if (hLenSq > 1e-6f) {
        const float inv = 1.0f / std::sqrt(hLenSq);
        pos.x += hInput.x * inv * moveSpeed * dtSec;
        pos.z += hInput.z * inv * moveSpeed * dtSec;
        _player->transform.SetLocalEulerRad(VGet(0.0f, std::atan2(hInput.x, hInput.z), 0.0f));
    }
    if (input.y != 0.0f) pos.y += input.y * verticalSpeed * dtSec;
    _player->transform.SetLocalPosition(pos);
}

void CollisionScene::UpdateGameCamera_()
{
    auto* cam = CameraManager::Instance().Get(_gameCamId);
    if (!cam || !_player) return;
    const VECTOR target = _player->transform.LocalPosition();
    cam->LookAt(VAdd(target, VGet(6.0f, 4.0f, -8.0f)), VAdd(target, VGet(0.0f, 1.0f, 0.0f)));
}

CollisionDebugClass* CollisionScene::SpawnCollisionObject_(const VariantMap& params)
{
    return dynamic_cast<CollisionDebugClass*>(ObjectManager::Instance().Spawn(CollisionDebugClass::StaticPoolKey(), params));
}

void CollisionScene::DrawGridFloor_(float y, int halfCells, float step)
{
    const unsigned int colGrid = GetColor(60, 60, 60);
    for (int i = -halfCells; i <= halfCells; ++i) {
        const float x = i * step;
        DrawLine3D(VGet(x, y, -(float)halfCells * step), VGet(x, y, (float)halfCells * step), colGrid);
        const float z = i * step;
        DrawLine3D(VGet(-(float)halfCells * step, y, z), VGet((float)halfCells * step, y, z), colGrid);
    }
    DrawLine3D(VGet(0, y, 0), VGet(2, y, 0), GetColor(255, 80, 80));
    DrawLine3D(VGet(0, y, 0), VGet(0, y + 2, 0), GetColor(80, 255, 80));
    DrawLine3D(VGet(0, y, 0), VGet(0, y, 2), GetColor(80, 80, 255));
}
