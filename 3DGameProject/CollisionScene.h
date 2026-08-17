#pragma once
#include <string>
#include "CameraController.h"
#include "CameraTags.h"
#include "CollisionDebugClass.h"
#include "Debug Class.h"
#include "SceneTpl.h"
#include "LayerMask.h"
class CollisionScene : public SceneTpl<CollisionScene> {
public:
    static std::string StaticName() { return "CollisionScene"; }
    void Start() override;
    void End() override;
    void Update(float dtSec) override;
    void Draw() override;
private:
    void RegisterPools_();
    void SpawnDemoObjects_();
    void ReleaseDemoObjects_();
    void UpdateControlledPlayer_(float dtSec);
    void UpdateGameCamera_();
    static CollisionDebugClass* SpawnCollisionObject_(const VariantMap& params);
    static void DrawGridFloor_(float y, int halfCells, float step);
private:
    CameraController _debugCamCtrl;
    CameraController _gameCamCtrl;
    CameraController::CameraId _debugCamId   = 0;
    CameraController::CameraId _gameCamId    = 0;
    CameraController::CameraId _currentCamId = 0;
    bool  _useBlend  = true;
    float _blendSec  = 0.4f;
    DebugPlayer*         _player          = nullptr;
    DebugEnemy*          _enemy           = nullptr;
    DebugHat*            _hat             = nullptr;
    DebugGround*         _ground          = nullptr;
    CollisionDebugClass* _boxObstacle     = nullptr;
    CollisionDebugClass* _sphereObstacle  = nullptr;
    CollisionDebugClass* _capsuleObstacle = nullptr;
    CollisionDebugClass* _triggerSphere   = nullptr;
};