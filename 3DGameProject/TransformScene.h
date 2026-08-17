#pragma once
#include <string>
#include "CameraController.h"
#include "CameraTags.h"
#include "DxLib.h"
#include "SceneTpl.h"
#include "TransformDebugClass.h"
class TransformScene : public SceneTpl<TransformScene> {
public:
    static std::string StaticName() { return "TransformScene"; }
    void Start() override;
    void End() override;
    void Update(float dtSec) override;
    void Draw() override;
private:
    void UpdateParentMotion_();
    static TransformDebugClass* SpawnTransformObject_(const VariantMap& params);
    static void DrawGridFloor_(float y, int halfCells, float step);
private:
    CameraController _cameraController;
    CameraController::CameraId _cameraId = 0;
    TransformDebugClass* _parent      = nullptr;
    TransformDebugClass* _child       = nullptr;
    TransformDebugClass* _worldMarker = nullptr;
    VECTOR _childLocalOffset = { 2.0f, 0.6f, 0.0f };
    float  _driveTime  = 0.0f;
    bool   _isAttached = true;
};