#pragma once

#include <memory>
#include <string>

#include "SceneTpl.h"
#include "CameraController.h"
#include "CameraModelObject.h"

class CameraScene : public SceneTpl<CameraScene>
{
public:
    static std::string StaticName() { return "CameraScene"; }
    void Start() override;
    void Update(float dtSec) override;
    void Draw() override;
private:
    void UpdateCircusModelInput_(float dtSec);
    void SwitchRenderCamera_(CameraController::CameraId targetId);
    void UpdateDemoCameras_(float dtSec);
private:
    CameraController _freeCamCtrl;
    CameraController _followCamCtrl;
    CameraController _orbitCamCtrl;
    CameraController _topCamCtrl;
    CameraController::CameraId _freeCamId    = 0;
    CameraController::CameraId _followCamId  = 0;
    CameraController::CameraId _orbitCamId   = 0;
    CameraController::CameraId _topCamId     = 0;
    CameraController::CameraId _currentCamId = 0;
    std::unique_ptr<CameraModelObject> _ownedCircusModel;
    CameraModelObject* _circusModel = nullptr;
    bool  _useBlend   = true;
    float _blendSec   = 0.5f;
    float _cameraTime = 0.0f;
};
