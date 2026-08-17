#pragma once
#include <string>
#include "SceneTpl.h"
#include "SkyBox.h"
#include "CameraController.h"
#include "CameraTags.h"
class SkyBoxScene : public SceneTpl<SkyBoxScene> {
public:
    static std::string StaticName() { return "SkyBoxScene"; }
    void Start() override;
    void Update(float dtSec) override;
    void Draw() override;
    void End() override;
private:
    SkyBox _skybox;
    CameraController _camCtrl;
    CameraController::CameraId _camId = 0;
};