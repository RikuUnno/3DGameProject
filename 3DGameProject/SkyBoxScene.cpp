#include "SkyBoxScene.h"

#include <array>
#include <string>
#include <memory>

#include "CameraManager.h"
#include "DxLib.h"
#include "KeyInput.h"
#include "SceneManager.h"
#include "SceneTransition.h"
#include "DemoMenuScene.h"

void SkyBoxScene::Start()
{
    auto& cm = CameraManager::Instance();
    const int sceneId = SceneManager::Instance().CurrentSceneId();
    if (_camId == 0 || cm.Get(_camId) == nullptr)
        _camId = _camCtrl.SpawnAuto(sceneId, CameraTag::Debug, VGet(0.0f, 1.5f, -5.0f), VGet(0.0f, 1.0f, 0.0f));
    cm.SetRender(_camId);

    _skybox.SetSize(500.0f);
    const std::array<std::string, 6> names = { "+X.png", "-X.png", "+Y.png", "-Y.png", "+Z.png", "-Z.png" };
    const std::array<std::string, 6> dirs = {
        "models/SkyBox/",
        "./models/SkyBox/",
        "../models/SkyBox/",
        "3DGameProject/models/SkyBox/",
        "../3DGameProject/models/SkyBox/",
        "../../3DGameProject/models/SkyBox/",
    };
    for (const auto& d : dirs)
        if (_skybox.Load(d, names)) break;
}

void SkyBoxScene::Update(float dtSec)
{
    _camCtrl.SetCamera(_camId);
    _camCtrl.UpdateFreeMoveMouse(10.0f, 0.45f, 8.0f, dtSec);

    if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_ESCAPE)) {
        SceneTransition::Params p;
        p.mode = SceneTransition::Mode::MaskImage;
        p.durationSec = 0.4;
        p.maskGraphPath   = "Data/Transition/mask.png";
        p.pixelShaderPath = "Data/Transition/mask_transition.pso";
        SceneTransition::Instance().Start(std::make_unique<DemoMenuScene>(), p, 0.5f);
    }
}

void SkyBoxScene::Draw()
{
    if (auto* cam = CameraManager::Instance().Render())
        _skybox.Draw(cam->transform.LocalPosition());

    const unsigned int col = GetColor(120, 120, 120);
    for (int i = -10; i <= 10; ++i) {
        DrawLine3D(VGet((float)i, 0, -10), VGet((float)i, 0, 10), col);
        DrawLine3D(VGet(-10, 0, (float)i), VGet(10, 0, (float)i), col);
    }
    DrawString(10, 10, "SkyBoxScene  ESC:ƒƒjƒ…[", GetColor(255, 255, 255));
}

void SkyBoxScene::End()
{
    _skybox.Reset();
}
