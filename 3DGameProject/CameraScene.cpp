#include "CameraScene.h"

#include <cmath>
#include <memory>
#include <string>

#include "CameraModelObject.h"
#include "CameraManager.h"
#include "DxLib.h"
#include "KeyInput.h"
#include "ObjectFactory.h"
#include "ObjectManager.h"
#include "SceneManager.h"
#include "SceneTransition.h"
#include "DemoMenuScene.h"

void CameraScene::Start()
{
    auto& cameraManager = CameraManager::Instance();
    const int sceneId = SceneManager::Instance().CurrentSceneId();

    static bool s_registered = false;
    if (!s_registered) {
        ObjectFactory::Instance().RegisterCreator(CameraModelObject::StaticPoolKey(), [](const VariantMap&) { return std::make_unique<CameraModelObject>(); });
        ObjectManager::Instance().RegisterPool(CameraModelObject::StaticPoolKey(), 4);
        s_registered = true;
    }

    _ownedCircusModel = std::make_unique<CameraModelObject>();
    VariantMap circusParams{
        {"modelPath", "models/chicken-gun-fruzer-circus/source/circus.mv1"},
        {"px", "13.5"}, {"py", "10.0"}, {"pz", "5.0"},
        {"scale", "0.003"}
    };
    _ownedCircusModel->OnAcquire(circusParams);
    _circusModel = _ownedCircusModel.get();

    if (_freeCamId == 0 || cameraManager.Get(_freeCamId) == nullptr)
        _freeCamId = _freeCamCtrl.SpawnAuto(sceneId, CameraTag::Debug, VGet(0.0f, 5.0f, -10.0f), VGet(0.0f, 0.0f, 0.0f));
    if (_followCamId == 0 || cameraManager.Get(_followCamId) == nullptr)
        _followCamId = _followCamCtrl.SpawnAuto(sceneId, CameraTag::Game, VGet(0.0f, 0.0f, 0.0f), VGet(0.0f, 0.0f, 0.0f));
    if (_orbitCamId == 0 || cameraManager.Get(_orbitCamId) == nullptr)
        _orbitCamId = _orbitCamCtrl.SpawnAuto(sceneId, CameraTag::Game, VGet(0.0f, 3.0f, -10.0f), VGet(0.0f, 0.0f, 0.0f));
    if (_topCamId == 0 || cameraManager.Get(_topCamId) == nullptr)
        _topCamId = _topCamCtrl.SpawnAuto(sceneId, CameraTag::Game, VGet(0.0f, 7.0f, -14.0f), VGet(0.0f, 0.0f, 0.0f));

    _cameraTime   = 0.0f;
    _currentCamId = _freeCamId;
    cameraManager.SetRender(_currentCamId);
}

void CameraScene::Update(float dtSec)
{
    UpdateDemoCameras_(dtSec);
    UpdateCircusModelInput_(dtSec);

    _freeCamCtrl.SetCamera(_freeCamId);
    _freeCamCtrl.UpdateFreeMoveMouse(10.0f, 0.45f, 8.0f, dtSec);

    auto& key = KeyInput::Instance();
    if (key.IsKeyInputTrigger(KEY_INPUT_B)) _useBlend = !_useBlend;
    if (key.IsKeyInputTrigger(KEY_INPUT_1)) SwitchRenderCamera_(_freeCamId);
    if (key.IsKeyInputTrigger(KEY_INPUT_2)) SwitchRenderCamera_(_followCamId);
    if (key.IsKeyInputTrigger(KEY_INPUT_3)) SwitchRenderCamera_(_orbitCamId);
    if (key.IsKeyInputTrigger(KEY_INPUT_4)) SwitchRenderCamera_(_topCamId);
    if (key.IsKeyInputTrigger(KEY_INPUT_R)) SceneManager::Instance().RequestChange(std::make_unique<CameraScene>());
    if (key.IsKeyInputTrigger(KEY_INPUT_ESCAPE)) {
        SceneTransition::Params p;
        p.mode = SceneTransition::Mode::MaskImage;
        p.durationSec = 0.4;
        p.maskGraphPath = "Data/Transition/mask.png";
        p.pixelShaderPath = "Data/Transition/mask_transition.pso";
        SceneTransition::Instance().Start(std::make_unique<DemoMenuScene>(), p, 0.5f);
    }
}

void CameraScene::Draw()
{
    if (_ownedCircusModel && _ownedCircusModel->IsActive())
        _ownedCircusModel->Draw();

    DrawString(10, 10,  "CameraScene - ESC:メニュー R:リセット", GetColor(255, 255, 255));
    DrawString(10, 30,  "1:フリーカメラ", GetColor(255, 220, 140));
    DrawString(10, 50,  "2:位置(0,0,0)で自動旋回(5秒/1周)", GetColor(255, 220, 140));
    DrawString(10, 70,  "3:Offset10で原点オービットカメラ", GetColor(255, 220, 140));
    DrawString(10, 90,  "4:高速で原点を追従しカメラ3を注視", GetColor(255, 220, 140));
    DrawString(10, 110, "B : ブレンド ON/OFF", GetColor(180, 255, 180));
    DrawString(10, 130, "CircusModel Move: Arrow(XZ) / PgUp PgDn(Y)", GetColor(180, 255, 180));
    if (_circusModel && !_circusModel->IsModelLoaded())
        DrawString(10, 150, "Circus model load failed. Check path: models/circus.mv1", GetColor(255, 120, 120));
}

void CameraScene::UpdateCircusModelInput_(float dtSec)
{
    if (!_circusModel) return;
    VECTOR p = _circusModel->transform.LocalPosition();
    const float spd = 4.0f;
    auto& key = KeyInput::Instance();
    if (key.IsKeyInputHeld(KEY_INPUT_LEFT))  p.x -= spd * dtSec;
    if (key.IsKeyInputHeld(KEY_INPUT_RIGHT)) p.x += spd * dtSec;
    if (key.IsKeyInputHeld(KEY_INPUT_UP))    p.z += spd * dtSec;
    if (key.IsKeyInputHeld(KEY_INPUT_DOWN))  p.z -= spd * dtSec;
    if (key.IsKeyInputHeld(KEY_INPUT_PGUP))  p.y += spd * dtSec;
    if (key.IsKeyInputHeld(KEY_INPUT_PGDN))  p.y -= spd * dtSec;
    _circusModel->transform.SetLocalPosition(p);
}

void CameraScene::SwitchRenderCamera_(CameraController::CameraId targetId)
{
    _currentCamId = targetId;
    if (_useBlend) CameraManager::Instance().BlendRenderTo(_currentCamId, _blendSec);
    else           CameraManager::Instance().SetRender(_currentCamId);
}

void CameraScene::UpdateDemoCameras_(float dtSec)
{
    auto& cameraManager = CameraManager::Instance();
    const VECTOR origin = VGet(0.0f, 0.0f, 0.0f);
    const float omega = (2.0f * DX_PI_F) / 5.0f;
    _cameraTime += dtSec;

    // カメラ2: 位置(0,0,0)固定、向きを5秒で1周
    if (Camera* cam = cameraManager.Get(_followCamId)) {
        const float a = omega * _cameraTime;
        cam->LookAt(origin, VGet(std::cos(a), 0.0f, std::sin(a)), VGet(0, 1, 0));
    }
    // カメラ3: Offset=10で原点オービット
    VECTOR cam3Eye = VGet(0.0f, 3.0f, -10.0f);
    if (Camera* cam = cameraManager.Get(_orbitCamId)) {
        const float a = omega * _cameraTime + DX_PI_F * 0.5f;
        cam3Eye = VGet(std::cos(a) * 10.0f, 3.0f, std::sin(a) * 10.0f);
        cam->LookAt(cam3Eye, origin, VGet(0, 1, 0));
    }
    // カメラ4: 高速で原点を追従しカメラ3を注視
    if (Camera* cam = cameraManager.Get(_topCamId)) {
        const float a = omega * _cameraTime + DX_PI_F;
        const VECTOR eye = VGet(std::cos(a) * 16.0f, 18.0f, std::sin(a) * 16.0f);
        cam->LookAt(eye, cam3Eye, VGet(0, 1, 0));
    }
}
