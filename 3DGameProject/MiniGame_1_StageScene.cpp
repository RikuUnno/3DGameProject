#include "MiniGame_1_StageScene.h"

#include <cmath>
#include <algorithm>

#include "DxLib.h"
#include "KeyInput.h"
#include "SceneManager.h"
#include "SceneTransition.h"
#include "CameraManager.h"
#include "MiniGame_1_MenuScene.h"


namespace {
	// グリッド床を描画するユーティリティ関数 (y=指定した高さ, halfCells=半分のセル数, step=セルの間隔)
    inline void DrawGridFloor(float y, int halfCells, float step) {
        const unsigned int colGrid = GetColor(60, 60, 60);
        for (int i = -halfCells; i <= halfCells; ++i) {
            const float x = i * step;
            DrawLine3D(VGet(x, y, -(float)halfCells * step), VGet(x, y, (float)halfCells * step), colGrid);
            const float z = i * step;
            DrawLine3D(VGet(-(float)halfCells * step, y, z), VGet((float)halfCells * step, y, z), colGrid);
        }
        // XYZ軸
        DrawLine3D(VGet(0, y, 0), VGet(3, y, 0), GetColor(255, 80, 80));
        DrawLine3D(VGet(0, y, 0), VGet(0, y + 3, 0), GetColor(80, 255, 80));
        DrawLine3D(VGet(0, y, 0), VGet(0, y, 3), GetColor(80, 80, 255));
    }
}

void MiniGame_1_StageScene::Start()
{
    // 戦闘機の生成・配置
    _aircraft = std::make_unique<FighterAircraft>();
    _aircraft->transform.SetLocalPosition(VGet(0.0f, 5.0f, 0.0f));
    _aircraft->Start();

    // カメラ生成
    const int sceneId = SceneManager::Instance().CurrentSceneId();
    _camId = _camCtrl.SpawnAuto(sceneId, CameraTag::Game, VGet(0.0f, 8.0f, -20.0f));
    CameraManager::Instance().SetRender(_camId);
    _camYaw    = 0.0f;
    _camPitch  = 0.0f;
    _mouseInit = false;
}

void MiniGame_1_StageScene::Update(float dtSec)
{
	// 機体更新
    if (_aircraft) {
        _aircraft->Update(dtSec);
    }

	// 三人称カメラ更新
    UpdateThirdPersonCamera_(dtSec);

	// --- Scene の更新 ---

    // Esc でメニューへ戻る
    if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_ESCAPE)) {
        if (_aircraft) { _aircraft->End(); }

        SceneTransition::Params p;
        p.mode           = SceneTransition::Mode::MaskImage;
        p.durationSec    = 0.6;
        p.maskGraphPath  = "Data/Transition/mask.png";
        p.pixelShaderPath = "Data/Transition/mask_transition.pso";
        SceneTransition::Instance().Start(std::make_unique<MiniGame_1_MenuScene>(), p, 0.5f);
    }
}

void MiniGame_1_StageScene::UpdateThirdPersonCamera_(float dtSec)
{
    if (!_aircraft) return;

    auto* cam = CameraManager::Instance().Get(_camId);
    if (!cam) return;

    // 右クリック中はマウスで手動カメラ回転
    if (GetMouseInput() & MOUSE_INPUT_RIGHT) {
        int mx = 0, my = 0;
        GetMousePoint(&mx, &my);
        if (_mouseInit) {
			const int dx = mx - _prevMouseX;    // マウス移動量
			const int dy = my - _prevMouseY;    // マウス移動量
			_camYaw += dx * 0.005f;             // ヨーは左右そのまま
			_camPitch += dy * 0.004f;           // ピッチは上下逆にする
			_camPitch = (std::max)(-1.2f, (std::min)(0.0f, _camPitch)); // ピッチ制限
        }
        _prevMouseX = mx;
        _prevMouseY = my;
        _mouseInit  = true;
    } else {
        _mouseInit = false;

        const VECTOR fwd = _aircraft->transform.Forward();

        // ヨー角追従
        const float targetYaw = std::atan2f(fwd.x, fwd.z);
        float diffYaw = targetYaw - _camYaw;
        while (diffYaw >  DX_PI_F) diffYaw -= 2.0f * DX_PI_F;
        while (diffYaw < -DX_PI_F) diffYaw += 2.0f * DX_PI_F;

        // ピッチ角追従（機体の前方ベクトルのY成分から算出）
        const float targetPitch = std::asinf(
            (std::max)(-1.0f, (std::min)(1.0f, -fwd.y)));
        float diffPitch = targetPitch - _camPitch;
        while (diffPitch >  DX_PI_F) diffPitch -= 2.0f * DX_PI_F;
        while (diffPitch < -DX_PI_F) diffPitch += 2.0f * DX_PI_F;

        const float t = (std::min)(1.0f, 3.0f * dtSec);
        _camYaw   += diffYaw   * t;
        _camPitch += diffPitch * t;
        _camPitch  = (std::max)(-1.2f, (std::min)(1.2f, _camPitch));
    }

    // スクロールホイールで距離調整
    _camDistance -= GetMouseWheelRotVol() * 1.5f;
    _camDistance = (std::max)(5.0f, (std::min)(60.0f, _camDistance));

    // 球面座標でカメラ位置を計算し機体に追従
    const VECTOR target = _aircraft->transform.WorldPosition();
    const float cosP = std::cos(_camPitch);
    const float sinP = std::sin(_camPitch);
    const float cosY = std::cos(_camYaw);
    const float sinY = std::sin(_camYaw);
    const VECTOR offset = VGet(
        sinY * cosP * _camDistance,
        -sinP * _camDistance + _camHeight,
        cosY * cosP * _camDistance
    );
    const VECTOR camPos = VAdd(target, offset);

    cam->transform.SetLocalPosition(camPos);
    cam->LookAt(camPos, target, VGet(0.0f, 1.0f, 0.0f));
}

void MiniGame_1_StageScene::End()
{
    if (_aircraft) {
        _aircraft->End();
        _aircraft.reset();
    }
}

void MiniGame_1_StageScene::Draw()
{
	// グリッド床の描画
    DrawGridFloor(0.0f, 80, 4.0f);

    if (_aircraft) {
        _aircraft->Draw();
    }

    DrawString(10, 10, "MiniGame_1 - 戦闘機飛行", GetColor(255, 255, 120));
    DrawString(10, 30, "W/S: ピッチ  A/D: ヨー  Q/E: ロール", GetColor(255, 255, 255));
    DrawString(10, 50, "Space: 加速  LShift: 減速", GetColor(255, 255, 255));
    DrawString(10, 70, "右クリック+ドラッグ: カメラ回転  ホイール: 距離調整", GetColor(200, 200, 255));
    DrawString(10, 90, "Esc: メニューへ戻る", GetColor(180, 255, 180));
}

