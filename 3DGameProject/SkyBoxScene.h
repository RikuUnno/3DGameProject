#pragma once

#include <memory>
#include <string>

#include "SceneTpl.h"
#include "SkyBox.h"
#include "CameraController.h"
#include "CameraManager.h"
#include "CameraTags.h"
#include "DxLib.h"
#include "KeyInput.h"
#include "SceneManager.h"
#include "SceneTransition.h"
#include "TitleScene.h"

// SkyBoxScene
// - SkyBox の動作確認用シーン
// - WASDQE + 右ドラッグでフリーカメラ操作 / T でタイトルへ戻る
class SkyBoxScene : public SceneTpl<SkyBoxScene> {
public:
    static std::string StaticName() { return "SkyBoxScene"; }

    void Start() override {
        auto& cm = CameraManager::Instance();
        const int sceneId = SceneManager::Instance().CurrentSceneId();
        if (_camId == 0 || cm.Get(_camId) == nullptr) {
            _camId = _camCtrl.SpawnAuto(sceneId, CameraTag::Debug,
                VGet(0.0f, 1.5f, -5.0f), VGet(0.0f, 1.0f, 0.0f));
        }
        cm.SetRender(_camId);

        // 6 面テクスチャを読み込み（存在しなければグレーで描画されるだけ）
        // ファイル名は一般的な命名 (right/left/top/bottom/front/back) を仮定
        _skybox.SetSize(500.0f);
        _skybox.Load("Data/SkyBox/", {
            "right.png",
            "left.png",
            "top.png",
            "bottom.png",
            "front.png",
            "back.png"
        });
    }

    void Update(float dtSec) override {
        _camCtrl.SetCamera(_camId);
        _camCtrl.UpdateFreeMoveMouse(10.0f, 0.45f, 8.0f, dtSec);

        if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_T)) {
            SceneTransition::Params p;
            p.mode = SceneTransition::Mode::MaskImage;
            p.durationSec = 0.4;
            p.maskGraphPath = "Data/Transition/mask.png";
            p.pixelShaderPath = "Data/Transition/mask_transition.pso";
            SceneTransition::Instance().Start(std::make_unique<TitleScene>(), p, 0.5f);
        }
    }

    void Draw() override {
        // SkyBox は最初に描画（Z 書き込み無効）
        if (auto* cam = CameraManager::Instance().Render()) {
            _skybox.Draw(cam->transform.LocalPosition());
        }

        // 確認用にグリッド地面を 1 枚描画
        const unsigned int col = GetColor(120, 120, 120);
        for (int i = -10; i <= 10; ++i) {
            DrawLine3D(VGet((float)i, 0, -10), VGet((float)i, 0, 10), col);
            DrawLine3D(VGet(-10, 0, (float)i), VGet(10, 0, (float)i), col);
        }

        DrawString(10, 10, "SkyBoxScene  T:Title", GetColor(255, 255, 255));
    }

    void End() override {
        _skybox.Reset();
    }

private:
    SkyBox _skybox;
    CameraController _camCtrl;
    CameraController::CameraId _camId = 0;
};
