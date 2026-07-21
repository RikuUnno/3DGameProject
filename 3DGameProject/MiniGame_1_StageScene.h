#pragma once

#include "SceneTpl.h"
#include "FighterAircraft.h"
#include "CameraController.h"
#include <memory>

// MiniGame_1 のゲームプレイ本体シーン（戦闘機飛行）
class MiniGame_1_StageScene : public SceneTpl<MiniGame_1_StageScene>
{
public:
    static std::string StaticName() { return "MiniGame_1_StageScene"; }

    void Start() override;
    void Update(float dtSec) override;
    void Draw() override;
    void End() override;

private:
    void UpdateThirdPersonCamera_(float dtSec);	// 三人称カメラ更新

    std::unique_ptr<FighterAircraft> _aircraft;	// 機体
    CameraController _camCtrl;					// カメラコントローラ
    CameraController::CameraId _camId = 0;		// カメラID

    // 三人称カメラのオフセット
    float _camDistance  = 30.0f;  // 機体後方への距離
    float _camHeight    = 5.0f;   // 機体上方への高さ
    float _camYaw       = 0.0f;   // 水平旋回角（ラジアン）
    float _camPitch     = 0.0f;   // 仰俯角（ラジアン）
    int   _prevMouseX   = 0;      // 前フレームのマウスX座標
    int   _prevMouseY   = 0;      // 前フレームのマウスY座標
    bool  _mouseInit    = false;  // マウス初期化フラグ
};
