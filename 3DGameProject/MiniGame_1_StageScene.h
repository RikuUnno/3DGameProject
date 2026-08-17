#pragma once

#include "SceneTpl.h"
#include "FighterAircraft.h"
#include "EnemyAircraft.h"
#include "EnemyDifficulty.h"
#include "CameraController.h"
#include "HudDisplay.h"
#include <memory>
#include <vector>

// MiniGame_1 のゲームプレイ本体シーン（戦闘機飛行）
class MiniGame_1_StageScene : public SceneTpl<MiniGame_1_StageScene>
{
public:
    static std::string StaticName() { return "MiniGame_1_StageScene"; }

    void Start() override;
    void Update(float dtSec) override;
    void Draw() override;
    void End() override;

    // 敵AIの難易度を外部から設定（メニュー等から呼ぶ想定）
    void SetDifficulty(EnemyDifficulty difficulty) noexcept { _difficulty = difficulty; }

private:
    void UpdateThirdPersonCamera_(float dtSec);	// 三人称カメラ更新
    void ReturnToMenu_();							// メニューシーンへ戻る（Esc / 体力ゼロ共通処理）

    std::shared_ptr<FighterAircraft> _aircraft;	// 機体（EnemyAircraft から weak_ptr で参照されるため shared_ptr で所有）
    std::vector<std::unique_ptr<EnemyAircraft>> _enemies;	// 敵機体一覧
    EnemyDifficulty _difficulty = EnemyDifficulty::Normal;	// 敵AIの難易度
    bool _returningToMenu = false;	// メニューへの遷移が開始済みか（多重遷移防止）
    CameraController _camCtrl;					// カメラコントローラ
    CameraController::CameraId _camId = 0;		// カメラID
    HudDisplay _hud;								// 右下ステータスHUD（体力・弾数等）

    // 三人称カメラのオフセット
    float _camDistance  = 30.0f;  // 機体後方への距離
    float _camHeight    = 5.0f;   // 機体上方への高さ
    float _camYaw       = 0.0f;   // 水平旋回角（ラジアン）
    float _camPitch     = 0.0f;   // 仰俯角（ラジアン）
    int   _prevMouseX   = 0;      // 前フレームのマウスX座標
    int   _prevMouseY   = 0;      // 前フレームのマウスY座標
    bool  _mouseInit    = false;  // マウス初期化フラグ
};
