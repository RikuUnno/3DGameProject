#pragma once

#include "SceneTpl.h"
#include "FighterAircraft.h"
#include "CameraController.h"
#include <memory>

class MiniGame_1_MenuScene : public SceneTpl<MiniGame_1_MenuScene>
{
public:
	static std::string StaticName() { return "MiniGame_1_MenuScene"; }

	void Start() override;
	void Update(float dtSec) override;
	void Draw() override;
	void End() override;

private:
	void UpdateThirdPersonCamera_(float dtSec);	// 三人称カメラ更新

	std::unique_ptr<FighterAircraft> _aircraft;			// 機体
	CameraController _camCtrl;							// カメラコントローラ
	CameraController::CameraId _camId = 0;				// カメラID

	// 三人称カメラのオフセット
	float _camDistance = 20.0f;	// 機体後方への距離
	float _camHeight = 6.0f;		// 機体上方への高さ
	float _camYaw = 0.0f;			// 水平旋回角（ラジアン）
	float _camPitch = -0.25f;		// 仰俯角（ラジアン）
	int _prevMouseX = 0;			// 前フレームのマウスX座標
	int _prevMouseY = 0;			// 前フレームのマウスY座標
	bool _mouseInit = false;		// マウス初期化フラグ
};