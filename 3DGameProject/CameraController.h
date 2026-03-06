#pragma once

#include <cstdint>
#include <unordered_set>

#include "CameraManager.h"
#include "CameraTags.h"

// CameraController
// - ObjectController に寄せた「単一コントローラ」例。対象IDを保持し、Updateで操作する
// - 本体(Camera)は CameraManager が所有
class CameraController {
public:
	using CameraId = CameraManager::CameraId;

	// 操作対象をセット
	void SetCamera(CameraId id) noexcept { _cameraId = id; }
	CameraId CameraIdValue() const noexcept { return _cameraId; }

	// ObjectController::SpawnAuto 相当
	// - 未作成なら CameraManager にカメラを作成し、初期値をセットして返す
	// -既に作成済みなら「そのIDを操作対象にする」だけ
	CameraId SpawnAuto(
		int ownerSceneId,
		CameraTag tag = CameraTag::Game,
		const VECTOR& pos = VGet(0,0,-10),
		const VECTOR& eulerRad = VGet(0,0,0),
		float fovYRad =60.0f * DX_PI_F /180.0f,
		float nearZ =0.1f,
		float farZ =1000.0f
	);

	// シンプルな操作: dt秒ぶん更新
	void UpdateFreeMove(float moveSpeed, float rotSpeed, float dtSec);

	// フリームーブ（Quaternionで回転を直接保持する版）
	// - Eulerの直積弊害で「意図しない回転」が起きづらい
	// - rotSpeed: rad/sec
	void UpdateFreeMoveQuat(float moveSpeed, float rotSpeed, float dtSec);

	// フリームーブ（Quaternion版 + ピッチ制限）
	// - pitchMin/pitchMax: ラジアン（例: -DX_PI_F/2+0.1 ～ +DX_PI_F/2-0.1）
	void UpdateFreeMoveQuatClamped(float moveSpeed, float rotSpeed, float pitchMinRad, float pitchMaxRad, float dtSec);

	// マウス対応フリームーブ（デバッグカメラ想定）
	// -右ボタン押下中に視点回転
	// - ホイールで前後移動
	// - WASD/EQで移動
	void UpdateFreeMoveMouse(float moveSpeed, float rotSpeed, float wheelMoveSpeed, float dtSec);

	// Renderカメラを targetへ補間しながら切替
	bool BlendRenderTo(CameraId targetId, float durationSec) {
		return CameraManager::Instance().BlendRenderTo(targetId, durationSec);
	}

private:
	CameraId _cameraId =0;
	std::unordered_set<CameraId> _registeredIds;
};
