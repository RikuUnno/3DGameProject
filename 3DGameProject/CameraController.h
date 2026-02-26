#pragma once

#include <cstdint>
#include <unordered_set>

#include "CameraManager.h"
#include "CameraTags.h"

// CameraController
// - ObjectController に寄せた「このコントローラが触る対象を保持し、Updateで操作」方式
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

	// シンプルな操作例: 毎フレーム呼ぶ
	void UpdateFreeMove(float moveSpeed, float rotSpeed);

	// フリームーブ（Quaternionで回転を直接合成する版）
	// - Eulerの特異点付近で「がくがく」しづらい
	// - rotSpeed: rad/sec
	void UpdateFreeMoveQuat(float moveSpeed, float rotSpeed);

	// フリームーブ（Quaternion版 + ピッチ制限）
	// - pitchMin/pitchMax: ラジアン（例: -DX_PI_F/2+0.1 ～ +DX_PI_F/2-0.1）
	void UpdateFreeMoveQuatClamped(float moveSpeed, float rotSpeed, float pitchMinRad, float pitchMaxRad);

	// マウス対応フリームーブ（デバッグカメラ向け）
	// -右ボタン押下中に視点回転
	// - ホイールで前後移動
	// - WASD/EQで移動
	void UpdateFreeMoveMouse(float moveSpeed, float rotSpeed, float wheelMoveSpeed =4.0f);

	// Renderカメラを targetへ補間しながら切替
	bool BlendRenderTo(CameraId targetId, float durationSec) {
		return CameraManager::Instance().BlendRenderTo(targetId, durationSec);
	}

private:
	CameraId _cameraId =0;
	std::unordered_set<CameraId> _registeredIds;
};
