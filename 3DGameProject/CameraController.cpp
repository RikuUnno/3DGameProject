#include "CameraController.h"

#include "KeyInput.h"
#include "Time.h"
#include "DxLib.h"

CameraController::CameraId CameraController::SpawnAuto(
	int ownerSceneId,
	CameraTag tag,
	const VECTOR& pos,
	const VECTOR& eulerRad,
	float fovYRad,
	float nearZ,
	float farZ
) {
	auto& mgr = CameraManager::Instance();

	// 以前のIDを「再利用」する前に、CameraManager に実体があるか確認する
	if (_cameraId != 0) {
		if (mgr.Get(_cameraId) != nullptr) {
			_registeredIds.insert(_cameraId);
			return _cameraId;
		}

		// ここに来たら「IDは保持しているが実体がない」= scene切替等で破棄済み
		_registeredIds.erase(_cameraId);
		_cameraId = 0;
	}

	const CameraId id = mgr.CreateCamera(ownerSceneId);
	auto* cam = mgr.Get(id);
	if (cam) {
		cam->tag = tag;
		cam->transform.SetLocalPosition(pos);
		cam->transform.SetLocalEulerRad(eulerRad);
		cam->fovYRad = fovYRad;
		cam->nearZ = nearZ;
		cam->farZ = farZ;
		cam->MarkDirty();
	}

	_registeredIds.insert(id);
	_cameraId = id;
	return id;
}

void CameraController::UpdateFreeMove(float moveSpeed, float rotSpeed) {
	auto* cam = CameraManager::Instance().Get(_cameraId);
	if (!cam) return;

	const float dt = (float)Time::Instance().GetDeltaTime();

	// 回転入力（矢印）
	// - Eulerで入力するが、Transform内部はQuaternionへ変換される
	VECTOR e = cam->transform.LocalEulerRad();
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_LEFT)) e.y -= rotSpeed * dt;
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_RIGHT)) e.y += rotSpeed * dt;
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_UP)) e.x -= rotSpeed * dt;
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_DOWN)) e.x += rotSpeed * dt;
	cam->transform.SetLocalEulerRad(e);

	// 移動：カメラのローカル軸で移動
	VECTOR p = cam->transform.LocalPosition();
	const VECTOR f = cam->transform.Forward();
	const VECTOR r = cam->transform.Right();
	const VECTOR u = cam->transform.Up();

	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_W)) p = VAdd(p, VScale(f, moveSpeed * dt));
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_S)) p = VAdd(p, VScale(f, -moveSpeed * dt));
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_D)) p = VAdd(p, VScale(r, moveSpeed * dt));
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_A)) p = VAdd(p, VScale(r, -moveSpeed * dt));
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_E)) p = VAdd(p, VScale(u, moveSpeed * dt));
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_Q)) p = VAdd(p, VScale(u, -moveSpeed * dt));

	cam->transform.SetLocalPosition(p);
	cam->MarkDirty();
}

void CameraController::UpdateFreeMoveQuat(float moveSpeed, float rotSpeed) {
	auto* cam = CameraManager::Instance().Get(_cameraId);
	if (!cam) return;

	const float dt = (float)Time::Instance().GetDeltaTime();

	// 回転：Quaternionを直接合成する
	// - yaw: ワールドUp(0,1,0) を軸として回す（左/右）
	// - pitch: 現在のカメラRight軸を基準に回す（上/下）
	Quaternion q = cam->transform.LocalRotation();

	const float yawDelta = rotSpeed * dt;
	const float pitchDelta = rotSpeed * dt;

	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_LEFT)) {
		q = Quaternion::FromAxisAngleRad(VGet(0,1,0), -yawDelta) * q;
	}
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_RIGHT)) {
		q = Quaternion::FromAxisAngleRad(VGet(0,1,0), yawDelta) * q;
	}

	// pitchだけは「現在の右軸」を使う（ローカル回転）
	const VECTOR right = VNorm(q.RotateVector(VGet(1,0,0)));
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_UP)) {
		q = Quaternion::FromAxisAngleRad(right, -pitchDelta) * q;
	}
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_DOWN)) {
		q = Quaternion::FromAxisAngleRad(right, pitchDelta) * q;
	}

	cam->transform.SetLocalRotation(q);

	// 移動：カメラのローカル軸で移動
	VECTOR p = cam->transform.LocalPosition();
	const VECTOR f = cam->transform.Forward();
	const VECTOR r = cam->transform.Right();
	const VECTOR u = cam->transform.Up();

	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_W)) p = VAdd(p, VScale(f, moveSpeed * dt));
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_S)) p = VAdd(p, VScale(f, -moveSpeed * dt));
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_D)) p = VAdd(p, VScale(r, moveSpeed * dt));
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_A)) p = VAdd(p, VScale(r, -moveSpeed * dt));
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_E)) p = VAdd(p, VScale(u, moveSpeed * dt));
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_Q)) p = VAdd(p, VScale(u, -moveSpeed * dt));

	cam->transform.SetLocalPosition(p);
	cam->MarkDirty();
}

namespace {
	// 現在の回転(q)のピッチ角を推定する
	// 前方ベクトルから pitch = asin(-y) を用いる（forwardが(0,0,1)基準）
	inline float EstimatePitchRadFromQuat(const Quaternion& q) noexcept {
		const VECTOR f = VNorm(q.RotateVector(VGet(0,0,1)));
		// f.y は上向きが+。 pitchを上向きで負にしている流儀もあるが、ここでは一貫性より「制限」目的。
		// 上を見るほど pitch が +になるように定義（必要なら符号を反転してもよい）。
		return asinf(std::clamp(f.y, -1.0f,1.0f));
	}
}

void CameraController::UpdateFreeMoveQuatClamped(float moveSpeed, float rotSpeed, float pitchMinRad, float pitchMaxRad) {
	auto* cam = CameraManager::Instance().Get(_cameraId);
	if (!cam) return;

	const float dt = (float)Time::Instance().GetDeltaTime();

	Quaternion q = cam->transform.LocalRotation();

	const float yawDelta = rotSpeed * dt;
	const float pitchDelta = rotSpeed * dt;

	// yaw（ワールドUp軸）
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_LEFT)) {
		q = Quaternion::FromAxisAngleRad(VGet(0,1,0), -yawDelta) * q;
	}
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_RIGHT)) {
		q = Quaternion::FromAxisAngleRad(VGet(0,1,0), yawDelta) * q;
	}

	// pitch（現在のRight軸） + 制限
	const VECTOR right = VNorm(q.RotateVector(VGet(1,0,0)));
	const float pitchNow = EstimatePitchRadFromQuat(q);

	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_UP)) {
		// 上を向く方向へ回す
		if (pitchNow < pitchMaxRad) {
			q = Quaternion::FromAxisAngleRad(right, pitchDelta) * q;
		}
	}
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_DOWN)) {
		// 下を向く方向へ回す
		if (pitchNow > pitchMinRad) {
			q = Quaternion::FromAxisAngleRad(right, -pitchDelta) * q;
		}
	}

	cam->transform.SetLocalRotation(q);

	// 移動：カメラのローカル軸で移動
	VECTOR p = cam->transform.LocalPosition();
	const VECTOR f = cam->transform.Forward();
	const VECTOR r = cam->transform.Right();
	const VECTOR u = cam->transform.Up();

	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_W)) p = VAdd(p, VScale(f, moveSpeed * dt));
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_S)) p = VAdd(p, VScale(f, -moveSpeed * dt));
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_D)) p = VAdd(p, VScale(r, moveSpeed * dt));
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_A)) p = VAdd(p, VScale(r, -moveSpeed * dt));
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_E)) p = VAdd(p, VScale(u, moveSpeed * dt));
	if (KeyInput::Instance().IsKeyInputHeld(KEY_INPUT_Q)) p = VAdd(p, VScale(u, -moveSpeed * dt));

	cam->transform.SetLocalPosition(p);
	cam->MarkDirty();
}
