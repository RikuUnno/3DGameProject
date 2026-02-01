#pragma once

#include "DxLib.h"
#include "Transform.h"
#include "CameraTags.h"

class Camera {
public:
	Transform transform;

	// 確定値（Render時に反映）
	float fovYRad =60.0f * DX_PI_F /180.0f;
	float nearZ =0.1f;
	float farZ =1000.0f;

	// tag/layer 的な分類（最小）
	CameraTag tag = CameraTag::Game;
	int ownerSceneId = -1;

	const MATRIX& ViewMatrix() const;
	void MarkDirty() noexcept;

	// LookAt を要求する（当面はDxLib適用時に position/target/up を使う）
	void LookAt(const VECTOR& eye, const VECTOR& target, const VECTOR& up = VGet(0,1,0));
	bool HasLookAt() const noexcept { return _hasLookAt; }
	VECTOR LookAtTarget() const noexcept { return _lookAtTarget; }
	VECTOR LookAtUp() const noexcept { return _lookAtUp; }

private:
	mutable bool _dirty = true;
	mutable MATRIX _view{};

	bool _hasLookAt = false;
	VECTOR _lookAtTarget = VGet(0,0,0);
	VECTOR _lookAtUp = VGet(0,1,0);
};
