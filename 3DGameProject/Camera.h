#pragma once

#include "DxLib.h"
#include "Transform.h"
#include "CameraTags.h"

class Camera {
public:
	Transform transform;

	// 固定値（Renderに反映）
	float _fovYRad =60.0f * DX_PI_F /180.0f;
	float _nearZ =0.1f;
	float _farZ =1000.0f;

	// tag/layer 的な属性
	CameraTag _tag = CameraTag::Game;
	int _ownerSceneId = -1;

	const MATRIX& ViewMatrix() const;
	void MarkDirty() noexcept;

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
