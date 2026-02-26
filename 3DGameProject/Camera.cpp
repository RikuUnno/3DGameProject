#include "Camera.h"

const MATRIX& Camera::ViewMatrix() const {
	if (_dirty) {
		_view = MInverse(transform.WorldMatrix());
		_dirty = false;
	}
	return _view;
}

void Camera::MarkDirty() noexcept {
	_dirty = true;
}

void Camera::LookAt(const VECTOR& eye, const VECTOR& target, const VECTOR& up) {
	transform.SetLocalPosition(eye);
	_hasLookAt = true;
	_lookAtTarget = target;
	_lookAtUp = up;
	MarkDirty();
}

void Camera::Reset() noexcept {
	transform = Transform{};

	_fovYRad =60.0f * DX_PI_F /180.0f;
	_nearZ =0.1f;
	_farZ =1000.0f;

	_tag = CameraTag::Game;
	_ownerSceneId = -1;

	_hasLookAt = false;
	_lookAtTarget = VGet(0,0,0);
	_lookAtUp = VGet(0,1,0);

	_dirty = true;
	_view = MATRIX{};
}
