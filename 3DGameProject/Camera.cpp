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
