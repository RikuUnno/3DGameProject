#include "CameraManager.h"

#include "DxLib.h"
#include <algorithm>

namespace {
	inline float Clamp01(float t) noexcept {
		return std::clamp(t,0.0f,1.0f);
	}

	inline VECTOR LerpVec(const VECTOR& a, const VECTOR& b, float t) noexcept {
		return VAdd(a, VScale(VSub(b, a), t));
	}

	inline float LerpF(float a, float b, float t) noexcept { return a + (b - a) * t; }

	inline VECTOR LerpEuler(const VECTOR& a, const VECTOR& b, float t) noexcept {
		return LerpVec(a, b, t);
	}
}

CameraManager& CameraManager::Instance() noexcept {
	static CameraManager inst;
	return inst;
}

CameraManager::CameraId CameraManager::CreateCamera(int ownerSceneId) {
	const CameraId id = _nextId++;
	auto cam = std::make_unique<Camera>();
	cam->ownerSceneId = ownerSceneId;
	_cameras.emplace(id, std::move(cam));

	if (_activeId ==0) _activeId = id;
	if (_renderId ==0) _renderId = id;
	return id;
}

bool CameraManager::DestroyCamera(CameraId id) {
	auto it = _cameras.find(id);
	if (it == _cameras.end()) return false;
	_cameras.erase(it);

	if (_activeId == id) _activeId =0;
	if (_renderId == id) _renderId =0;

	if (_blend.active && (_blend.fromId == id || _blend.toId == id)) {
		_blend.active = false;
		_blend.scratch.reset();
	}
	return true;
}

void CameraManager::ReleaseBySceneId(int sceneId) {
	for (auto it = _cameras.begin(); it != _cameras.end();) {
		if (it->second && it->second->ownerSceneId == sceneId) {
			const auto removedId = it->first;
			it = _cameras.erase(it);
			if (_activeId == removedId) _activeId =0;
			if (_renderId == removedId) _renderId =0;
			if (_blend.active && (_blend.fromId == removedId || _blend.toId == removedId)) {
				_blend.active = false;
				_blend.scratch.reset();
			}
			continue;
		}
		++it;
	}
}

Camera* CameraManager::Get(CameraId id) {
	auto it = _cameras.find(id);
	return it == _cameras.end() ? nullptr : it->second.get();
}

const Camera* CameraManager::Get(CameraId id) const {
	auto it = _cameras.find(id);
	return it == _cameras.end() ? nullptr : it->second.get();
}

bool CameraManager::SetActive(CameraId id) {
	if (!_cameras.contains(id)) return false;
	_activeId = id;
	return true;
}

bool CameraManager::SetRender(CameraId id) {
	if (!_cameras.contains(id)) return false;
	_renderId = id;
	_blend.active = false;
	_blend.scratch.reset();
	return true;
}

Camera* CameraManager::Active() { return Get(_activeId); }
Camera* CameraManager::Render() { return Get(_renderId); }

bool CameraManager::BlendRenderTo(CameraId targetId, float durationSec) {
	if (!_cameras.contains(targetId)) return false;
	if (_renderId ==0) {
		_renderId = targetId;
		return true;
	}
	if (_renderId == targetId) return true;

	_blend.active = true;
	_blend.fromId = _renderId;
	_blend.toId = targetId;
	_blend.duration = (durationSec >0.0001f) ? durationSec :0.0001f;
	_blend.t =0.0f;
	_blend.scratch = std::make_unique<Camera>();
	// scratchは描画専用なのでシーン紐付けは不要
	_blend.scratch->ownerSceneId = -1;
	return true;
}

void CameraManager::Update(float dtSec) {
	if (!_blend.active) return;

	Camera* from = Get(_blend.fromId);
	Camera* to = Get(_blend.toId);
	if (!from || !to || !_blend.scratch) {
		_blend.active = false;
		_blend.scratch.reset();
		return;
	}

	_blend.t += dtSec;
	const float a = Clamp01(_blend.t / _blend.duration);

	//位置
	_blend.scratch->transform.SetLocalPosition(LerpVec(from->transform.LocalPosition(), to->transform.LocalPosition(), a));

	// 回転（Quaternion）
	const Quaternion rq = Quaternion::Slerp(from->transform.LocalRotation(), to->transform.LocalRotation(), a);
	_blend.scratch->transform.SetLocalRotation(rq);

	// 射影パラメータ
	_blend.scratch->fovYRad = LerpF(from->fovYRad, to->fovYRad, a);
	_blend.scratch->nearZ = LerpF(from->nearZ, to->nearZ, a);
	_blend.scratch->farZ = LerpF(from->farZ, to->farZ, a);

	_blend.scratch->MarkDirty();

	if (a >=1.0f) {
		_renderId = _blend.toId;
		_blend.active = false;
		_blend.scratch.reset();
	}
}

void CameraManager::ApplyRenderCameraToDxLib(int screenW, int screenH) {
	(void)screenW;
	(void)screenH;

	Camera* cam = _blend.active ? _blend.scratch.get() : Render();
	if (!cam) return;

	SetupCamera_Perspective(cam->fovYRad);
	SetCameraNearFar(cam->nearZ, cam->farZ);

	const VECTOR eye = cam->transform.LocalPosition();
	VECTOR target{};
	VECTOR up = VGet(0,1,0);

	if (cam->HasLookAt()) {
		target = cam->LookAtTarget();
		up = cam->LookAtUp();
	} else {
		const VECTOR forward = cam->transform.Forward();
		target = VAdd(eye, forward);
	}

	SetCameraPositionAndTargetAndUpVec(eye, target, up);
}
