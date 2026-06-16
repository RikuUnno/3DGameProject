#include "CameraManager.h"

#include "DxLib.h"
#include <algorithm>

// ヘルパー関数（CameraManager 内でのみ使用）
namespace {
	// 0～1 にクランプ
	inline float Clamp01(float t) noexcept {
		return std::clamp(t,0.0f,1.0f);
	}

	// ベクトルの線形補間
	inline VECTOR LerpVec(const VECTOR& a, const VECTOR& b, float t) noexcept {
		return VAdd(a, VScale(VSub(b, a), t));
	}

	// float の線形補間
	inline float LerpF(float a, float b, float t) noexcept { return a + (b - a) * t; }
}

// シングルトンインスタンス取得
CameraManager& CameraManager::Instance() noexcept {
	static CameraManager inst;
	return inst;
}

// カメラ作成
// ownerSceneId をセットして管理下に置く。返り値はカメラID
CameraManager::CameraId CameraManager::CreateCamera(int ownerSceneId) {
	const CameraId id = _nextId++;
	auto cam = _pool.Acquire();
	if (!cam) {
		// fallback（想定外だが安全側）
		Pool::Deleter del = [](void* p) { delete static_cast<Camera*>(p); };
		cam = Pool::UniquePtr(new Camera(), del);
	}
	// ownerSceneId をセット
	static_cast<Camera*>(cam.get())->_ownerSceneId = ownerSceneId;
	_cameras.emplace(id, std::move(cam));

	if (_activeId ==0) _activeId = id;
	if (_renderId ==0) _renderId = id;
	return id;
}

// カメラ破棄
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

// シーンID指定でカメラ一括破棄
void CameraManager::ReleaseBySceneId(int sceneId) {
	for (auto it = _cameras.begin(); it != _cameras.end();) {
		Camera* cam = static_cast<Camera*>(it->second.get());
		if (cam && cam->_ownerSceneId == sceneId) {
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

// カメラ取得
Camera* CameraManager::Get(CameraId id) {
	auto it = _cameras.find(id);
	return it == _cameras.end() ? nullptr : static_cast<Camera*>(it->second.get());
}

// カメラ取得（const 版）
const Camera* CameraManager::Get(CameraId id) const {
	auto it = _cameras.find(id);
	return it == _cameras.end() ? nullptr : static_cast<const Camera*>(it->second.get());
}

// アクティブカメラ設定
// Blend中であっても即座に切り替える（Blendはキャンセルされる）
bool CameraManager::SetActive(CameraId id) {
	if (!_cameras.contains(id)) return false;
	_activeId = id;
	return true;
}
// レンダリングカメラ設定
// Blend中であっても即座に切り替える（Blendはキャンセルされる）
bool CameraManager::SetRender(CameraId id) {
	if (!_cameras.contains(id)) return false;
	_renderId = id;
	_blend.active = false;
	_blend.scratch.reset();
	return true;
}

// アクティブカメラ取得/レンダリングカメラ取得
Camera* CameraManager::Active() { return Get(_activeId); }
Camera* CameraManager::Render() { return Get(_renderId); }

// レンダリングカメラを別のカメラへ Blend で切り替える
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
	_blend.scratch->_ownerSceneId = -1;
	return true;
}

// Blend 更新
// Blend 中であっても from/to カメラが破棄されたら Blend をキャンセルする
void CameraManager::Update(float dtSec) {
	if (!_blend.active) return; // Blend 中でなければ何もしない

	// from/to カメラが存在しないなら Blend をキャンセル
	Camera* from = Get(_blend.fromId);
	Camera* to = Get(_blend.toId);
	if (!from || !to || !_blend.scratch) { // 念のため scratch もチェック
		_blend.active = false;	// Blend キャンセル
		_blend.scratch.reset(); // Blend 用カメラ破棄
		return;					// Blend 中であっても from/to カメラが破棄されたら Blend をキャンセルする
	}

	// Blend 更新
	_blend.t += dtSec;
	const float a = Clamp01(_blend.t / _blend.duration);

	// 位置、回転、FOV、Near/Far を線形補間して Blend 用カメラにセット
	const VECTOR fromEye = from->transform.LocalPosition();	// fromEye は from カメラの位置
	const VECTOR toEye = to->transform.LocalPosition();		// toEye は to カメラの位置
	const VECTOR eye = LerpVec(fromEye, toEye, a);			// 位置は線形補間でセット
	_blend.scratch->transform.SetLocalPosition(eye);

	// 回転は Slerp（球面線形補間）で補間する
	const Quaternion rq = Quaternion::Slerp(from->transform.LocalRotation(), to->transform.LocalRotation(), a);
	_blend.scratch->transform.SetLocalRotation(rq);

	// FOV、Near/Far は線形補間で十分
	_blend.scratch->_fovYRad = LerpF(from->_fovYRad, to->_fovYRad, a);
	_blend.scratch->_nearZ = LerpF(from->_nearZ, to->_nearZ, a);
	_blend.scratch->_farZ = LerpF(from->_farZ, to->_farZ, a);

	// LookAt 情報も線形補間でセット
	const VECTOR fromTarget = from->HasLookAt() ? from->LookAtTarget() : VAdd(fromEye, from->transform.Forward());	// fromTarget は fromEye + from の前方ベクトル（LookAt でない場合は transform の向きで補う）
	const VECTOR toTarget = to->HasLookAt() ? to->LookAtTarget() : VAdd(toEye, to->transform.Forward());			// toTarget は toEye + to の前方ベクトル（LookAt でない場合は transform の向きで補う）
	const VECTOR fromUp = from->HasLookAt() ? from->LookAtUp() : VGet(0, 1, 0);										// fromUp は from のアップベクトル（LookAt でない場合はワールドアップを使用）
	const VECTOR toUp = to->HasLookAt() ? to->LookAtUp() : VGet(0, 1, 0);											// toUp は to のアップベクトル（LookAt でない場合はワールドアップを使用）
	_blend.scratch->LookAt(LerpVec(fromEye, toEye, a), LerpVec(fromTarget, toTarget, a), LerpVec(fromUp, toUp, a)); // LookAt 情報も線形補間でセット

	// Blend 用カメラを dirty にする（位置・回転・FOV・Near/Far を直接セットしているため）
	_blend.scratch->MarkDirty();

	// Blend 完了したらレンダリングカメラを切り替える
	if (a >=1.0f) {
		_renderId = _blend.toId;
		_blend.active = false;
		_blend.scratch.reset();
	}
}

// 現在のレンダリングカメラ（Blend中は Blend の結果）を DxLib に適用する
void CameraManager::ApplyRenderCameraToDxLib(int screenW, int screenH) {
	(void)screenW;
	(void)screenH;

	Camera* cam = _blend.active ? _blend.scratch.get() : Render();
	if (!cam) return;

	SetupCamera_Perspective(cam->_fovYRad);					// FOV をセット
	SetCameraNearFar(cam->_nearZ, cam->_farZ);				// Near/Far をセット

	const VECTOR eye = cam->transform.LocalPosition();		// カメラ位置は transform の位置
	VECTOR target{};
	VECTOR up = VGet(0,1,0);

	if (cam->HasLookAt()) {
		target = cam->LookAtTarget();
		up = cam->LookAtUp();
	} else {
		const VECTOR forward = cam->transform.Forward();
		target = VAdd(eye, forward);
	}

	SetCameraPositionAndTargetAndUpVec(eye, target, up);	// カメラの視点、注視点、アップベクトルをセットする
}
