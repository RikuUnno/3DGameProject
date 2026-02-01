#pragma once

#include <unordered_map>
#include <memory>
#include <cstdint>

#include "Camera.h"

// CameraManager
// - 複数カメラを所有
// - A: active（論理） / B: render（描画に使用） を分離
// - Scene 終了時に ownerSceneIdで一括回収できる
class CameraManager {
public:
	using CameraId = std::uint32_t;

	static CameraManager& Instance() noexcept;

	CameraId CreateCamera(int ownerSceneId);
	bool DestroyCamera(CameraId id);
	void ReleaseBySceneId(int sceneId);

	Camera* Get(CameraId id);
	const Camera* Get(CameraId id) const;

	CameraId ActiveCameraId() const noexcept { return _activeId; }
	CameraId RenderCameraId() const noexcept { return _renderId; }

	bool SetActive(CameraId id);
	bool SetRender(CameraId id);

	Camera* Active();
	Camera* Render();

	// --- blend (Render camera transition) ---
	// 現在の Renderから targetへ duration 秒で補間して切替
	bool BlendRenderTo(CameraId targetId, float durationSec);
	bool IsBlending() const noexcept { return _blend.active; }
	void Update(float dtSec);

	// レンダーカメラをDxLibに適用（Draw直前で呼ぶ想定）
	void ApplyRenderCameraToDxLib(int screenW, int screenH);

private:
	CameraManager() = default;
	~CameraManager() = default;
	CameraManager(const CameraManager&) = delete;
	CameraManager& operator=(const CameraManager&) = delete;

	CameraId _nextId =1;
	CameraId _activeId =0;
	CameraId _renderId =0;

	std::unordered_map<CameraId, std::unique_ptr<Camera>> _cameras;

	struct BlendState {
		bool active = false;
		CameraId fromId =0;
		CameraId toId =0;
		float duration =0.0f;
		float t =0.0f;

		//補間結果を保持するための作業カメラ（Renderとして使う）
		std::unique_ptr<Camera> scratch;
	} _blend;
};
