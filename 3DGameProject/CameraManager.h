#pragma once

#include <unordered_map>
#include <memory>
#include <cstdint>

#include "Camera.h"
#include "CameraPool.h"
#include "Pool.h"
#include "Manager.h"

class CameraManager : public Manager {
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

	bool BlendRenderTo(CameraId targetId, float durationSec);
	bool IsBlending() const noexcept { return _blend.active; }
	void Update(float dtSec) override;

	// Manager
	void Update() override { /* no-op: CameraManager の更新は dt付き側で使う */ }

	void ApplyRenderCameraToDxLib(int screenW, int screenH);

private:
	CameraManager() = default;
	~CameraManager() = default;
	CameraManager(const CameraManager&) = delete;
	CameraManager& operator=(const CameraManager&) = delete;

	CameraId _nextId =1;
	CameraId _activeId =0;
	CameraId _renderId =0;

	CameraPool _pool{32 };
	std::unordered_map<CameraId, Pool::UniquePtr> _cameras;

	struct BlendState {
		bool active = false;
		CameraId fromId =0;
		CameraId toId =0;
		float duration =0.0f;
		float t =0.0f;

		std::unique_ptr<Camera> scratch;
	} _blend;
};
