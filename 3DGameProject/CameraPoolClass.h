#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>

#include "Pool.h"
#include "Camera.h"

// CameraPoolClass
// - CameraManagerから "カメラの所有" を分離したクラス（実験用/参考）
// - 現状のプロジェクトでは CameraManager が内部で CameraPool を使うため、こちらは補助的
class CameraPoolClass {
public:
	using CameraId = std::uint32_t;
	using PoolType = Pool<Camera>;

	CameraPoolClass()
		: _pool([] { return std::make_unique<Camera>(); }) {
	}

	CameraId Create(int ownerSceneId) {
		const CameraId id = _nextId++;
		auto cam = std::make_unique<Camera>();
		cam->_ownerSceneId = ownerSceneId;
		_cameras.emplace(id, std::move(cam));
		return id;
	}

	bool Destroy(CameraId id) {
		auto it = _cameras.find(id);
		if (it == _cameras.end()) return false;
		_cameras.erase(it);
		return true;
	}

	void ReleaseBySceneId(int sceneId) {
		for (auto it = _cameras.begin(); it != _cameras.end();) {
			if (it->second && it->second->_ownerSceneId == sceneId) {
				it = _cameras.erase(it);
				continue;
			}
			++it;
		}
	}

	Camera* Get(CameraId id) {
		auto it = _cameras.find(id);
		return it == _cameras.end() ? nullptr : it->second.get();
	}

	const Camera* Get(CameraId id) const {
		auto it = _cameras.find(id);
		return it == _cameras.end() ? nullptr : it->second.get();
	}

	bool Contains(CameraId id) const { return _cameras.contains(id); }

	void Clear() { _cameras.clear(); }

	CameraId NextIdForDebug() const noexcept { return _nextId; }

	PoolType& PoolRef() { return _pool; }
	const PoolType& PoolRef() const { return _pool; }

private:
	CameraId _nextId =1;
	std::unordered_map<CameraId, std::unique_ptr<Camera>> _cameras;
	PoolType _pool;
};
