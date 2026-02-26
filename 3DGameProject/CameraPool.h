#pragma once

#include <cstddef>
#include <vector>
#include <mutex>
#include <algorithm>

#include "Pool.h"
#include "Camera.h"
#include "Time.h"

// CameraPool
// - Camera をプールする（テンプレート設計をやめ、継承ベースにする）
class CameraPool : public Pool {
public:
	using TypedDeleter = std::function<void(Camera*)>;
	using TypedUniquePtr = std::unique_ptr<Camera, TypedDeleter>;

	explicit CameraPool(size_t maxSize =32)
		: _maxSize(maxSize) {
	}

	// typed API
	TypedUniquePtr AcquireCamera() {
		auto vp = Acquire();
		return TypedUniquePtr(static_cast<Camera*>(vp.release()), [d = vp.get_deleter()](Camera* p) mutable { d(p); });
	}

	// Pool
	UniquePtr Acquire() override {
		std::lock_guard lk(_mtx);
		Camera* p = nullptr;
		if (!_free.empty()) {
			p = _free.back().obj;
			_free.pop_back();
		} else {
			p = new Camera();
		}

		if (p) p->MarkDirty();

		Deleter del = [this](void* obj) { this->Release(obj); };
		return UniquePtr(p, std::move(del));
	}

	void Release(void* obj) override {
		auto* cam = static_cast<Camera*>(obj);
		if (!cam) return;

		cam->Reset();

		std::lock_guard lk(_mtx);
		if (_free.size() >= _maxSize) {
			delete cam;
			return;
		}
		_free.push_back(FreeEntry{ cam, Time::Instance().GetTotalTime() });
	}

	size_t Size() const override {
		std::lock_guard lk(_mtx);
		return _free.size();
	}

	void SetMaxSize(size_t maxSize) override {
		std::lock_guard lk(_mtx);
		_maxSize = maxSize;
		while (_free.size() > _maxSize) {
			delete _free.back().obj;
			_free.pop_back();
		}
	}

	void Clear() override {
		std::lock_guard lk(_mtx);
		for (auto& e : _free) {
			delete e.obj;
		}
		_free.clear();
	}

	size_t TrimUnused(double maxIdleSeconds, double nowSeconds) override {
		if (maxIdleSeconds <=0.0) return 0;
		std::lock_guard lk(_mtx);
		size_t removed =0;
		_free.erase(
			std::remove_if(_free.begin(), _free.end(), [&](const FreeEntry& e) {
				const double idle = nowSeconds - e.lastReleasedSec;
				if (idle < maxIdleSeconds) return false;
				delete e.obj;
				++removed;
				return true;
			}),
			_free.end()
		);
		return removed;
	}

private:
	struct FreeEntry {
		Camera* obj{};
		double lastReleasedSec{};
	};

	mutable std::mutex _mtx;
	size_t _maxSize =32;
	std::vector<FreeEntry> _free;
};
