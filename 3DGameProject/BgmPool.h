#pragma once

#include <cstddef>
#include <vector>
#include <mutex>
#include <algorithm>
#include <functional>
#include <memory>

#include "Pool.h"
#include "Bgm.h"
#include "Time.h"

// BgmPool
// - Bgm ÇçƒóòópÇ∑ÇÈÉvÅ[Éã
class BgmPool : public Pool {
public:
	using TypedDeleter = std::function<void(Bgm*)>;
	using TypedUniquePtr = std::unique_ptr<Bgm, TypedDeleter>;

	explicit BgmPool(size_t maxSize =8)
		: _maxSize(maxSize) {
	}

	TypedUniquePtr AcquireBgm() {
		auto vp = Acquire();
		return TypedUniquePtr(static_cast<Bgm*>(vp.release()), [d = vp.get_deleter()](Bgm* p) mutable { d(p); });
	}

	UniquePtr Acquire() override {
		std::lock_guard lk(_mtx);
		Bgm* p = nullptr;
		if (!_free.empty()) {
			p = _free.back().obj;
			_free.pop_back();
		} else {
			p = new Bgm();
		}
		Deleter del = [this](void* obj) { this->Release(obj); };
		return UniquePtr(p, std::move(del));
	}

	void Release(void* obj) override {
		auto* bgm = static_cast<Bgm*>(obj);
		if (!bgm) return;

		bgm->Stop();

		std::lock_guard lk(_mtx);
		if (_free.size() >= _maxSize) {
			delete bgm;
			return;
		}
		_free.push_back(FreeEntry{ bgm, Time::Instance().GetTotalTime() });
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
		Bgm* obj{};
		double lastReleasedSec{};
	};

	mutable std::mutex _mtx;
	size_t _maxSize =8;
	std::vector<FreeEntry> _free;
};
