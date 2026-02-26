#pragma once

#include <cstddef>
#include <vector>
#include <mutex>
#include <algorithm>
#include <functional>
#include <memory>

#include "Pool.h"
#include "Se.h"
#include "Time.h"

// SePool
// - Se を再利用するプール（継承ベース）
class SePool : public Pool {
public:
	using TypedDeleter = std::function<void(Se*)>;
	using TypedUniquePtr = std::unique_ptr<Se, TypedDeleter>;

	explicit SePool(size_t maxSize =64)
		: _maxSize(maxSize) {
	}

	// typed API
	TypedUniquePtr AcquireSe() {
		auto vp = Acquire();
		return TypedUniquePtr(static_cast<Se*>(vp.release()), [d = vp.get_deleter()](Se* p) mutable { d(p); });
	}

	UniquePtr Acquire() override {
		std::lock_guard lk(_mtx);
		Se* p = nullptr;
		if (!_free.empty()) {
			p = _free.back().obj;
			_free.pop_back();
		} else {
			p = new Se();
		}
		Deleter del = [this](void* obj) { this->Release(obj); };
		return UniquePtr(p, std::move(del));
	}

	void Release(void* obj) override {
		auto* se = static_cast<Se*>(obj);
		if (!se) return;

		// 再利用前提: 再生停止してハンドルを残すかどうかは Manager 側で制御
		se->Stop();

		std::lock_guard lk(_mtx);
		if (_free.size() >= _maxSize) {
			delete se; // SeデストラクタはResetしないので、ここで明示的に解放
			return;
		}
		_free.push_back(FreeEntry{ se, Time::Instance().GetTotalTime() });
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
		Se* obj{};
		double lastReleasedSec{};
	};

	mutable std::mutex _mtx;
	size_t _maxSize =64;
	std::vector<FreeEntry> _free;
};
