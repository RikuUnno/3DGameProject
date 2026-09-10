#pragma once

#include <cstddef>
#include <vector>
#include <mutex>
#include <algorithm>

#include "Pool.h"
#include "Camera.h"
#include "Time.h"

// CameraPool
// - Camera をプールする（継承ベース）
class CameraPool : public Pool {
public:
	using TypedDeleter = std::function<void(Camera*)>;				// Camera 用の型付きデリータ
	using TypedUniquePtr = std::unique_ptr<Camera, TypedDeleter>;	// Camera 用の型付きユニークポインタ

	// コンストラクタ
	explicit CameraPool(size_t maxSize =32)
		: _maxSize(maxSize) {	// デフォルト最大サイズ 32
	}

	// typed API : Camera* 用の型付きユニークポインタを返す
	TypedUniquePtr AcquireCamera() {
		auto vp = Acquire();
		return TypedUniquePtr(static_cast<Camera*>(vp.release()), [d = vp.get_deleter()](Camera* p) mutable { d(p); });
	}

	// Pool : void* 用の型非依存ユニークポインタを返す
	UniquePtr Acquire() override {
		std::lock_guard lk(_mtx);
		Camera* p = nullptr;
		// まずはプールから取得
		if (!_free.empty()) {
			p = _free.back().obj;
			_free.pop_back();
		}
		else { // プールが空なら新規作成
			p = new Camera();
		}

		// Camera は Transform を持つので、再利用時に dirty にしておく
		if (p) p->MarkDirty();

		// 返却時のデリータを設定して UniquePtr を返す
		Deleter del = [this](void* obj) { this->Release(obj); };
		// UniquePtr は void* 用なので、Camera* を void* にキャストして渡す
		return UniquePtr(p, std::move(del));
	}

	// Pool : void* 用の型非依存ユニークポインタを返す
	void Release(void* obj) override {
		// Camera* にキャストして、Camera の状態を初期化してからプールに戻す
		auto* cam = static_cast<Camera*>(obj);
		if (!cam) return;

		// Camera は Transform を持つので、再利用時に dirty にしておく
		cam->Reset();

		// ミューテクスで保護しつつ、プールの最大サイズを超えていなければプールに戻す
		std::lock_guard lk(_mtx);
		if (_free.size() >= _maxSize) {
			delete cam;
			return;
		}
		_free.push_back(FreeEntry{ cam, Time::Instance().GetTotalTime() }); // 現在時刻を記録してプールに戻す
	}

	// Size : プール内の利用可能オブジェクト数を返す
	size_t Size() const override {
		std::lock_guard lk(_mtx);
		return _free.size();
	}

	// SetMaxSize : プールの最大サイズを設定する
	void SetMaxSize(size_t maxSize) override {
		std::lock_guard lk(_mtx);
		_maxSize = maxSize;
		while (_free.size() > _maxSize) {
			delete _free.back().obj;
			_free.pop_back();
		}
	}

	// Clear : freeList を全破棄する（使用中のオブジェクトには触れない）
	void Clear() override {
		std::lock_guard lk(_mtx);
		for (auto& e : _free) {
			delete e.obj;
		}
		_free.clear();
	}

	// TrimUnused : 指定秒以上「未使用」のストックを削除する（使用中のオブジェクトには触れない）。戻り値: 削除した個数
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
	// プール内の Camera と最後に返却された時刻を保持する構造体
	struct FreeEntry {
		Camera* obj{};
		double lastReleasedSec{};	// 最後に返却された時刻
	};

	mutable std::mutex _mtx;		// プールのスレッドセーフ用ミューテクス
	size_t _maxSize = 32;			// プールの最大サイズ
	std::vector<FreeEntry> _free;	// プール内の利用可能な Camera のリスト
};
