#pragma once

#include <cstddef>
#include <vector>
#include <mutex>
#include <algorithm>
#include <functional>
#include <memory>

#include "Pool.h"
#include "IModel.h"
#include "Time.h"

// ModelPool
// - 1 つのモデルキー (= テンプレート) に対する IModel 再利用プール
// - BgmPool と同じ Pool 派生スタイル
// - Creator は ModelManager が「テンプレートを Duplicate する関数」を渡す想定
//   (重い実ファイルの再読み込みは行わず、軽量複製で済ませる)
class ModelPool : public Pool {
public:
    using Creator = std::function<std::unique_ptr<IModel>()>;
    using TypedDeleter = std::function<void(IModel*)>;
    using TypedUniquePtr = std::unique_ptr<IModel, TypedDeleter>;

    ModelPool() = default;
    explicit ModelPool(Creator creator, size_t maxSize = 32)
        : _creator(std::move(creator)), _maxSize(maxSize) {
    }

    void SetCreator(Creator c) {
        std::lock_guard lk(_mtx);
        _creator = std::move(c);
    }

    TypedUniquePtr AcquireModel() {
        auto vp = Acquire();
        IModel* raw = static_cast<IModel*>(vp.release());
        auto del = vp.get_deleter();
        return TypedUniquePtr(raw, [d = std::move(del)](IModel* p) mutable { d(p); });
    }

    UniquePtr Acquire() override {
        IModel* p = nullptr;
        {
            std::lock_guard lk(_mtx);
            if (!_free.empty()) {
                p = _free.back().obj;
                _free.pop_back();
            }
        }
        if (!p) {
            if (_creator) {
                auto up = _creator();
                p = up.release();
            }
        }
        Deleter del = [this](void* obj) { this->Release(obj); };
        return UniquePtr(p, std::move(del));
    }

    void Release(void* obj) override {
        auto* m = static_cast<IModel*>(obj);
        if (!m) return;

        // 状態初期化 (見えない位置で再利用される事故を防ぐため可視は維持)
        // 必要なら追加リセットは派生 IModel 側で
        std::lock_guard lk(_mtx);
        if (_free.size() >= _maxSize) {
            delete m;
            return;
        }
        _free.push_back(FreeEntry{ m, Time::Instance().GetTotalTime() });
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
        for (auto& e : _free) delete e.obj;
        _free.clear();
    }

    size_t TrimUnused(double maxIdleSeconds, double nowSeconds) override {
        if (maxIdleSeconds <= 0.0) return 0;
        std::lock_guard lk(_mtx);
        size_t removed = 0;
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
        IModel* obj{};
        double lastReleasedSec{};
    };

    Creator _creator;
    mutable std::mutex _mtx;
    size_t _maxSize = 32;
    std::vector<FreeEntry> _free;
};
