#pragma once

#include <vector>
#include <functional>
#include <memory>
#include <mutex>

class GameObject;

// GameObject の生成/破棄コストを削減するためのオブジェクトプール。
// Acquire でプールから取り出し（なければ creator で新規生成）、
// Release でプールへ返却する。プールが満杯なら delete する。
class ObjectPool {
public:
    using Creator   = std::function<std::unique_ptr<GameObject>()>;
    using Deleter   = std::function<void(GameObject*)>;
    using UniquePtr = std::unique_ptr<GameObject, Deleter>;

    ObjectPool() = default;
    explicit ObjectPool(Creator creator, size_t maxSize = 64);

    // プールから取得。空なら creator で新規生成。
    // wasCreated が非 null なら新規生成かどうかを返す。
    UniquePtr Acquire(bool* wasCreated = nullptr);

    // プールへ返却（満杯なら delete）。Deleter を介して自動呼び出しされる。
    void Release(GameObject* obj);

    size_t Size() const;
    void   SetMaxSize(size_t maxSize);

    // 未使用ストックを全破棄（使用中オブジェクトには触れない）
    void Clear();

    // maxIdleSeconds 以上未使用のストックを削除。戻り値: 削除した個数。
    size_t TrimUnused(double maxIdleSeconds, double nowSeconds);

private:
    struct FreeEntry {
        GameObject* obj{};
        double lastReleasedSec{};
    };

    Creator                _creator;
    std::vector<FreeEntry> _freeList;
    size_t                 _maxSize = 64;
    mutable std::mutex     _mtx;
};
