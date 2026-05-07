#include "ObjectPool.h"
#include "GameObject.h"
#include "Time.h"
#include <algorithm>

ObjectPool::ObjectPool(Creator creator, size_t maxSize)
    : _creator(std::move(creator)), _maxSize(maxSize)
{
}

ObjectPool::UniquePtr ObjectPool::Acquire(bool* wasCreated) {
    Deleter del = [this](GameObject* obj) { this->Release(obj); };

    std::lock_guard lk(_mtx);
    if (!_freeList.empty()) {
        if (wasCreated) *wasCreated = false;
        GameObject* p = _freeList.back().obj;
        _freeList.pop_back();
        return UniquePtr(p, del);
    }
    if (_creator) {
        if (wasCreated) *wasCreated = true;
        auto up = _creator();
        return UniquePtr(up.release(), del);
    }
    if (wasCreated) *wasCreated = false;
    return UniquePtr(nullptr, del);
}

void ObjectPool::Release(GameObject* obj) {
    if (!obj) return;
    std::lock_guard lk(_mtx);
    if (_freeList.size() >= _maxSize) {
        delete obj;
        return;
    }
    _freeList.push_back(FreeEntry{ obj, Time::Instance().GetTotalTime() });
}

size_t ObjectPool::Size() const {
    std::lock_guard lk(_mtx);
    return _freeList.size();
}

void ObjectPool::SetMaxSize(size_t maxSize) {
    std::lock_guard lk(_mtx);
    _maxSize = maxSize;
    while (_freeList.size() > _maxSize) {
        delete _freeList.back().obj;
        _freeList.pop_back();
    }
}

void ObjectPool::Clear() {
    std::lock_guard lk(_mtx);
    for (auto& e : _freeList) delete e.obj;
    _freeList.clear();
}

size_t ObjectPool::TrimUnused(double maxIdleSeconds, double nowSeconds) {
    if (maxIdleSeconds <= 0.0) return 0;
    std::lock_guard lk(_mtx);
    const size_t before = _freeList.size();
    _freeList.erase(
        std::remove_if(_freeList.begin(), _freeList.end(), [&](const FreeEntry& e) {
            if (nowSeconds - e.lastReleasedSec < maxIdleSeconds) return false;
            delete e.obj;
            return true;
        }),
        _freeList.end()
    );
    return before - _freeList.size();
}
