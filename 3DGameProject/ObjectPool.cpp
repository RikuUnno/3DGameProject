#include "ObjectPool.h"
#include "GameObject.h"
#include "Time.h"
#include <cassert>
#include <algorithm>

/*
	ObjectPool の実装
	- 目的: 頻繁に生成/破棄される GameObject を再利用し、アロケーション負荷を低減する。
	- スレッド安全: 内部の freeList_ 操作は mtx_ によって保護される。
	- 挙動要約:
	  Acquire()
		- プールに利用可能なオブジェクトがあれば取り出して返す（Deleter はこのプールへ返却する関数）。
		- なければ creator_ を使って新規生成し、同様にプールへ返却する Deleter で包んで返す。
	  Release(obj)
		- プールが満杯でなければ freeList_ に戻す。満杯なら delete して破棄する。
*/

ObjectPool::ObjectPool(Creator creator, size_t maxSize)
	: _creator(std::move(creator)), _maxSize(maxSize) {
}

// Acquire: プールからオブジェクトを取得する。
ObjectPool::UniquePtr ObjectPool::Acquire(bool* wasCreated) {
	std::lock_guard lk(_mtx);
	if (!_freeList.empty()) {
		if (wasCreated) *wasCreated = false;
		GameObject* p = _freeList.back().obj;
		_freeList.pop_back();
		Deleter del = [this](GameObject* obj) { this->Release(obj); };
		return UniquePtr(p, del);
	}
	if (_creator) {
		if (wasCreated) *wasCreated = true;
		auto up = _creator();
		GameObject* raw = up.release();
		Deleter del = [this](GameObject* obj) { this->Release(obj); };
		return UniquePtr(raw, del);
	}
	if (wasCreated) *wasCreated = false;
	return nullptr;
}

// Release: プールへオブジェクトを返却する
// - プール満杯(maxSize_) の場合はオブジェクトを破棄する
// - そうでなければ freeList_ に追加して再利用可能にする
void ObjectPool::Release(GameObject* obj) {
	if (!obj) return;
	std::lock_guard lk(_mtx);
	if (_freeList.size() >= _maxSize) {
		delete obj;
		return;
	}
	_freeList.push_back(FreeEntry{ obj, Time::Instance().GetTotalTime() });
}


// 現在プールにあるオブジェクト数を返す
size_t ObjectPool::Size() const {
	std::lock_guard lk(_mtx);
	return _freeList.size();
}

// プールの最大サイズを設定する（トリミングを行う）
// - maxSize_ より多い要素があれば破棄してサイズを縮める
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
	for (auto& e : _freeList) {
		delete e.obj;
	}
	_freeList.clear();
}

size_t ObjectPool::TrimUnused(double maxIdleSeconds, double nowSeconds) {
	if (maxIdleSeconds <= 0.0) return 0;

	std::lock_guard lk(_mtx);
	const auto before = _freeList.size();

	_freeList.erase(
		std::remove_if(_freeList.begin(), _freeList.end(), [&](const FreeEntry& e) {
			const double idle = nowSeconds - e.lastReleasedSec;
			if (idle < maxIdleSeconds) return false;
			delete e.obj;
			return true;
			}),
		_freeList.end()
	);

	return before - _freeList.size();
}