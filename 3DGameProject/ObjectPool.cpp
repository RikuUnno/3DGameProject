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
	: creator_(std::move(creator)), maxSize_(maxSize) {}

// Acquire: プールからオブジェクトを取得する。
ObjectPool::UniquePtr ObjectPool::Acquire() {
	std::lock_guard lk(mtx_); // freeList_ 操作を保護
	if (!freeList_.empty()) {
		GameObject* p = freeList_.back().obj; // プール末尾から取り出す
		freeList_.pop_back();
		// 返却時にこのプールに戻すデリータを作成
		Deleter del = [this](GameObject* obj) { this->Release(obj); };
		return UniquePtr(p, del); // カスタムデリータ付き unique_ptr を返す
	}
	// プールに余裕がなければ creator_ で新規生成
	if (creator_) {
		auto up = creator_();        // std::unique_ptr<GameObject>
		GameObject* raw = up.release(); // 生ポインタを取り出す
		Deleter del = [this](GameObject* obj) { this->Release(obj); };
		return UniquePtr(raw, del); // 新規オブジェクトも同様にプールへ返却されるようにする
	}
	// 生成方法が無ければ nullptr を返す
	return nullptr;
}

// Release: プールへオブジェクトを返却する
// - プールが満杯(maxSize_) の場合はオブジェクトを破棄する
// - そうでなければ freeList_ に追加して再利用可能にする
void ObjectPool::Release(GameObject* obj) {
	if (!obj) return;
	std::lock_guard lk(mtx_);
	if (freeList_.size() >= maxSize_) {
		delete obj;
		return;
	}
	freeList_.push_back(FreeEntry{ obj, Time::Instance().GetTotalTime() });
}


// 現在プールにあるオブジェクト数を返す
size_t ObjectPool::Size() const {
	std::lock_guard lk(mtx_);
	return freeList_.size();
}

// プールの最大サイズを設定する（トリミングを行う）
// - maxSize_ より多い要素があれば破棄してサイズを縮める
void ObjectPool::SetMaxSize(size_t maxSize) {
	std::lock_guard lk(mtx_);
	maxSize_ = maxSize;
	while (freeList_.size() > maxSize_) {
		delete freeList_.back().obj;
		freeList_.pop_back();
	}
}

void ObjectPool::Clear() {
	std::lock_guard lk(mtx_);
	for (auto& e : freeList_) {
		delete e.obj;
	}
	freeList_.clear();
}

size_t ObjectPool::TrimUnused(double maxIdleSeconds, double nowSeconds) {
	if (maxIdleSeconds <= 0.0) return 0;

	std::lock_guard lk(mtx_);
	const auto before = freeList_.size();

	freeList_.erase(
		std::remove_if(freeList_.begin(), freeList_.end(), [&](const FreeEntry& e) {
			const double idle = nowSeconds - e.lastReleasedSec;
			if (idle < maxIdleSeconds) return false;
			delete e.obj;
			return true;
		}),
		freeList_.end()
	);

	return before - freeList_.size();
}