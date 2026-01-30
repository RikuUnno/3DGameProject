#include "ObjectManager.h"
#include "ObjectFactory.h"
#include "ObjectPool.h"
#include "GameObject.h"
#include "Time.h"
#include <algorithm>
#include <iostream>
#include <functional>

#ifdef _DEBUG
#include "DxLib.h"
#endif

// ObjectManager はシングルトンでオブジェクトの所有・再利用管理を行う。
// - Factory: 生成責務（ObjectFactory）に委譲してオブジェクトを作る。
// - Pool: 再利用可能な型は ObjectPool を通して Acquire/Release する。
// - 内部コンテナ objects_ は ObjectPool::UniquePtr を保持する（カスタムデリータ許容）。
// スレッド安全のため内部操作は mtx_ で保護する。

ObjectManager& ObjectManager::Instance() noexcept {
	static ObjectManager inst;
	return inst;
}

ObjectManager::~ObjectManager() {
	// 管理中オブジェクトとプールを破棄（unique_ptr のデリータで適切に処理される）
	objects_.clear();
	pools_.clear();
}

// 指定キーでプールを登録する。
// - key に対して ObjectPool を生成する。creator は ObjectFactory::Create を利用する。
// - maxSize はプールの最大保持数（満杯時は破棄される）
void ObjectManager::RegisterPool(const std::string& key, size_t maxSize) {
	std::lock_guard lk(mtx_);
	if (pools_.find(key) == pools_.end()) {
		// プール用の Creator を生成（Factory::Create を利用）
		auto poolCreator = [key]() -> std::unique_ptr<GameObject> {
			return ObjectFactory::Instance().Create(key);
			};
		pools_[key] = std::make_unique<ObjectPool>(poolCreator, maxSize);
	}
}

void ObjectManager::SetCurrentSceneId(int sceneId) {
	std::lock_guard lk(mtx_);
	currentSceneId_ = sceneId;
}

int ObjectManager::CurrentSceneId() const {
	std::lock_guard lk(mtx_);
	return currentSceneId_;
}

void ObjectManager::ReleaseBySceneId(int sceneId) {
	// 注意: Release は objects_ を eraseするので、イテレータを使ったループでまとめて処理する
	std::lock_guard lk(mtx_);
	for (auto it = objects_.begin(); it != objects_.end(); ) {
		GameObject* obj = it->get();
		if (!obj || obj->ownerSceneId != sceneId) {
			++it;
			continue;
		}

		// Release(GameObject*) と同じルールで処理するが、再ロックを避けてここで完結させる
		if (!obj->poolKey.empty()) {
			obj->OnRelease();
			it = objects_.erase(it); // deleter がプールへ返す
			continue;
		}
		obj->OnDestroy();
#ifdef _DEBUG
		++debugTotalDeleted_;
#endif
		it = objects_.erase(it);
	}
}

// Spawn: オブジェクトを取得して Manager が所有する。
// フロー:
// 1) まずプールがあれば Acquire() して OnAcquire(params) を呼ぶ。
// 2) プールが無い/空なら Factory::Create(key, params) による生成を行う。
// どちらの場合も最終的に内部 objects_ に所有権を移して raw ポインタを返す。
// - 所有権ルール: 呼び出し側は返却（Release）を ObjectManager に委ねる。
GameObject* ObjectManager::Spawn(const std::string& key, const VariantMap& params) {
	std::unique_ptr<GameObject> up;

	//1) try pool
	{
		std::lock_guard lk(mtx_);
		auto pit = pools_.find(key);
		if (pit != pools_.end()) {
			auto u = pit->second->Acquire();
			if (u) {
				GameObject* raw = u.get();
				raw->poolKey = key;
				raw->ownerSceneId = currentSceneId_;
				raw->OnAcquire(params);
				objects_.push_back(std::move(u));
#ifdef _DEBUG
				++debugTotalSpawn_; //取得成功（プール経由）
#endif
				return raw;
			}
		}
	}

	//2) fallback to factory create
	up = ObjectFactory::Instance().Create(key, params);
	if (!up) return nullptr;

	up->poolKey.clear();
	up->ownerSceneId = CurrentSceneId();
	up->OnAcquire(params);
	GameObject* raw = up.get();
	{
		std::lock_guard lk(mtx_);
		objects_.push_back(
			ObjectPool::UniquePtr(
				up.release(),
				ObjectPool::Deleter([](GameObject* p) { delete p; })
			)
		);
#ifdef _DEBUG
		++debugTotalSpawn_; //取得成功（Factory new 経由）
#endif
	}
	return raw;
}

void ObjectManager::Release(GameObject* obj) {
	if (!obj) return;
	std::lock_guard lk(mtx_);
	auto it = std::find_if(objects_.begin(), objects_.end(),
		[obj](const ObjectPool::UniquePtr& up) { return up.get() == obj; });
	if (it == objects_.end()) return;

	std::string key = obj->poolKey;
	if (!key.empty()) {
		obj->OnRelease();
		objects_.erase(it);
		return;
	}
	(*it)->OnDestroy();
#ifdef _DEBUG
	++debugTotalDeleted_;
#endif
	objects_.erase(it);
}

bool ObjectManager::RemoveById(int id) {
	std::lock_guard lk(mtx_);
	auto it = std::find_if(objects_.begin(), objects_.end(),
		[id](const ObjectPool::UniquePtr& up) { return up && up->GetId() == id; });
	if (it == objects_.end()) return false;
	(*it)->OnDestroy();
#ifdef _DEBUG
	++debugTotalDeleted_;
#endif
	objects_.erase(it);
	return true;
}

bool ObjectManager::ClearPool(const std::string& key) {
	std::lock_guard lk(mtx_);
	auto it = pools_.find(key);
	if (it == pools_.end() || !it->second) return false;
#ifdef _DEBUG
	//クリア前に消える数を加算
	debugTotalDeleted_ += it->second->Size();
#endif
	it->second->Clear();
	return true;
}

size_t ObjectManager::TrimPoolUnused(const std::string& key, double maxIdleSeconds) {
	const double now = Time::Instance().GetTotalTime();
	std::lock_guard lk(mtx_);
	auto it = pools_.find(key);
	if (it == pools_.end() || !it->second) return 0;
	const size_t removed = it->second->TrimUnused(maxIdleSeconds, now);
#ifdef _DEBUG
	debugTotalDeleted_ += removed;
#endif
	return removed;
}

size_t ObjectManager::TrimAllPoolsUnused(double maxIdleSeconds) {
	const double now = Time::Instance().GetTotalTime();
	std::lock_guard lk(mtx_);
	size_t total = 0;
	for (auto& [k, pool] : pools_) {
		if (!pool) continue;
		const size_t removed = pool->TrimUnused(maxIdleSeconds, now);
		total += removed;
#ifdef _DEBUG
		debugTotalDeleted_ += removed;
#endif
	}
	return total;
}

bool ObjectManager::UnregisterPool(const std::string& key) {
	std::lock_guard lk(mtx_);
	for (const auto& up : objects_) {
		if (up && up->poolKey == key) return false;
	}
	auto it = pools_.find(key);
	if (it == pools_.end()) return false;
#ifdef _DEBUG
	if (it->second) debugTotalDeleted_ += it->second->Size();
#endif
	if (it->second) it->second->Clear();
	pools_.erase(it);
	return true;
}

#ifdef _DEBUG
void ObjectManager::DebugDraw(int x, int y) const {
	std::lock_guard lk(mtx_);

	DrawFormatString(x, y, GetColor(255,255,0), "[ObjectManager] 現在オブジェクト数: %d", (int)objects_.size());
	y +=16;
	DrawFormatString(x, y, GetColor(255,255,0), "[ObjectManager] 総生成数(取得回数): %d", (int)debugTotalSpawn_);
	y +=16;
	DrawFormatString(x, y, GetColor(255,255,0), "[ObjectManager] 総削除数(破棄回数): %d", (int)debugTotalDeleted_);
	y +=16;
	DrawFormatString(x, y, GetColor(255,255,0), "[ObjectManager] 現在SceneId: %d", currentSceneId_);
	y +=16;
	DrawFormatString(x, y, GetColor(255,255,0), "[ObjectManager] プール数: %d", (int)pools_.size());
	y +=16;

	for (const auto& [key, pool] : pools_) {
		const size_t freeCount = pool ? pool->Size() :0;
		DrawFormatString(x, y, GetColor(200,255,200), "- %s 未使用ストック: %d", key.c_str(), (int)freeCount);
		y +=16;
	}
}
#endif