#include "ObjectManager.h"
#include "ObjectFactory.h"
#include "ObjectPool.h"
#include "GameObject.h"
#include "Time.h"
#include "ThreadPool.h"
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
	_objects.clear();
	_pools.clear();
}

// 指定キーでプールを登録する。
// - key に対して ObjectPool を生成する。creator は ObjectFactory::Create を利用する。
// - maxSize はプールの最大保持数（満杯時は破棄される）
void ObjectManager::RegisterPool(const std::string& key, size_t maxSize) {
	std::lock_guard lk(_mtx);
	if (_pools.find(key) == _pools.end()) {
		// プール用の Creator を生成（Factory::Create を利用）
		auto poolCreator = [key]() -> std::unique_ptr<GameObject> {
			return ObjectFactory::Instance().Create(key);
			};
		_pools[key] = std::make_unique<ObjectPool>(poolCreator, maxSize);
	}
}

void ObjectManager::SetCurrentSceneId(int sceneId) {
	std::lock_guard lk(_mtx);
	_currentSceneId = sceneId;
}

int ObjectManager::CurrentSceneId() const {
	std::lock_guard lk(_mtx);
	return _currentSceneId;
}

void ObjectManager::ReleaseBySceneId(int sceneId) {
	// 注意: Release は objects_ を eraseするので、イテレータを使うループでまとめて処理する
	std::lock_guard lk(_mtx);
	for (auto it = _objects.begin(); it != _objects.end(); ) {
		GameObject* obj = it->get();
		if (!obj || obj->_ownerSceneId != sceneId) {
			++it;
			continue;
		}

		// シーン終了時フック（コメント通り：終了時に一度だけ）
		obj->End();

		if (!obj->_poolKey.empty()) {
			obj->OnRelease();
			obj->SetActive(false);
			it = _objects.erase(it);
			continue;
		}
		obj->OnDestroy();
#ifdef _DEBUG
		++_debugTotalDeleted;
#endif
		obj->SetActive(false);
		it = _objects.erase(it);
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
		std::lock_guard lk(_mtx);
		auto pit = _pools.find(key);
		if (pit != _pools.end()) {
			bool wasCreated = false;
			auto u = pit->second->Acquire(&wasCreated);
			if (u) {
				GameObject* raw = u.get();
				raw->SetActive(true);
				raw->_poolKey = key;
				raw->_ownerSceneId = _currentSceneId;
				raw->OnAcquire(params);
				_objects.push_back(std::move(u));
#ifdef _DEBUG
				++_debugTotalAcquire;
				if (wasCreated) {
					++_debugTotalCreated;
				}
#endif
				return raw;
			}
		}
	}

	//2) fallback to factory create
	up = ObjectFactory::Instance().Create(key, params);
	if (!up) return nullptr;

	up->SetActive(true);
	up->_poolKey.clear();
	up->_ownerSceneId = CurrentSceneId();
	up->OnAcquire(params);
	GameObject* raw = up.get();
	{
		std::lock_guard lk(_mtx);
		_objects.push_back(
			ObjectPool::UniquePtr(
				up.release(),
				ObjectPool::Deleter([](GameObject* p) { delete p; })
			)
		);
#ifdef _DEBUG
		++_debugTotalAcquire;
		++_debugTotalCreated;
#endif
	}
	return raw;
}

void ObjectManager::Release(GameObject* obj) {
	if (!obj) return;
	std::lock_guard lk(_mtx);
	auto it = std::find_if(_objects.begin(), _objects.end(),
		[obj](const ObjectPool::UniquePtr& up) { return up.get() == obj; });
	if (it == _objects.end()) return;

	std::string key = obj->_poolKey;
	if (!key.empty()) {
		obj->OnRelease();
		obj->SetActive(false);
		_objects.erase(it);
		return;
	}
	(*it)->OnDestroy();
#ifdef _DEBUG
	++_debugTotalDeleted;
#endif
	(*it)->SetActive(false);
	_objects.erase(it);
}

bool ObjectManager::RemoveById(int id) {
	std::lock_guard lk(_mtx);
	auto it = std::find_if(_objects.begin(), _objects.end(),
		[id](const ObjectPool::UniquePtr& up) { return up && up->GetId() == id; });
	if (it == _objects.end()) return false;
	(*it)->OnDestroy();
#ifdef _DEBUG
	++_debugTotalDeleted;
#endif
	_objects.erase(it);
	return true;
}

bool ObjectManager::ClearPool(const std::string& key) {
	std::lock_guard lk(_mtx);
	auto it = _pools.find(key);
	if (it == _pools.end() || !it->second) return false;
#ifdef _DEBUG
	//クリア前に消える数を加算
	_debugTotalDeleted += it->second->Size();
#endif
	it->second->Clear();
	return true;
}

size_t ObjectManager::TrimPoolUnused(const std::string& key, double maxIdleSeconds) {
	const double now = Time::Instance().GetTotalTime();
	std::lock_guard lk(_mtx);
	auto it = _pools.find(key);
	if (it == _pools.end() || !it->second) return 0;
	const size_t removed = it->second->TrimUnused(maxIdleSeconds, now);
#ifdef _DEBUG
	_debugTotalDeleted += removed;
#endif
	return removed;
}

size_t ObjectManager::TrimAllPoolsUnused(double maxIdleSeconds) {
	const double now = Time::Instance().GetTotalTime();
	std::lock_guard lk(_mtx);
	size_t total = 0;
	for (auto& [k, pool] : _pools) {
		if (!pool) continue;
		const size_t removed = pool->TrimUnused(maxIdleSeconds, now);
		total += removed;
#ifdef _DEBUG
		_debugTotalDeleted += removed;
#endif
	}
	return total;
}

bool ObjectManager::UnregisterPool(const std::string& key) {
	std::lock_guard lk(_mtx);
	for (const auto& up : _objects) {
		if (up && up->_poolKey == key) return false;
	}
	auto it = _pools.find(key);
	if (it == _pools.end()) return false;
#ifdef _DEBUG
	if (it->second) _debugTotalDeleted += it->second->Size();
#endif
	if (it->second) it->second->Clear();
	_pools.erase(it);
	return true;
}

void ObjectManager::UpdateAll(float dtSec) {
	// raw ポインタのスナップショットをロック下で取得し、
	// ParallelFor はロック外で実行する。
	// これにより obj->Update() 内で Spawn/Release を呼んでもデッドロックしない。
	std::vector<GameObject*> snapshot;
	{
		std::lock_guard lk(_mtx);
		snapshot.reserve(_objects.size());
		for (auto& up : _objects) {
			GameObject* obj = up.get();
			if (obj && obj->IsActive()) {
				snapshot.push_back(obj);
			}
		}
	}
	const size_t count = snapshot.size();
	if (count == 0) return;

	ThreadPool::Instance().ParallelFor(0, count, [&](size_t i) {
		snapshot[i]->Update(dtSec);
	}, 4);
}

void ObjectManager::DrawAll() {
	std::lock_guard lk(_mtx);
	for (auto& up : _objects) {
		GameObject* obj = up.get();
		if (!obj) continue;
		if (!obj->IsActive()) continue;
		obj->Draw();
	}
}

GameObject* ObjectManager::FindById(int id) const {
	std::lock_guard lk(_mtx);
	auto it = std::find_if(_objects.begin(), _objects.end(),
		[id](const ObjectPool::UniquePtr& up) { return up && up->GetId() == id; });
	return (it == _objects.end()) ? nullptr : it->get();
}

#ifdef _DEBUG
void ObjectManager::DebugDraw(int x, int y) const {
	std::lock_guard lk(_mtx);

	const int leftX = x;
	const int rightX = x + 360;
	const int lineH = 16;
	int leftY = y;
	int rightY = y;

	DrawFormatString(leftX, leftY, GetColor(255, 255, 0), "[ObjectManager] 現在オブジェクト数: %d", (int)_objects.size());
	leftY += lineH;
	DrawFormatString(leftX, leftY, GetColor(255, 255, 0), "[ObjectManager] 取得数(現状): %d", (int)_debugTotalAcquire);
	leftY += lineH;
	DrawFormatString(leftX, leftY, GetColor(255, 255, 0), "[ObjectManager] 生成数(Factory): %d", (int)_debugTotalCreated);
	leftY += lineH;
	DrawFormatString(leftX, leftY, GetColor(255, 255, 0), "[ObjectManager] 累計削除数(破棄数): %d", (int)_debugTotalDeleted);
	leftY += lineH;
	DrawFormatString(leftX, leftY, GetColor(255, 255, 0), "[ObjectManager] 現在SceneId: %d", _currentSceneId);
	leftY += lineH;
	DrawFormatString(leftX, leftY, GetColor(255, 255, 0), "[ObjectManager] プール数: %d", (int)_pools.size());

	DrawFormatString(rightX, rightY, GetColor(255, 255, 0), "[Pool] 未使用ストック");
	rightY += lineH;
	for (const auto& [key, pool] : _pools) {
		const size_t freeCount = pool ? pool->Size() : 0;
		DrawFormatString(rightX, rightY, GetColor(200, 255, 200), "%s : %d", key.c_str(), (int)freeCount);
		rightY += lineH;
	}
}
#endif