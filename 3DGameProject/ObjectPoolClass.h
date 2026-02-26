#pragma once

#include <string>

#include "PoolTpl.h"
#include "ObjectFactory.h"

class GameObject;

// ObjectPoolClass
// - PoolTpl<GameObject, std::string> の具象ラッパー
// - ObjectFactory と連携して GameObject を生成する
// - ObjectManagerから "プール" を切り出して設計を統一するためのクラス
class ObjectPoolClass {
public:
	using Key = std::string;
	using Pool = PoolTpl<GameObject, Key>;
	using UniquePtr = Pool::UniquePtr;

	ObjectPoolClass()
		: _pool([](const Key& key) { return ObjectFactory::Instance().Create(key); }) {
	}

	void Register(const Key& key, size_t maxSize =64) { _pool.Register(key, maxSize); }
	bool IsRegistered(const Key& key) const { return _pool.IsRegistered(key); }

	UniquePtr Acquire(const Key& key, double nowSeconds) { return _pool.Acquire(key, nowSeconds); }
	void Release(const Key& key, GameObject* obj, double nowSeconds) { _pool.Release(key, obj, nowSeconds); }

	size_t FreeCount(const Key& key) const { return _pool.FreeCount(key); }
	bool Clear(const Key& key) { return _pool.Clear(key); }
	size_t TrimUnused(const Key& key, double maxIdleSeconds, double nowSeconds) { return _pool.TrimUnused(key, maxIdleSeconds, nowSeconds); }
	size_t TrimAllUnused(double maxIdleSeconds, double nowSeconds) { return _pool.TrimAllUnused(maxIdleSeconds, nowSeconds); }
	bool Unregister(const Key& key) { return _pool.Unregister(key); }
	void Shutdown() { _pool.Shutdown(); }

private:
	Pool _pool;
};
