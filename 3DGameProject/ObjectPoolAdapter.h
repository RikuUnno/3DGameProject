#pragma once

#include <string>
#include <memory>

#include "PoolTpl2.h"
#include "ObjectFactory.h"
#include "Time.h"

class GameObject;

// ObjectPoolAdapter
// -既存の ObjectFactory を使って PoolTpl2<GameObject, std::string> を動かすためのヘルパー
// - ObjectManager が段階移行できるようにする（方針:3案）
class ObjectPoolAdapter {
public:
	using Key = std::string;
	using Pool = PoolTpl2<GameObject, Key>;

	ObjectPoolAdapter()
		: _pool([](const Key& key) { return ObjectFactory::Instance().Create(key); }) {
	}

	Pool& PoolRef() { return _pool; }
	const Pool& PoolRef() const { return _pool; }

private:
	Pool _pool;
};
