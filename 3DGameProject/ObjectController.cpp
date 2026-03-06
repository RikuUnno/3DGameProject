#include "ObjectController.h"
#include "ObjectManager.h"
#include "ObjectFactory.h"
#include "GameObject.h"

#include <algorithm>

GameObject* ObjectController::Spawn(const std::string& key, const VariantMap& params) {
	GameObject* obj = ObjectManager::Instance().Spawn(key, params);
	if (obj) _objects.push_back(obj);
	return obj;
}

GameObject* ObjectController::SpawnAuto(const std::string& key, ObjectFactory::Creator creator,
	size_t poolSize, const VariantMap& params) {
	// –¢“o˜^‚È‚çA•K—v‚É‰ž‚¶‚Ä Factory/Pool ‚ð“o˜^‚·‚é
	if (!_registeredKeys.contains(key)) {
		_registeredKeys.insert(key);
		ObjectFactory::Instance().RegisterCreator(key, std::move(creator));
		if (poolSize > 0) {
			ObjectManager::Instance().RegisterPool(key, poolSize);
		}
	}
	return Spawn(key, params);
}

void ObjectController::Release(GameObject* obj) {
	if (!obj) return;
	ObjectManager::Instance().Release(obj);
	_objects.erase(std::remove(_objects.begin(), _objects.end(), obj), _objects.end());
}

void ObjectController::ReleaseAll() {
	for (auto* obj : _objects) {
		ObjectManager::Instance().Release(obj);
	}
	_objects.clear();
}

void ObjectController::UpdateAll() {
	UpdateAll(0.0f);
}

void ObjectController::UpdateAll(float dtSec) {
	for (auto it = _objects.begin(); it != _objects.end();) {
		GameObject* obj = *it;
		if (!obj) {
			it = _objects.erase(it);
			continue;
		}
		if (!obj->IsActive()) {
			++it;
			continue;
		}
		obj->Update(dtSec);
		++it;
	}
}

void ObjectController::DrawAll() {
	for (auto it = _objects.begin(); it != _objects.end();) {
		GameObject* obj = *it;
		if (!obj) {
			it = _objects.erase(it);
			continue;
		}
		if (!obj->IsActive()) {
			++it;
			continue;
		}
		obj->Draw();
		++it;
	}
}
