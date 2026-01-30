#include "ObjectController.h"
#include "ObjectManager.h"
#include "ObjectFactory.h"
#include "GameObject.h"

#include <algorithm>

GameObject* ObjectController::Spawn(const std::string& key, const VariantMap& params) {
	GameObject* obj = ObjectManager::Instance().Spawn(key, params);
	if (obj) objects_.push_back(obj);
	return obj;
}

GameObject* ObjectController::SpawnAuto( const std::string& key, ObjectFactory::Creator creator,
	size_t poolSize, const VariantMap& params) {
	// –¢“o˜^‚È‚çA‚±‚±‚Å Factory/Pool ‚ð“o˜^‚·‚é
	if (!registeredKeys_.contains(key)) {
		registeredKeys_.insert(key);
		ObjectFactory::Instance().RegisterCreator(key, std::move(creator));
		if (poolSize >0) {
			ObjectManager::Instance().RegisterPool(key, poolSize);
		}
	}
	return Spawn(key, params);
}

void ObjectController::Release(GameObject* obj) {
	if (!obj) return;
	ObjectManager::Instance().Release(obj);
	objects_.erase(std::remove(objects_.begin(), objects_.end(), obj), objects_.end());
}

void ObjectController::ReleaseAll() {
	for (auto* obj : objects_) {
		ObjectManager::Instance().Release(obj);
	}
	objects_.clear();
}

void ObjectController::UpdateAll() {
	// Release“™‚Å–³Œø‚É‚È‚Á‚½ŽQÆ‚ð‘|œ‚µ‚È‚ª‚ç Update
	for (auto it = objects_.begin(); it != objects_.end();) {
		GameObject* obj = *it;
		if (!obj) {
			it = objects_.erase(it);
			continue;
		}
		obj->Update();
		++it;
	}
}

void ObjectController::DrawAll() {
	for (auto it = objects_.begin(); it != objects_.end();) {
		GameObject* obj = *it;
		if (!obj) {
			it = objects_.erase(it);
			continue;
		}
		obj->Draw();
		++it;
	}
}
