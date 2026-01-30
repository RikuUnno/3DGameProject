#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cstddef>

#include "ObjectFactory.h" // ObjectFactory::Creator

class GameObject;

class ObjectController {
public:
	using VariantMap = std::unordered_map<std::string, std::string>;

	GameObject* Spawn(const std::string& key, const VariantMap& params = {});

	//1s‚ÅŠ®Œ‹‚·‚é¶¬APIi–¢“o˜^‚È‚ç Factory/Pool “o˜^‚às‚¤j
	// - creator: Factory “o˜^—p‚Ì¶¬ŠÖ”iparams ‚ğó‚¯æ‚ê‚éj
	// - poolSize:0 ‚Ìê‡‚Íƒv[ƒ‹“o˜^‚µ‚È‚¢
	GameObject* SpawnAuto(
		const std::string& key,
		ObjectFactory::Creator creator,
		size_t poolSize,
		const VariantMap& params = {}
	);

	void Release(GameObject* obj);
	void ReleaseAll();

	void UpdateAll();
	void DrawAll();

	size_t Count() const { return objects_.size(); }

private:
	std::unordered_set<std::string> registeredKeys_;
	std::vector<GameObject*> objects_;
};
