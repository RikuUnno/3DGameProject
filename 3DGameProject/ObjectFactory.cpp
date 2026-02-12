#include "ObjectFactory.h"
#include "GameObject.h"
#include <mutex>
#include <iostream>

// ObjectFactory はシングルトンでオブジェクト生成責務を担う。
ObjectFactory& ObjectFactory::Instance() noexcept {
	static ObjectFactory inst;
	return inst;
}

// 登録 API
void ObjectFactory::RegisterCreator(const std::string& key, Creator c) {
	if (!c) return;
	_creators[key] = std::move(c);
}

// プロトタイプ登録
void ObjectFactory::RegisterPrototype(const std::string& key, std::unique_ptr<GameObject> prototype) {
	if (!prototype) return;
	_prototypes[key] = std::move(prototype);
}

// 生成 API
std::unique_ptr<GameObject> ObjectFactory::Create(const std::string& key, const VariantMap& params) const {
	auto it = _creators.find(key);
	if (it != _creators.end()) {
		return it->second(params);
	}
	auto pit = _prototypes.find(key);
	if (pit != _prototypes.end() && pit->second) {
		auto clone = pit->second->Clone();
		// params は clone->OnAcquire で適用できる（呼び出し側で呼ぶ）
		return clone;
	}
	// 未登録: null を返す（運用でログ／例外に変更可）
	std::cerr << "ObjectFactory::Create: key not found: " << key << "\n";
	return nullptr;
}

// 利用確認
bool ObjectFactory::IsRegistered(const std::string& key) const {
	return _creators.find(key) != _creators.end() || _prototypes.find(key) != _prototypes.end(); // key が登録されているか
}