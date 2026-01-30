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
	creators_[key] = std::move(c);
}

// プロトタイプ登録
void ObjectFactory::RegisterPrototype(const std::string& key, std::unique_ptr<GameObject> prototype) {
	if (!prototype) return;
	prototypes_[key] = std::move(prototype);
}

// 生成 API
std::unique_ptr<GameObject> ObjectFactory::Create(const std::string& key, const VariantMap& params) const {
	auto it = creators_.find(key);
	if (it != creators_.end()) {
		return it->second(params);
	}
	auto pit = prototypes_.find(key);
	if (pit != prototypes_.end() && pit->second) {
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
	return creators_.find(key) != creators_.end() || prototypes_.find(key) != prototypes_.end(); // key が登録されているか
}