#pragma once
#include <string>
#include <unordered_map>
#include <functional>
#include <memory>

class GameObject;
using VariantMap = std::unordered_map<std::string, std::string>;

class ObjectFactory {
public:
	// 生成関数型定義
	using Creator = std::function<std::unique_ptr<GameObject>(const VariantMap&)>; // 生成関数型

	// シングルトンインスタンス取得
	static ObjectFactory& Instance() noexcept;

	// 登録 API
	void RegisterCreator(const std::string& key, Creator c);                                // 関数登録
	void RegisterPrototype(const std::string& key, std::unique_ptr<GameObject> prototype);  // プロトタイプ登録

	// 生成 API（所有権は呼び出し側）
	std::unique_ptr<GameObject> Create(const std::string& key, const VariantMap& params = {}) const;

	// 利用確認
	bool IsRegistered(const std::string& key) const;

private:
	std::unordered_map<std::string, Creator> creators_;                         // key -> Creator
	std::unordered_map<std::string, std::unique_ptr<GameObject>> prototypes_;   // key -> Prototype
};
