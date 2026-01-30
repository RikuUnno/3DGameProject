#pragma once

#include <string>
#include <unordered_map>
#include <memory>

#include "Transform.h"

using VariantMap = std::unordered_map<std::string, std::string>; // パラメータマップ定義

class GameObject {
public:
	GameObject() = default;
	virtual ~GameObject() = default;

	// Transform（位置・回転・スケール）
	// - 見やすさ重視で Euler(rad) を保持
	// - 行列生成は Quaternion を使用
	// - 親子関係を見越した local/world キャッシュ
	Transform transform;

	// ライフサイクル
	virtual void Awake() {}
	virtual void Start() {}
	virtual void Update() {}
	virtual void Draw() {}
	virtual void OnDestroy() {}

	// プール/再利用フック
	virtual void OnAcquire(const VariantMap& params) {} //取得時初期化
	virtual void OnRelease() {} //返却時クリア

	// Prototype: 深い複製（派生で実装）
	virtual std::unique_ptr<GameObject> Clone() const { return nullptr; }

	// 再利用設定
	std::string poolKey;

	// 所属情報（シーン終了時の一括回収用）
	int ownerSceneId = -1;

	// ID 管理（ObjectManager の検索/削除用）
	int GetId() const { return id_; }
	void SetId(int id) { id_ = id; }

private:
	int id_ = -1;
};
