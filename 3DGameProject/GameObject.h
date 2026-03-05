#pragma once

#include <string>
#include <unordered_map>
#include <memory>

#include "Transform.h"

using VariantMap = std::unordered_map<std::string, std::string>; // パラメータマップ定義

class Collider;

class GameObject {
public:
	GameObject() = default;
	virtual ~GameObject() = default;

	// 押し戻し等で動かない固定物フラグ（ワールド固定/壁扱い）
	bool isStatic = false;

	// Active フラグ
	bool IsActive() const noexcept { return _isActive; }
	void SetActive(bool active) noexcept { _isActive = active; }

	// Transform
	Transform transform;

	// --- Unity風: GameObjectが直接受け取るコールバック ---
	virtual void OnCollisionEnter(Collider* self, Collider* other) {}
	virtual void OnCollisionStay(Collider* self, Collider* other) {}
	virtual void OnCollisionExit(Collider* self, Collider* other) {}

	virtual void OnTriggerEnter(Collider* self, Collider* other) {}
	virtual void OnTriggerStay(Collider* self, Collider* other) {}
	virtual void OnTriggerExit(Collider* self, Collider* other) {}

	// ライフサイクル
	virtual void Awake() {}
	virtual void Start() {}
	virtual void Update() {}
	virtual void Draw() {}
	virtual void End() {}
	virtual void OnDestroy() {}

	// プール/再利用フック
	virtual void OnAcquire(const VariantMap& params) {}
	virtual void OnRelease() {}

	// Prototype: 深い複製（派生で実装）未実装時は nullptr を返す
	virtual std::unique_ptr<GameObject> Clone() const { return nullptr; }

	// 再利用設定
	std::string _poolKey;

	// 所属情報（シーン終了時の一括回収用）
	int _ownerSceneId = -1;

	// ID 管理（ObjectManager の検索/削除用）
	int GetId() const { return _id; }
	void SetId(int id) { _id = id; }

private:
	int _id = -1;
	bool _isActive = true;
};
