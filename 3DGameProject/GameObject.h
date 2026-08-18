#pragma once

#include <string>
#include <unordered_map>
#include <memory>

#include "Transform.h"
#include "ModelPool.h"

using VariantMap = std::unordered_map<std::string, std::string>; // パラメータマップ型

class Collider;

class GameObject {
public:
	GameObject() { transform.SetOwner(this); }
	virtual ~GameObject() = default;

	// 当たり判定などで動かない固定物フラグ（ワールドで固定/追従用）
	bool isStatic = false;

	// Active フラグ
	bool IsActive() const noexcept { return _isActive; }
	void SetActive(bool active) noexcept { _isActive = active; }

	// Transform
	Transform transform;

	// --- Unity風: GameObjectが受け取るコールバック ---
	virtual void OnCollisionEnter(Collider* self, Collider* other) {}
	virtual void OnCollisionStay(Collider* self, Collider* other) {}
	virtual void OnCollisionExit(Collider* self, Collider* other) {}

	virtual void OnTriggerEnter(Collider* self, Collider* other) {}
	virtual void OnTriggerStay(Collider* self, Collider* other) {}
	virtual void OnTriggerExit(Collider* self, Collider* other) {}

	// ライフサイクル
	virtual void Awake() {}					// 生成時（最初に呼ばれる）
	virtual void Start() {}					// 開始時（Awake の後、最初の Update 前に呼ばれる）
	virtual void Update(float /*dt*/) {}	// 更新（毎フレーム呼ばれる）
	virtual void Draw() {}					// 描画（毎フレーム呼ばれる）
	virtual void End() {}					// 終了（破棄前に呼ばれる）
	virtual void OnDestroy() {}			// 破棄（完全終了時に呼ばれる）

	// プール/再利用フック
	virtual void OnAcquire(const VariantMap& params) {}	// プールから取得された時の初期化
	virtual void OnRelease() {}							// プールに返却される直前の後始末

	// Prototype: 複製機能（使う側で実装）未実装なら nullptr を返す
	virtual std::unique_ptr<GameObject> Clone() const { return nullptr; }

	// 再利用設定
	std::string _poolKey;

	// 描画用モデル（ObjectManager::Spawn 時に _poolKey から ModelManager を見て、
	// 必要なら割り当てる）。派生クラスの Draw() で _model->Draw() を呼べばよい。
	// 単一モデル前提（1 オブジェクト = 1 モデル）。
	ModelPool::TypedUniquePtr _model{ nullptr, [](IModel*) {} };
	IModel* GetModel() const noexcept { return _model.get(); }

	// 所属シーンID（シーン終了時の一括解放用）
	int _ownerSceneId = -1;

	// ID 管理（ObjectManager で検索/削除用）
	int GetId() const { return _id; }
	void SetId(int id) { _id = id; }

protected:
	// 共通: OnAcquire 系の初期化
	void PrepareForAcquire_();
	// 共通: OnRelease 系の後始末
	void PrepareForRelease_();
	// 共通: VariantMap から Transform を適用（回転はラジアン）
	void ApplyTransformFromParams_(const VariantMap& params,
		const VECTOR& defaultPos = VGet(0.0f, 0.0f, 0.0f),
		const VECTOR& defaultScale = VGet(1.0f, 1.0f, 1.0f));

	// 共通: VariantMap パースヘルパ
	static float ParseFloatParam_(const VariantMap& params, const char* key, float defaultValue);
	static int ParseIntParam_(const VariantMap& params, const char* key, int defaultValue);
	static bool ParseBoolParam_(const VariantMap& params, const char* key, bool defaultValue);
	static std::string ParseStringParam_(const VariantMap& params, const char* key, const std::string& defaultValue);

private:
	int _id = -1;
	bool _isActive = true;
};
