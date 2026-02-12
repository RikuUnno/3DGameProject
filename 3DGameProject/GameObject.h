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

	// Active フラグ（無効化中は Update/Draw/Collision 等の対象外にする想定）
	bool IsActive() const noexcept { return _isActive; }
	void SetActive(bool active) noexcept { _isActive = active; }

	// Transform（位置・回転・スケール）
	// - 見やすさ重視で Euler(rad) を保持
	// - 行列生成は Quaternion を使用
	// - 親子関係を見越した local/world キャッシュ
	Transform transform;

	// ライフサイクル
	virtual void Awake() {}			// 生成直後一度だけ呼ばれる
	virtual void Start() {}			// シーン開始時一度だけ呼ばれる
	virtual void Update() {}		// 毎フレーム呼ばれる
	virtual void Draw() {}			// 毎フレーム呼ばれる（描画用）
	virtual void End() {}			// シーン終了時一度だけ呼ばれる
	virtual void OnDestroy() {}		// 破棄直前一度だけ呼ばれる

	// プール/再利用フック
	virtual void OnAcquire(const VariantMap& params) {} //取得時初期化
	virtual void OnRelease() {}							//返却時クリア

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
