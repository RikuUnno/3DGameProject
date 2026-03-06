// Colliderの基底クラス
// 派生先で具体的な形状・判定情報を持つ
// 移動/回転/拡縮は自身では行わない
// GameObjectのTransformに依存している
#pragma once
#include "DXLib.h"
#include "ColliderType.h"
#include "LayerMask.h"

// 前方宣言（必要なときに GameObject と接続）
class GameObject;

class Collider {
public:
	Collider() = default;
	virtual ~Collider() = default;

public:
	// 所有者（Transform参照/Active判定用）
	GameObject* owner = nullptr;

	// 子Colliderのイベントを owner に送るか
	// - true: 従来通り owner->OnCollisionXXX / OnTriggerXXX を呼ぶ
	// - false: Collider自身の OnCollisionXXX / OnTriggerXXX のみ呼ぶ（独立運用向け）
	bool sendEventsToOwner = true;

	// 子Colliderのイベントを owner の親GameObject にも伝えるか
	// - true: owner の Transform に親がある場合、その親GameObject にもイベントを送る
	// - false: owner までで止める
	bool bubbleEventsToParentOwner = false;

	// コライダー有効/スリープ（プール待機中・非アクティブ中は false にする想定）
	bool IsEnabled() const noexcept { return _enabled; }
	void SetEnabled(bool enabled) noexcept { _enabled = enabled; }

	// Trigger（押し戻しはしない）
	bool isTrigger = false;

	// Layer/Mask（LayerMask.h の定義を使用）
	int layer = layerMask::DEFAULT;    // デフォルトレイヤー
	int mask = mask::ALL;              //すべてのレイヤーと当たる

	// シーンIDフィルタ対象か（trueなら現在シーン以外とは判定しない）
	//ほとんどのオブジェクトは true のままでOK
	bool useSceneFilter = true;

public:
	// --- CCD / 高速移動検出設定 ---
	// enableCCD: true のときは常にスイープAABBを使用（広義の衝突検出を有効にする）
	// ccdDistanceThreshold: フレーム間の速度（ワールド単位/秒）がこの値を超える場合にスイープを使う
	// 具体的には Time::Instance().GetDeltaTime() を用いて、
	// speed = (center displacement) / deltaTime として比較します。
	bool enableCCD = false;
	float ccdDistanceThreshold = 1.0f; // 単位: ワールド距離/秒 (速度)

public:
	// コライダー種
	enum class Kind {
		AABB,
		Sphere,
		Capsule,
		Box, // OBB想定
	};

	// コライダー種別取得
	virtual Kind GetKind() const =0;

	// Broad-phase 用 AABB（ワールド）
	virtual const AABB& GetAABB() const =0;

	// 中心点（ワールド）
	virtual VECTOR GetCenter() const =0;

	// 更新(Transform変更時に呼び出す)
	virtual void UpdateShape() =0;

	// AABB設定（各クラスで実装）
	virtual void SetAABB() {}

public:
	// 衝突判定イベント(実体、押し戻しあり)
	virtual void OnCollisionEnter(Collider* other) {}
	virtual void OnCollisionStay(Collider* other) {}
	virtual void OnCollisionExit(Collider* other) {}

	// トリガーイベント(実体なし)
	virtual void OnTriggerEnter(Collider* other) {}
	virtual void OnTriggerStay(Collider* other) {}
	virtual void OnTriggerExit(Collider* other) {}

public:
	// デバッグ描画
	virtual void DrawDebug() {}
	virtual void DrawDebugAABB() {}

	// デバッグ描画色（0 の場合は各Colliderのデフォルト色）
	void SetDebugColor(unsigned int color) noexcept { _debugColor = color; }
	unsigned int DebugColor() const noexcept { return _debugColor; }
	void ClearDebugColor() noexcept { _debugColor =0; }

private:
	bool _enabled = true;
	unsigned int _debugColor =0;
};
