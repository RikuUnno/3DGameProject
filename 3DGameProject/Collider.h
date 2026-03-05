// Colliderの基底クラス
//これを継承したクラスで各種コリジョン判定を実装する
// 移動回転拡縮は持たない
// GameObjectのTransformに依存する
#pragma once
#include "DXLib.h"
#include "ColliderType.h"
#include "LayerMask.h"

// 前方宣言（必要なら後で GameObject と接続）
class GameObject;

class Collider {
public:
	Collider() = default;
	virtual ~Collider() = default;

public:
	// 所有者（Transform参照/Active判定用）
	GameObject* owner = nullptr;

	// 子Colliderのイベントを owner(親側) に伝播するか
	// - true: 従来通り owner->OnCollisionXXX / OnTriggerXXX が呼ばれる
	// - false: Collider自身の OnCollisionXXX / OnTriggerXXX のみ呼ばれる（独立処理）
	bool sendEventsToOwner = true;

	// コライダー有効/スリープ（プール待機中・非アクティブ中は false にする想定）
	bool IsEnabled() const noexcept { return _enabled; }
	void SetEnabled(bool enabled) noexcept { _enabled = enabled; }

	// Trigger（物理反応なし）
	bool isTrigger = false;

	// Layer/Mask（LayerMask.h の定義を使用）
	int layer = layerMask::DEFAULT;    // デフォルトレイヤー
	int mask = mask::ALL;              //すべてのレイヤーと当たる

	// シーンIDフィルタ対象か（trueなら現在シーン以外とは判定しない）
	//ほとんどのオブジェクトは true のままでOK
	bool useSceneFilter = true;

public:
	// --- CCD / 高速移動検出設定 ---
	// enableCCD: true のときは常にスイープAABBを使用（連続衝突検出を有効にする）
	// ccdDistanceThreshold: フレーム間の速度（ワールド単位/秒）がこの値を超える場合にスイープを使う
	// 判定には Time::Instance().GetDeltaTime() を用いて、
	// speed = (center displacement) / deltaTime として比較します。
	bool enableCCD = false;
	float ccdDistanceThreshold = 1.0f; // 単位: ワールド長さ/秒 (速度)

public:
	// コライダー種別
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

	// 更新(形状変更時に呼び出す)
	virtual void UpdateShape() =0;

	// AABB設定（派生クラスで実装）
	virtual void SetAABB() {}

public:
	// 当たり判定イベント(反発アリ)
	virtual void OnCollisionEnter(Collider* other) {}
	virtual void OnCollisionStay(Collider* other) {}
	virtual void OnCollisionExit(Collider* other) {}

	// トリガーイベント(反発ナシ)
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
