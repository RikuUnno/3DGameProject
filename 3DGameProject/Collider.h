// Colliderの基底クラス
// 派生先で具体的な形状・判定を実装
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
	// デストラクタは Collider.cpp で定義し、ColliderManager から自動的に登録解除する
	virtual ~Collider();

public:
	// 所有者（Transform参照/Active判定用）
	GameObject* owner = nullptr;

	// 子Colliderのイベントを owner に送るか
	// - true: 従来通り owner->OnCollisionXXX / OnTriggerXXX を呼ぶ
	// - false: Collider自身の OnCollisionXXX / OnTriggerXXX のみ呼ぶ（独立運用向け）
	bool sendEventsToOwner = true;

	// 子Colliderのイベントを owner の親GameObject にも伝えるか
	// - true: owner の Transform に親がいる場合、その親GameObject にもイベントを送る
	// - false: owner までで止める
	bool bubbleEventsToParentOwner = false;

	// コライダー有効/スリープ（プール待機中・非アクティブ時は false にする想定）
	bool IsEnabled() const noexcept { return _enabled; }
	void SetEnabled(bool enabled) noexcept { _enabled = enabled; }

	// Trigger（押し戻しはしない）
	bool isTrigger = false;

	// Layer/Mask（LayerMask.h の定義を使用）
	int layer = layerMask::DEFAULT;    // デフォルトレイヤー
	int mask = mask::ALL;              //すべてのレイヤーと当たる

	// シーンIDフィルタ対象か（true なら現在シーン以外とは判定しない）
	//ほとんどのオブジェクトは true のままでOK
	bool useSceneFilter = true;

public:
	// Speculative CCD（予測接触検出）有効化フラグ
	bool enableCCD = false;				// true なら Speculative CCD を有効化（高速移動でのトンネルを減らすための予測接触検出）
	float ccdDistanceThreshold = 1.0f;	// 単位: ワールド距離/秒 (速度)

public:
	// コライダー種別
	enum class Kind {
		AABB,
		Sphere,
		Capsule,
		Box, // OBB想定
		HalfPlane, // 半空間コライダー
		Compound,  // 複合コライダー
		Mesh,      // 三角形メッシュコライダー（ステージ等）
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
	virtual void DrawDebug() {}			// デバッグ描画 (線状の形状)
	virtual void DrawDebugAABB() {}		// デバッグ描画（AABBのみ）
	virtual void DrawPrimitive() {}		// デバッグ描画（DXLibのプリミティブ描画）

	// デバッグ描画色（0 の場合は各Colliderのデフォルト色）
	void SetDebugColor(unsigned int color) noexcept { _debugColor = color; }	// 0xAARRGGBB
	unsigned int DebugColor() const noexcept { return _debugColor; }			// 0 の場合は各Colliderのデフォルト色
	void ClearDebugColor() noexcept { _debugColor = 0; }						// デバッグ描画色をクリア（0 にする）

public:
	// 前フレームのAABB（Broad-phase用）
	AABB prevAABB{};			// 前フレームのAABB（Broad-phase用）
	bool hasPrevAABB = false;	// true なら prevAABB が有効

private:
	bool _enabled = true;			// コライダー有効/スリープ（プール待機中・非アクティブ時は false にする想定）
protected: // DebugColor() は派生クラスからもアクセスできるように protected にする
	unsigned int _debugColor = 0;	// デバッグ描画色（0 の場合は各Colliderのデフォルト色を使用）
};
