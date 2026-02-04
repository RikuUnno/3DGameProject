// Colliderの基底クラス
// これを継承したクラスで各種コリジョン判定を実装する
// 移動回転拡縮は持たない
// GameObjectのTransformに依存する
#pragma once
#include "DXLib.h"

class Collider {
public:// コンストラクタ/デストラクタ
	// (仮)
	Collider() = default;
	virtual ~Collider() = default;
protected: 
	// AABB設定
	virtual void SetAABB() {} // AABB設定

protected:
	// 中心点を求める(ワールド)
	virtual VECTOR GetCenter() const { return VECTOR(); }

public:
	// デバッグ描画
	virtual void DrawDebug() {}		// 本体デバッグ描画
	virtual void DrawDebugAABB() {} // AABBデバッグ描画

};
