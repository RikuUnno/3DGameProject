#pragma once

#include <memory>
#include <string>

#include "LayerMask.h"
#include "PachinkoPhysicsObjectTpl.h"

class CapsuleCollider;

class PachinkoNail : public PachinkoPhysicsObjectTpl {
public:
	// プールキー
	static std::string StaticPoolKey() { return "PachinkoNail"; }

	// コンストラクタ・デストラクタ
	PachinkoNail();
	~PachinkoNail() override;

protected:
	// PachinkoPhysicsObjectTpl の実装
	Collider* GetCollider_() const noexcept override;			// コライダーを返す
	void EnsureCollider_() override;							// コライダーが存在しない場合は作成する
	void ConfigureShape_(const VariantMap& params) override;	// パラメータに基づいて形状を設定する

	bool DefaultStatic_() const noexcept override { return true; }					// 釘は固定物
	int DefaultLayer_() const noexcept override { return layerMask::ENVIRONMENT; }	// 釘は環境レイヤー
	float DefaultMass_() const noexcept override { return 0.0f; }					// 釘は質量なし
	bool DefaultUseGravity_() const noexcept override { return false; }				// 釘は重力なし
	bool DefaultFreezeRotation_() const noexcept override { return true; }			// 釘は回転固定
	bool DefaultCcd_() const noexcept override { return false; }					// 釘はCCDなし
	unsigned int DefaultColor_() const noexcept override;							// 釘のデフォルト描画色
	const char* DefaultMaterial_() const noexcept override { return "metal"; }		// 釘のデフォルトマテリアル

private:
	std::unique_ptr<CapsuleCollider> _capsuleCollider;	// 釘のカプセルコライダー
	float _radius = 0.06f;		// 釘の半径
	float _halfHeight = 0.22f;	// 釘の半高さ（中心から先端までの距離）
};
