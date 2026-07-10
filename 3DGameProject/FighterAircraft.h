#pragma once
#include <memory>

#include "GameObject.h"
#include "PhysicsBody.h"
#include "CapsuleCollider.h"


// MiniGame1のプレイヤー機体クラス
class FighterAircraft : public GameObject
{
public:
	// コンストラクタ/デストラクタ
	FighterAircraft();
	virtual ~FighterAircraft() override;

	// コピー禁止
	FighterAircraft(const FighterAircraft&) = delete;
	

	void Start() override;				// 初期化（Awake 後、最初の Update 前に呼ばれる）
	void Update(float dt) override;		// 更新（毎フレーム呼ばれる）
	void Draw() override;				// 描画（毎フレーム呼ばれる）
	void End() override;				// 終了（破棄前に呼ばれる）

	// Pool から取得/返却される時の初期化・後片付け
	void OnAcquire(const VariantMap& params) override;
	void OnRelease() override;

	// 現在使用中の Collider / PhysicsBody 取得
	Collider* GetCollider() const noexcept;									// コライダーの取得
	PhysicsBody* GetPhysicsBody() noexcept { return &_physicsBody; }			// 物理本体の取得
	const PhysicsBody* GetPhysicsBody() const noexcept { return &_physicsBody; }	// 物理本体の取得

	// 外部パラメータから形状・物理値を構築
	void ConfigureFromParams_(const VariantMap& params);	// 物理パラメータの構築

private:
	// 物理本体・コライダーの登録/登録解除
	void RegisterToManagers_();		// PhysicsBody / Collider を各マネージャーに登録
	void UnregisterFromManagers_();	// PhysicsBody / Collider を各マネージャーから登録解除	

private:
	// 物理本体
	PhysicsBody _physicsBody{};

	// コライダー本体
	std::unique_ptr<CapsuleCollider> _capsuleCollider;	// カプセルコライダー

	// 飛行機の基本パラメータ
	float _radius = 0.35f;		// コライダー半径
	float _height = 1.2f;		// コライダー高さ
	float _moveSpeed = 12.0f;	// 移動速度
	float _turnSpeed = 3.2f;	// 回転速度（ラジアン/秒）
	bool _registered = false;	// PhysicsBody / Collider がマネージャーに登録済みか
};