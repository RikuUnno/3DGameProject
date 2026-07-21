#pragma once
#include <memory>

#include "GameObject.h"
#include "PhysicsBody.h"
#include "BoxCollider.h"
#include "CapsuleCollider.h"
#include "CompoundCollider.h"
#include "GunSystem.h"


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
	Collider* GetCollider() const noexcept;											// コライダーの取得（CompoundCollider）
	CapsuleCollider* GetAfterCollider() const noexcept;								// 後方カプセルコライダーの取得
	BoxCollider* GetBodyCollider() const noexcept;									// 胴体ボックスコライダーの取得
	BoxCollider* GetWingLeftCollider() const noexcept;								// 左主翼ボックスコライダーの取得
	BoxCollider* GetWingRightCollider() const noexcept;								// 右主翼ボックスコライダーの取得
	BoxCollider* GetTailVerticalCollider() const noexcept;							// 垂直尾翼ボックスコライダーの取得S

	PhysicsBody* GetPhysicsBody() noexcept { return &_physicsBody; }				// 物理本体の取得
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
	std::unique_ptr<CompoundCollider> _compoundCollider;	// 複合コライダー（胴体Box + 先頭Capsule）
	BoxCollider*     _bodyCollider = nullptr;				// 胴体ボックスコライダー（CompoundCollider の子）
	CapsuleCollider* _afterCollider = nullptr;				// 後方カプセルコライダー（CompoundCollider の子）
	BoxCollider*	 _wingLeftCollider = nullptr;			// 左主翼ボックスコライダー（CompoundCollider の子）
	BoxCollider*	 _wingRightCollider = nullptr;			// 右主翼ボックスコライダー（CompoundCollider の子）
	BoxCollider*	_tailVerticalCollider = nullptr;		// 垂直尾翼ボックスコライダー（CompoundCollider の子）
		

	// 飛行機の基本パラメータ
	float _radius = 0.525f;			// コライダー半径（ノーズカプセル・胴体Box共通の基準）
	float _height = 1.8f;			// コライダー高さ（胴体Box の前後長）
	float _minSpeed = 8.0f;			// 最低飛行速度（スロットル0時）
	float _maxSpeed = 40.0f;		// 最大飛行速度（スロットル1時）
	float _pitchSpeed = 1.4f;		// ピッチ速度（ラジアン/秒）
	float _yawSpeed = 1.0f;			// ヨー速度（ラジアン/秒）
	float _rollSpeed = 2.0f;		// ロール速度（ラジアン/秒）
	float _throttleSpeed = 0.6f;	// スロットル変化速度（0〜1/秒）
	float _turnSpeed = 3.2f;		// （ConfigureFromParams_ 互換用）

	// 実行時状態
	float _throttle = 0.3f;			// 現在のスロットル（0=最低速、1=最大速）
	float _currentSpeed = 0.0f;		// 現在の飛行速度
	bool _registered = false;		// マネージャー登録済みか

	// 武器システム
	GunSystem _gun;                 // 機銃システム（デフォルト 600rpm / 30発 / リロード1.5秒）
};