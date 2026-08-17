#pragma once

#include "GameObject.h"
#include "PhysicsBody.h"
#include "CapsuleCollider.h"
#include "LayerMask.h"
#include <memory>

// 戦闘機の機銃弾クラス
// 砲口位置・進行方向を渡して生成し、直進後自動消滅する
class Bullet : public GameObject
{
public:
	// コンストラクタ/デストラクタ
	Bullet();
	virtual ~Bullet() override;

	// コピー禁止
	Bullet(const Bullet&) = delete;
	Bullet& operator=(const Bullet&) = delete;

	// ライフサイクル
	void Start() override;
	void Update(float dt) override;
	void Draw() override;
	void End() override;

	// 命中判定（GameObject::OnTriggerEnter）: 相手が IDamageable ならダメージを与えて消滅
	void OnTriggerEnter(Collider* self, Collider* other) override;

	// 発射設定（砲口位置・方向・速度・寿命・発射者のレイヤー・威力）
	void Fire(const VECTOR& muzzlePos, const VECTOR& direction,
			  float speed = 120.0f, float lifeSec = 3.0f,
			  int shooterLayer = layerMask::PLAYER, float damage = 10.0f);
	// 生存状態の取得
	bool IsAlive() const noexcept { return _alive; }
	// 威力の取得
	float GetDamage() const noexcept { return _damage; }

private:
	// マネージャー登録/解除
    void RegisterToManagers_();
    void UnregisterFromManagers_();

private:
	// 物理本体/コライダー
    PhysicsBody       _physicsBody{};
    std::unique_ptr<CapsuleCollider> _collider;

	// 発射パラメータ
    VECTOR _direction  = { 0.0f, 0.0f, 1.0f };  // 進行方向（正規化済み）
    float  _speed      = 120.0f;                // 速度（単位/秒）
    float  _lifeTimer  = 0.0f;                  // 残り寿命（秒）
    float  _damage     = 10.0f;                 // 命中時のダメージ量
    bool   _alive      = false;                 // 生存フラグ
    bool   _registered = false;                 // マネージャー登録済みフラグ
};
