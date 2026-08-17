#pragma once

// ダメージを受けられるオブジェクトのインターフェース
// FighterAircraft / EnemyAircraft などが実装し、Bullet の命中時に呼び出される
class GameObject;

class IDamageable
{
public:
    virtual ~IDamageable() = default;

    // ダメージを受ける（amount: ダメージ量, instigator: 加害者の GameObject）
    virtual void TakeDamage(float amount, GameObject* instigator) = 0;

    // 死亡しているか
    virtual bool IsDead() const noexcept = 0;
};
