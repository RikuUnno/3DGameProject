#pragma once
#include <memory>

#include "GameObject.h"
#include "PhysicsBody.h"
#include "BoxCollider.h"
#include "CapsuleCollider.h"
#include "CompoundCollider.h"
#include "GunSystem.h"
#include "IDamageable.h"
#include "EnemyDifficulty.h"

class FighterAircraft;

// 敵AI機体クラス（MiniGame1用の対戦相手）
// ・プレイヤー機（FighterAircraft）を検知して追尾・攻撃する簡易AI
// ・PhysicsBody / CompoundCollider は FighterAircraft と同等の構成
// ・難易度（EnemyDifficulty）に応じて索敵・追尾・弾幕の各パラメータが変化する
class EnemyAircraft : public GameObject, public IDamageable
{
public:
    // コンストラクタ/デストラクタ
    explicit EnemyAircraft(EnemyDifficulty difficulty = EnemyDifficulty::Normal);
    virtual ~EnemyAircraft() override;

    // コピー禁止
    EnemyAircraft(const EnemyAircraft&) = delete;

    void Start() override;				// 初期化
    void Update(float dt) override;		// 更新
    void Draw() override;				// 描画
    void End() override;				// 終了

    // Pool から取得/返却される時の初期化・後片付け
    void OnAcquire(const VariantMap& params) override;
    void OnRelease() override;

    // AI が追尾するターゲットを設定（weak_ptr で保持し、破棄済みなら自動的に無効化される）
    void SetTarget(const std::shared_ptr<FighterAircraft>& target) noexcept { _target = target; }

    // 難易度を変更（現在のパラメータに即時反映）
    void SetDifficulty(EnemyDifficulty difficulty) noexcept;
    EnemyDifficulty GetDifficulty() const noexcept { return _difficulty; }

    // IDamageable 実装
    void TakeDamage(float amount, GameObject* instigator) override;
    bool IsDead() const noexcept override { return _hp <= 0.0f; }
    float GetHp() const noexcept { return _hp; }
    float GetMaxHp() const noexcept { return _maxHp; }

    // コライダー/物理本体の取得
    Collider* GetCollider() const noexcept { return _compoundCollider.get(); }
    PhysicsBody* GetPhysicsBody() noexcept { return &_physicsBody; }

    // 半径・機体位置の取得（他機体との重なり回避に使用）
    float GetRadius() const noexcept { return _radius; }

    // 外部パラメータから形状・物理値を構築
    void ConfigureFromParams_(const VariantMap& params);

private:
    void RegisterToManagers_();
    void UnregisterFromManagers_();

    void UpdateAI_(float dt);			// 追尾・攻撃AIロジック

private:
    // 物理本体
    PhysicsBody _physicsBody{};

    // コライダー本体（FighterAircraft と同形状: 胴体Box + ノーズCapsule + 左右主翼Box + 垂直尾翼Box）
    std::unique_ptr<CompoundCollider> _compoundCollider;
    BoxCollider*     _bodyCollider = nullptr;
    CapsuleCollider* _noseCollider = nullptr;
    BoxCollider*     _wingLeftCollider = nullptr;
    BoxCollider*     _wingRightCollider = nullptr;
    BoxCollider*     _tailVerticalCollider = nullptr;

    // 基本パラメータ（難易度で上書きされる）
	float _radius = 0.525f;             // 機体の半径（コライダーの半径ではなく、機体モデルの半径）
	float _height = 1.8f;               // 機体の高さ（コライダーの高さではなく、機体モデルの高さ）
	float _minSpeed = 6.0f;             // プレイヤーよりやや遅め
    float _maxSpeed = 28.0f;			// プレイヤーよりやや遅め
    float _turnSpeed = 1.2f;			// 旋回速度（ラジアン/秒）
    float _searchTurnSpeed = 0.8f;		// 索敵中（未発見時）の旋回速度（ラジアン/秒）

    // 実行時状態
    float _currentSpeed = 0.0f;
    bool  _registered = false;

    // 体力
    float _maxHp = 50.0f;
    float _hp = 50.0f;

    // 武器システム（敵用レイヤーで発射）
    GunSystem _gun;

    // AI 関連: 生ポインタではなく weak_ptr にして、ターゲットが破棄済みなら
    // lock() が nullptr を返すため安全にチェックできるようにする。
    std::weak_ptr<FighterAircraft> _target;
    float _fireRange = 60.0f;
    float _fireTimer = 0.0f;

    // 難易度
    EnemyDifficulty _difficulty = EnemyDifficulty::Normal;

    // 索敵（視野角）関連
    float _detectionRange = 70.0f;			// 索敵範囲（この距離を超えると発見できない）
    float _detectionHalfAngleCos = 0.70710678f; // 円錐視野角の半角のcos値（難易度の detectionHalfAngleDeg から算出）
    bool  _hasSpottedPlayer = false;		// プレイヤーを発見済みか（視野角から外れたら再び旋回に戻る）
};
