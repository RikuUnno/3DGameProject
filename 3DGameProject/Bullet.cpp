#include "Bullet.h"

#include <algorithm>
#include <cmath>

#include "ColliderManager.h"
#include "PhysicsManager.h"
#include "LayerMask.h"
#include "PhysicsMaterial.h"
#include "IDamageable.h"
#include "DxLib.h"

// コンストラクタ
Bullet::Bullet()
{
    // コライダー設定（細いカプセル：弾頭）
    auto col = std::make_unique<CapsuleCollider>();
    col->owner             = this;
    col->layer             = layerMask::DEFAULT; // レイヤーをデフォルトに設定
    col->mask              = mask::ALL;          // マスクをすべてのレイヤーに設定
    col->isTrigger         = true;               // トリガーとして使用（貫通させる場合はtrue）
    col->sendEventsToOwner = true;
    col->useSceneFilter    = false;         // Bulletは ObjectManager::Spawn を介さず直接生成されるため、_ownerSceneId によるシーンフィルタを無効化
    col->enableCCD         = true;
    col->ccdDistanceThreshold = 0.0f;
    col->_cap.radius = 0.05f;
    col->_cap.bottom = VGet(0.0f, 0.0f,  0.0f);
    col->_cap.top    = VGet(0.0f, 0.0f,  0.3f); // 弾頭の長さ
    col->UpdateShape();
    _collider = std::move(col);

    // 物理設定（重力なし・運動学的）
    _physicsBody._owner       = this;
    _physicsBody._useGravity  = false;
    _physicsBody._isKinematic = true; // 自前で位置を更新する
    _physicsBody._freezeRotation = true;
    _physicsBody.Reset();
}

// デストラクタ
Bullet::~Bullet()
{
	UnregisterFromManagers_();  // マネージャーからの登録解除
}

// 発射処理
void Bullet::Fire(const VECTOR& muzzlePos, const VECTOR& direction,
                  float speed, float lifeSec, int shooterLayer, float damage)
{
    transform.SetLocalPosition(muzzlePos);

    // 方向を正規化して保持
    const float len = std::sqrt(
        direction.x * direction.x +
        direction.y * direction.y +
        direction.z * direction.z);
    _direction = (len > 1e-6f) ? VScale(direction, 1.0f / len) : VGet(0, 0, 1);

    // ローカル +Z 軸を弾の進行方向へ向ける（CapsuleCollider の軸を進行方向に合わせる）
    {
        const VECTOR from = VGet(0.0f, 0.0f, 1.0f);
        const float dot = (std::max)(-1.0f, (std::min)(1.0f,
            from.x * _direction.x + from.y * _direction.y + from.z * _direction.z));

        Quaternion rot = Quaternion::Identity();
        if (dot < -0.9999f) {
            rot = Quaternion::FromAxisAngleRad(VGet(0.0f, 1.0f, 0.0f), DX_PI_F);
        } else if (dot < 0.9999f) {
            VECTOR axis = VCross(from, _direction);
            const float axisLen = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
            if (axisLen > 1e-6f) {
                axis = VScale(axis, 1.0f / axisLen);
                rot = Quaternion::FromAxisAngleRad(axis, std::acosf(dot));
            }
        }
        transform.SetLocalRotation(rot);
    }

    _speed     = speed;
    _lifeTimer = lifeSec;
    _damage    = damage;
    _alive     = true;

    // 発射者のレイヤーに応じて、命中対象（マスク）を切り替える
    // PLAYER が撃てば ENEMY/ENVIRONMENT/GROUND と、それ以外なら逆にフィルタ
    if (_collider) {
        _collider->layer = shooterLayer;
        _collider->mask  = (shooterLayer == layerMask::PLAYER) ? mask::PLAYER : mask::ENEMY;
    }

    SetActive(true);
    if (_collider) _collider->UpdateShape();
    RegisterToManagers_();
}

// ライフサイクル
void Bullet::Start()
{
    SetActive(true);
    if (_collider) _collider->UpdateShape();
    RegisterToManagers_();
}

void Bullet::Update(float dt)
{
    if (!_alive || dt <= 0.0f) return;

    _lifeTimer -= dt;
    if (_lifeTimer <= 0.0f) {
        _alive = false;
        End();
        return;
    }

    // 直線移動
    const VECTOR pos = transform.WorldPosition();
    transform.SetLocalPosition(VAdd(pos, VScale(_direction, _speed * dt)));
    if (_collider) _collider->UpdateShape();
}

void Bullet::Draw()
{
    if (!_alive) return;
    if (_collider) {
        _collider->SetDebugColor(GetColor(255, 255, 80));
        _collider->DrawDebug();
    }
}

// 命中判定（GameObject::OnTriggerEnter）: 相手が IDamageable ならダメージを与えて消滅
void Bullet::OnTriggerEnter(Collider* /*self*/, Collider* other)
{
    if (!_alive || !other) return;

    GameObject* otherOwner = other->owner;
    if (!otherOwner) return;

    if (auto* damageable = dynamic_cast<IDamageable*>(otherOwner)) {
        damageable->TakeDamage(_damage, this);
        _alive = false;
        End();
    }
}

void Bullet::End()
{
    UnregisterFromManagers_();
    SetActive(false);
}

// マネージャー登録/解除
void Bullet::RegisterToManagers_()
{
	if (_registered) return; // すでに登録済みなら何もしない
	// コライダーを ColliderManager に登録
    if (_collider) ColliderManager::Instance().RegisterCollider(_collider.get());
    _registered = true;
}

// マネージャー登録解除
void Bullet::UnregisterFromManagers_()
{
    if (!_registered) return;
    if (_collider) ColliderManager::Instance().UnregisterCollider(_collider.get());
    _registered = false;
}
