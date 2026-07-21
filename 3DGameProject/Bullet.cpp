#include "Bullet.h"

#include "ColliderManager.h"
#include "PhysicsManager.h"
#include "LayerMask.h"
#include "PhysicsMaterial.h"
#include "DxLib.h"

// コンストラクタ
Bullet::Bullet()
{
    // コライダー設定（細いカプセル：弾頭）
    auto col = std::make_unique<CapsuleCollider>();
    col->owner             = this;
    col->layer             = layerMask::PLAYER;
    col->mask              = mask::ALL;
    col->isTrigger         = true;          // トリガーとして使用（貫通させる場合はtrue）
    col->sendEventsToOwner = true;
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
                  float speed, float lifeSec)
{
    transform.SetLocalPosition(muzzlePos);

    // 方向を正規化して保持
    const float len = std::sqrt(
        direction.x * direction.x +
        direction.y * direction.y +
        direction.z * direction.z);
    _direction = (len > 1e-6f) ? VScale(direction, 1.0f / len) : VGet(0, 0, 1);

    _speed     = speed;
    _lifeTimer = lifeSec;
    _alive     = true;

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

void Bullet::End()
{
    UnregisterFromManagers_();
    SetActive(false);
}

// マネージャー登録/解除
void Bullet::RegisterToManagers_()
{
    if (_registered) return;
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
