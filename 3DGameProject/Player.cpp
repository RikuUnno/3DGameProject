#include "Player.h"

#include <algorithm>
#include <cmath>

#include "CapsuleCollider.h"
#include "Collider.h"
#include "ColliderManager.h"
#include "LayerMask.h"
#include "PhysicsManager.h"
#include "PhysicsMaterial.h"
#include "KeyInput.h"
#include "DxLib.h"

namespace {
    inline float Dot3(const VECTOR& a, const VECTOR& b) noexcept { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline float LenSq(const VECTOR& v) noexcept { return Dot3(v, v); }
    inline VECTOR FlattenY(const VECTOR& v) noexcept { return VGet(v.x, 0.0f, v.z); }
    inline VECTOR SafeNormalize(const VECTOR& v, const VECTOR& fallback) noexcept {
        const float l2 = LenSq(v);
        if (l2 < 1e-8f) return fallback;
        return VScale(v, 1.0f / std::sqrt(l2));
    }
}

Player::Player() {
    transform.SetOwner(this);

    // カプセルコライダーを構築（owner ローカルの端点として高さを設定）。
    _capsule = std::make_unique<CapsuleCollider>();
    _capsule->owner = this;
    _capsule->layer = layerMask::PLAYER;
    // ステージ（MeshCollider）は DEFAULT レイヤーに登録されているため、
    // 確実に衝突させるためマスクは ALL にしておく。
    _capsule->mask  = mask::ALL;

    const float half = (std::max)(_height * 0.5f - _radius, 0.01f);
    _capsule->_cap.radius = _radius;
    _capsule->_cap.bottom = VGet(0.0f, -half, 0.0f);
    _capsule->_cap.top    = VGet(0.0f,  half, 0.0f);
    _capsule->UpdateShape();

    // 物理ボディの初期設定。プレイヤーは転倒させたくないので回転を凍結する。
    _physicsBody._owner = this;
    _physicsBody.Reset();
    _physicsBody._useGravity = true;
    _physicsBody._freezeRotation = true;      // 直立を維持（カプセルが倒れない）
    _physicsBody._linearDamping = 0.0f;
    _physicsBody._friction = 0.6f;
    _physicsBody._restitution = 0.0f;
    _physicsBody._material = PhysicsMaterial::Default();
    _physicsBody._material.friction = 0.6f;
    _physicsBody._material.staticFriction = 0.7f;
    _physicsBody._material.restitution = 0.0f;
    _physicsBody._maxLinearSpeed = 40.0f;
    _physicsBody.SetMass(70.0f);
    _physicsBody.ComputeInertia(_capsule.get());
}

Player::~Player() {
    UnregisterFromManagers_();
}

Collider* Player::GetCollider() const noexcept {
    return _capsule.get();
}

void Player::SetMoveBasis(const VECTOR& forward, const VECTOR& right) noexcept {
    _moveForward = SafeNormalize(FlattenY(forward), VGet(0, 0, 1));
    _moveRight   = SafeNormalize(FlattenY(right),   VGet(1, 0, 0));
}

void Player::Spawn(const VECTOR& position) {
    SetActive(true);
    transform.SetParent(nullptr);
    transform.SetLocalPosition(position);
    transform.SetLocalRotation(Quaternion::Identity());
    transform.SetLocalScale(VGet(1.0f, 1.0f, 1.0f));

    _physicsBody._velocity = VGet(0, 0, 0);
    _physicsBody._angularVelocity = VGet(0, 0, 0);
    _physicsBody.WakeUp();

    if (_capsule) _capsule->UpdateShape();
    RegisterToManagers_();
}

void Player::Despawn() {
    UnregisterFromManagers_();
    SetActive(false);
}

void Player::RegisterToManagers_() {
    if (_registered) return;
    if (_capsule) ColliderManager::Instance().RegisterCollider(_capsule.get());
    PhysicsManager::Instance().RegisterBody(&_physicsBody);
    _registered = true;
}

void Player::UnregisterFromManagers_() {
    if (!_registered) return;
    if (_capsule) ColliderManager::Instance().UnregisterCollider(_capsule.get());
    PhysicsManager::Instance().UnregisterBody(&_physicsBody);
    _registered = false;
}

void Player::OnCollisionEnter(Collider* self, Collider* other) {
    OnCollisionStay(self, other);
}

// 非トリガーと接触している間は「接触あり」を立てる。
// ステージは MeshCollider のため GetCenter() が足元と一致せず、相手中心での
// 上下判定は使えない。そこで「接触している かつ 鉛直速度がほぼ無い（落下が
// 止められている）」ことをもって接地とみなす（Update 側で判定）。
void Player::OnCollisionStay(Collider* self, Collider* other) {
    if (!self || !other) return;
    if (other->isTrigger) return;
    _touching = true;
}

void Player::Update(float dt) {
    if (!IsActive()) return;

    auto& key = KeyInput::Instance();

    // --- 接地判定 ---
    // 接触があり、かつ鉛直速度がほぼ無い（落下が支えられている）なら接地。
    // ジャンプ直後（上昇中）は接地扱いしない。
    const float vy = _physicsBody._velocity.y;
    if (_touching && vy <= 0.5f) {
        _grounded = true;
        _groundedTimer = 0.0f;
    }

    // --- 水平移動 ---
    VECTOR wish = VGet(0, 0, 0);
    if (key.IsKeyInputHeld(KEY_INPUT_W)) wish = VAdd(wish, _moveForward);
    if (key.IsKeyInputHeld(KEY_INPUT_S)) wish = VSub(wish, _moveForward);
    if (key.IsKeyInputHeld(KEY_INPUT_D)) wish = VAdd(wish, _moveRight);
    if (key.IsKeyInputHeld(KEY_INPUT_A)) wish = VSub(wish, _moveRight);

    const bool hasInput = LenSq(wish) > 1e-6f;
    if (hasInput) wish = SafeNormalize(wish, _moveForward);

    // 水平速度を目標速度へ更新（鉛直速度は重力・ジャンプに任せる）。
    VECTOR vel = _physicsBody._velocity;
    const VECTOR targetHoriz = VScale(wish, hasInput ? moveSpeed : 0.0f);

    // 接地時はキビキビ、空中は緩やかに加速（簡易エアコントロール）。
    const float accel = _grounded ? 60.0f : 12.0f;
    VECTOR horiz = VGet(vel.x, 0.0f, vel.z);
    VECTOR diff  = VSub(targetHoriz, horiz);
    const float maxStep = accel * dt;
    if (LenSq(diff) > maxStep * maxStep) diff = VScale(SafeNormalize(diff, VGet(0, 0, 0)), maxStep);
    horiz = VAdd(horiz, diff);
    vel.x = horiz.x;
    vel.z = horiz.z;

    // --- ジャンプ ---
    if (_grounded && key.IsKeyInputTrigger(KEY_INPUT_SPACE)) {
        vel.y = jumpSpeed;
        _grounded = false;
        _groundedTimer = 1.0f;
    }

    _physicsBody._velocity = vel;
    if (hasInput || std::fabs(vel.y) > 0.01f) _physicsBody.WakeUp();

    // 進行方向へ向く（水平速度がある程度あるとき）。
    const VECTOR flatVel = FlattenY(vel);
    if (LenSq(flatVel) > 0.25f) {
        const VECTOR dir = SafeNormalize(flatVel, _moveForward);
        const float yaw = std::atan2(dir.x, dir.z);
        transform.SetLocalRotation(Quaternion::FromAxisAngleRad(VGet(0, 1, 0), yaw));
    }

    // 接地フラグは毎フレームの接触で再セットされる。猶予時間を超えたら離地とみなす。
    _groundedTimer += dt;
    if (_groundedTimer > 0.15f) _grounded = false;

    // 接触フラグは毎フレームの接触イベントで再セットされるため、ここでクリアする。
    _touching = false;
}

void Player::Draw() {
    if (_capsule) {
        _capsule->SetDebugColor(_grounded ? GetColor(120, 220, 255) : GetColor(255, 200, 120));
        _capsule->DrawDebug();
    }
}
