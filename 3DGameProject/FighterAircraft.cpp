#include "FighterAircraft.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "CapsuleCollider.h"
#include "ColliderManager.h"
#include "DxLib.h"
#include "KeyInput.h"
#include "LayerMask.h"
#include "PhysicsManager.h"
#include "PhysicsMaterial.h"


// ヘルパー関数群（内部使用）
namespace {
	// VariantMap から float を取得する。キーが存在しない場合や変換に失敗した場合は fallback を返す。
    inline float ParseFloat(const VariantMap& params, const std::string& key, float fallback) {
        auto it = params.find(key);
        if (it == params.end()) return fallback;
        try {
            return std::stof(it->second);
        } catch (...) {
            return fallback;
        }
    }
	// VariantMap から VECTOR を取得する。キーが存在しない場合や変換に失敗した場合は fallback を返す。
    inline VECTOR ParseVector3(const VariantMap& params, const std::string& prefix, const VECTOR& fallback) {
        return VGet(
            ParseFloat(params, prefix + "x", fallback.x),
            ParseFloat(params, prefix + "y", fallback.y),
            ParseFloat(params, prefix + "z", fallback.z)
        );
    }
	// VECTOR の Y 成分を 0 にして平面化する
    inline VECTOR FlattenY(const VECTOR& v) noexcept {
        return VGet(v.x, 0.0f, v.z);
    }
	// VECTOR を正規化する。長さが 0 に近い場合は fallback を返す。
    inline VECTOR SafeNormalize(const VECTOR& v, const VECTOR& fallback) noexcept {
        const float lenSq = v.x * v.x + v.y * v.y + v.z * v.z;
        if (lenSq < 1e-8f) return fallback;
        return VScale(v, 1.0f / std::sqrt(lenSq));
    }
}

// FighterAircraft クラスの実装
FighterAircraft::FighterAircraft() {   
	// デフォルトのコライダー形状を設定
	_capsuleCollider = std::make_unique<CapsuleCollider>(); // カプセルコライダーを作成
	_capsuleCollider->owner = this;                         // コライダーの所有者を設定
	_capsuleCollider->layer = layerMask::PLAYER;            // コライダーのレイヤーをプレイヤーに設定
	_capsuleCollider->mask = mask::ALL;                     // コライダーのマスクをすべてのレイヤーに設定
	_capsuleCollider->isTrigger = false;                    // コライダーをトリガーではなく実体として設定
	_capsuleCollider->sendEventsToOwner = true;             // コライダーのイベントを所有者に送るように設定
	_capsuleCollider->bubbleEventsToParentOwner = false;    // コライダーのイベントを親の所有者にバブルさせないように設定

	// PhysicsBody の初期化
	_physicsBody._owner = this;                             // PhysicsBody の所有者を設定
	_physicsBody.Reset();                                   // PhysicsBody のパラメータをリセット
	_physicsBody._useGravity = true;                        // 重力を有効化
	_physicsBody._freezeRotation = true;                    // 回転を固定
	_physicsBody._linearDamping = 0.0f;                     // 線形減衰を設定
	_physicsBody._angularDamping = 0.05f;                   // 角速度減衰を設定
	_physicsBody._friction = 0.15f;                         // 動摩擦係数を設定
	_physicsBody._restitution = 0.0f;                       // 反発係数を設定
	_physicsBody._material = PhysicsMaterial::Default();    // 物理マテリアルをデフォルトに設定
	_physicsBody._material.friction = 0.15f;                // 物理マテリアルの摩擦係数を設定
	_physicsBody._material.staticFriction = 0.2f;           // 物理マテリアルの静止摩擦係数を設定
	_physicsBody._material.restitution = 0.0f;              // 物理マテリアルの反発係数を設定
	_physicsBody._gravityScale = 0.35f;                     // 重力スケールを設定
	_physicsBody._maxLinearSpeed = 40.0f;                   // 最大線形速度を設定

	// デフォルトパラメータで構成
    ConfigureFromParams_(VariantMap{});
    if (_capsuleCollider) {
		_capsuleCollider->UpdateShape();                    // コライダーの形状を更新
    }
}

// PhysicsBody / Collider を各マネージャーに登録
FighterAircraft::~FighterAircraft() {
    UnregisterFromManagers_();
}

// PhysicsBody / Collider を各マネージャーに登録
Collider* FighterAircraft::GetCollider() const noexcept {
    return _capsuleCollider.get();
}

// PhysicsBody / Collider を各マネージャーに登録
void FighterAircraft::Start() {
    SetActive(true);
    if (_capsuleCollider) {
        _capsuleCollider->UpdateShape();
    }
    RegisterToManagers_();
}

// PhysicsBody / Collider を各マネージャーから登録解除
void FighterAircraft::Update(float dt) {
    if (!IsActive() || dt <= 0.0f) return;

    auto& key = KeyInput::Instance();
    VECTOR wish = VGet(0.0f, 0.0f, 0.0f);
    const VECTOR forward = transform.Forward();
    const VECTOR right = transform.Right();

    if (key.IsKeyInputHeld(KEY_INPUT_W)) {
        wish = VAdd(wish, forward);
    }
    if (key.IsKeyInputHeld(KEY_INPUT_S)) {
        wish = VSub(wish, forward);
    }
    if (key.IsKeyInputHeld(KEY_INPUT_D)) {
        wish = VAdd(wish, right);
    }
    if (key.IsKeyInputHeld(KEY_INPUT_A)) {
        wish = VSub(wish, right);
    }

    if (key.IsKeyInputHeld(KEY_INPUT_SPACE)) {
        wish = VAdd(wish, VGet(0.0f, 1.0f, 0.0f));
    }
    if (key.IsKeyInputHeld(KEY_INPUT_LSHIFT)) {
        wish = VSub(wish, VGet(0.0f, 1.0f, 0.0f));
    }

    const VECTOR flatWish = FlattenY(wish);
    if (flatWish.x != 0.0f || flatWish.y != 0.0f || flatWish.z != 0.0f) {
        wish = SafeNormalize(wish, forward);
    }

    VECTOR vel = _physicsBody._velocity;
    const VECTOR targetVel = VScale(wish, _moveSpeed);
    vel.x = targetVel.x;
    vel.z = targetVel.z;
    vel.y = targetVel.y;
    _physicsBody._velocity = vel;
    _physicsBody.WakeUp();

    if (flatWish.x != 0.0f || flatWish.z != 0.0f) {
        const float yaw = std::atan2(flatWish.x, flatWish.z);
        transform.SetLocalRotation(Quaternion::FromAxisAngleRad(VGet(0.0f, 1.0f, 0.0f), yaw));
    }

    if (_capsuleCollider) {
        _capsuleCollider->UpdateShape();
    }
}

void FighterAircraft::Draw() {
    if (_capsuleCollider) {
        _capsuleCollider->SetDebugColor(GetColor(120, 220, 255));
        _capsuleCollider->DrawDebug();
    }
}

void FighterAircraft::End() {
    UnregisterFromManagers_();
    SetActive(false);
}

void FighterAircraft::OnAcquire(const VariantMap& params) {
    SetActive(true);
    ConfigureFromParams_(params);

    const VECTOR position = ParseVector3(params, "position", VGet(0.0f, 0.0f, 0.0f));
    transform.SetLocalPosition(position);
    transform.SetLocalRotation(Quaternion::Identity());
    transform.SetLocalScale(VGet(1.0f, 1.0f, 1.0f));
    _physicsBody._velocity = VGet(0.0f, 0.0f, 0.0f);
    _physicsBody._angularVelocity = VGet(0.0f, 0.0f, 0.0f);
    _physicsBody.WakeUp();

    if (_capsuleCollider) {
        _capsuleCollider->UpdateShape();
    }
    RegisterToManagers_();
}

void FighterAircraft::OnRelease() {
    UnregisterFromManagers_();
    SetActive(false);
    _physicsBody._velocity = VGet(0.0f, 0.0f, 0.0f);
    _physicsBody._angularVelocity = VGet(0.0f, 0.0f, 0.0f);
}

void FighterAircraft::ConfigureFromParams_(const VariantMap& params) {
    _radius = (std::max)(ParseFloat(params, "radius", _radius), 0.01f);
    _height = (std::max)(ParseFloat(params, "height", _height), 0.05f);
    _moveSpeed = (std::max)(ParseFloat(params, "moveSpeed", _moveSpeed), 0.1f);
    _turnSpeed = (std::max)(ParseFloat(params, "turnSpeed", _turnSpeed), 0.1f);
    const float mass = (std::max)(ParseFloat(params, "mass", 1.5f), 0.1f);
    const float gravityScale = ParseFloat(params, "gravityScale", _physicsBody._gravityScale);
    const float maxLinearSpeed = ParseFloat(params, "maxLinearSpeed", _physicsBody._maxLinearSpeed);

    if (_capsuleCollider) {
        _capsuleCollider->owner = this;
        _capsuleCollider->layer = layerMask::PLAYER;
        _capsuleCollider->mask = mask::ALL;
        _capsuleCollider->_cap.radius = _radius;
        const float halfHeight = (std::max)((_height * 0.5f) - _radius, 0.01f);
        _capsuleCollider->_cap.bottom = VGet(0.0f, -halfHeight, 0.0f);
        _capsuleCollider->_cap.top = VGet(0.0f, halfHeight, 0.0f);
        _capsuleCollider->UpdateShape();
    }

    _physicsBody._owner = this;
    _physicsBody._gravityScale = gravityScale;
    _physicsBody._maxLinearSpeed = maxLinearSpeed;
    _physicsBody._useGravity = true;
    _physicsBody._freezeRotation = true;
    _physicsBody._material = PhysicsMaterial::Default();
    _physicsBody._material.friction = 0.15f;
    _physicsBody._material.staticFriction = 0.2f;
    _physicsBody._material.restitution = 0.0f;
    _physicsBody.SetMass(mass);
    if (_capsuleCollider) {
        _physicsBody.ComputeInertia(_capsuleCollider.get());
    }
}

void FighterAircraft::RegisterToManagers_() {
    if (_registered) return;
    if (_capsuleCollider) {
        ColliderManager::Instance().RegisterCollider(_capsuleCollider.get());
    }
    PhysicsManager::Instance().RegisterBody(&_physicsBody);
    _registered = true;
}

void FighterAircraft::UnregisterFromManagers_() {
    if (!_registered) return;
    if (_capsuleCollider) {
        ColliderManager::Instance().UnregisterCollider(_capsuleCollider.get());
    }
    PhysicsManager::Instance().UnregisterBody(&_physicsBody);
    _registered = false;
}
