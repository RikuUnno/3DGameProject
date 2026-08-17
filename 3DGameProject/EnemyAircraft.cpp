#include "EnemyAircraft.h"
#include "FighterAircraft.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "BoxCollider.h"
#include "CapsuleCollider.h"
#include "CompoundCollider.h"
#include "ColliderManager.h"
#include "DxLib.h"
#include "LayerMask.h"
#include "PhysicsManager.h"
#include "PhysicsMaterial.h"

// ユーティリティ関数群
namespace {
    inline float ParseFloat(const VariantMap& params, const std::string& key, float fallback) {
        auto it = params.find(key);
        if (it == params.end()) return fallback;
        try { return std::stof(it->second); } catch (...) { return fallback; }
    }

    inline VECTOR ParseVector3(const VariantMap& params, const std::string& prefix, const VECTOR& fallback) {
        return VGet(
            ParseFloat(params, prefix + "x", fallback.x),
            ParseFloat(params, prefix + "y", fallback.y),
            ParseFloat(params, prefix + "z", fallback.z)
        );
    }

    inline float Lerp(float a, float b, float t) noexcept { return a + (b - a) * t; }

    inline float Dot3(const VECTOR& a, const VECTOR& b) noexcept {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
    inline float Len3(const VECTOR& v) noexcept {
        return std::sqrt((std::max)(Dot3(v, v), 0.0f));
    }
    inline VECTOR SafeNormalize(const VECTOR& v, const VECTOR& fallback = VGet(0, 0, 1)) noexcept {
        const float len = Len3(v);
        return (len > 1e-6f) ? VScale(v, 1.0f / len) : fallback;
    }
}

// コンストラクタ
EnemyAircraft::EnemyAircraft(EnemyDifficulty difficulty)
    : _gun(360.0f, 30, 2.2f, 90.0f, 3.0f, layerMask::ENEMY, 6.0f) // 仮初期値（後で SetDifficulty により上書き）
    , _difficulty(difficulty)
{
    // CompoundCollider を生成（Player機と同形状: 胴体Box + 後部Capsule + 左右主翼Box + 垂直尾翼Box）
    auto compound = std::make_unique<CompoundCollider>();
    compound->owner = this;
    compound->layer = layerMask::ENEMY;
    compound->mask  = mask::ALL;
    compound->isTrigger = false;
    compound->useSceneFilter = false; // 本クラスは ObjectManager::Spawn を介さず直接生成されるため、_ownerSceneId によるシーンフィルタを無効化
    compound->sendEventsToOwner = true;
    compound->bubbleEventsToParentOwner = false;

    // 胴体ボックスコライダー
    auto body = std::make_unique<BoxCollider>();
    body->owner = this;
    body->layer = layerMask::ENEMY;
    body->mask  = mask::ALL;
    body->_box.center      = VGet(0.0f, 0.0f, 0.0f);
    body->_box.halfExtents = VGet(_radius, _radius * 0.6f, _height * 0.f);
    body->_box.axisX = VGet(1, 0, 0);
    body->_box.axisY = VGet(0, 1, 0);
    body->_box.axisZ = VGet(0, 0, 1);
    body->UpdateShape();
    _bodyCollider = body.get();
    compound->AddChild(std::move(body));

    // ノーズカプセルコライダー
    auto nose = std::make_unique<CapsuleCollider>();
    nose->owner = this;
    nose->layer = layerMask::ENEMY;
    nose->mask  = mask::ALL;
    nose->_cap.radius = _radius;
    nose->_cap.bottom = VGet(0.0f, 0.0f, _height * 0.5f);
    nose->_cap.top    = VGet(0.0f, 0.0f, _height * 0.5f + _radius * 1.0f);
    nose->UpdateShape();
    _noseCollider = nose.get();
    compound->AddChild(std::move(nose));

    // 右主翼ボックスコライダー
    auto wingRight = std::make_unique<BoxCollider>();
    wingRight->owner = this;
    wingRight->layer = layerMask::ENEMY;
    wingRight->mask  = mask::ALL;
    wingRight->_box.center = VGet(_radius * 2.0f, 0.0f, 0.0f);
    wingRight->_box.halfExtents = VGet(_radius * 1.0f, _radius * 0.1f, _height * 0.2f);
    wingRight->_box.axisX = VGet(1, 0, 0);
    wingRight->_box.axisY = VGet(0, 1, 0);
    wingRight->_box.axisZ = VGet(0, 0, 1);
    wingRight->UpdateShape();
    _wingRightCollider = wingRight.get();
    compound->AddChild(std::move(wingRight));

    // 左主翼ボックスコライダー
    auto wingLeft = std::make_unique<BoxCollider>();
    wingLeft->owner = this;
    wingLeft->layer = layerMask::ENEMY;
    wingLeft->mask  = mask::ALL;
    wingLeft->_box.center = VGet(-_radius * 2.0f, 0.0f, 0.0f);
    wingLeft->_box.halfExtents = VGet(_radius * 1.0f, _radius * 0.1f, _height * 0.2f);
    wingLeft->_box.axisX = VGet(1, 0, 0);
    wingLeft->_box.axisY = VGet(0, 1, 0);
    wingLeft->_box.axisZ = VGet(0, 0, 1);
    wingLeft->UpdateShape();
    _wingLeftCollider = wingLeft.get();
    compound->AddChild(std::move(wingLeft));

    // 垂直尾翼ボックスコライダー
    auto tailVertical = std::make_unique<BoxCollider>();
    tailVertical->owner = this;
    tailVertical->layer = layerMask::ENEMY;
    tailVertical->mask  = mask::ALL;
    tailVertical->_box.center = VGet(0.0f, _radius * 1.5f, _height * 0.5f);
    tailVertical->_box.halfExtents = VGet(_radius * 0.1f, _radius * 0.6f, _height * 0.2f);
    tailVertical->_box.axisX = VGet(1, 0, 0);
    tailVertical->_box.axisY = VGet(0, 1, 0);
    tailVertical->_box.axisZ = VGet(0, 0, 1);
    tailVertical->UpdateShape();
    _tailVerticalCollider = tailVertical.get();
    compound->AddChild(std::move(tailVertical));

    _compoundCollider = std::move(compound);

    // 物理本体の初期化
    _physicsBody._owner          = this;
    _physicsBody.Reset();
    _physicsBody._useGravity     = false;
    _physicsBody._isKinematic    = false;
    _physicsBody._freezeRotation = true;
    _physicsBody._restitution    = 0.0f;
    _physicsBody._friction       = 0.0f;
    _physicsBody._material       = PhysicsMaterial::Default();
    _physicsBody._material.friction    = 0.0f;
    _physicsBody._material.restitution = 0.0f;
    _physicsBody._maxLinearSpeed = _maxSpeed;
    _physicsBody.SetMass(3.0f);

    _currentSpeed = Lerp(_minSpeed, _maxSpeed, 0.6f);
    _hp = _maxHp;

    ConfigureFromParams_(VariantMap{});
    SetDifficulty(_difficulty); // 難易度パラメータを適用（HP/速度/旋回/索敵/銘を上書き）
    if (_compoundCollider) _compoundCollider->UpdateShape();
}

// 難易度を変更し、対応するパラメータを即時反映する
void EnemyAircraft::SetDifficulty(EnemyDifficulty difficulty) noexcept {
    _difficulty = difficulty;
    const EnemyDifficultyParams p = GetEnemyDifficultyParams(difficulty);

    // 索敵（視野角）関連
    _detectionRange = p.detectionRange;
    _detectionHalfAngleCos = std::cos(p.detectionHalfAngleDeg * (DX_PI_F / 180.0f));
    _searchTurnSpeed = p.searchTurnSpeed;

    // 追尾・戦闘関連
    _turnSpeed = p.turnSpeed;
    _minSpeed  = p.minSpeed;
    _maxSpeed  = p.maxSpeed;
    _fireRange = p.fireRange;

    // 体力
    _maxHp = p.maxHp;
    _hp    = (std::min)(_hp, _maxHp) > 0.0f ? (std::min)(_hp, _maxHp) : _maxHp;

    // 武器
    _gun.SetRPM(p.gunRpm);
    _gun.SetAmmoCount(p.gunAmmo);
    _gun.SetReloadTime(p.gunReloadTime);
    _gun.SetBulletSpeed(p.gunBulletSpeed);
    _gun.SetBulletLife(p.gunBulletLife);
    _gun.SetShooterLayer(layerMask::ENEMY); // 敵機の弾はENEMYレイヤーから発射（Playerに命中させるため）

    _physicsBody._maxLinearSpeed = _maxSpeed;
}

// デストラクタ
EnemyAircraft::~EnemyAircraft() {
    UnregisterFromManagers_();
}

// 初期化
void EnemyAircraft::Start() {
    SetActive(true);
    if (_compoundCollider) _compoundCollider->UpdateShape();
    RegisterToManagers_();
}

// AI追尾・攻撃ロジック
void EnemyAircraft::UpdateAI_(float dt) {
	// ターゲットが有効かチェック
	std::shared_ptr<FighterAircraft> target = _target.lock();
	if (!target || target->IsDead()) {
		// ターゲットがいなければその場で旋回して探す
		_hasSpottedPlayer = false;
		Quaternion rot = transform.LocalRotation();
		rot = Quaternion::FromAxisAngleRad(VGet(0, 1, 0), _searchTurnSpeed * dt) * rot;
		transform.SetLocalRotation(rot.Normalized());
		_currentSpeed = Lerp(_currentSpeed, _minSpeed, (std::min)(dt * 2.0f, 1.0f));
		_gun.Update(dt, transform.WorldPosition(), transform.Forward(), false);
		return;
	}

	// ターゲット方向を計算
	const VECTOR selfPos = transform.WorldPosition();
	const VECTOR targetPos = target->transform.WorldPosition();
	const VECTOR toTarget = VSub(targetPos, selfPos);
	const float  dist = Len3(toTarget);
	const VECTOR toTargetDir = SafeNormalize(toTarget);

	// 機首方向（Forward はワールド軸での機首方向を返す）
	const VECTOR noseDir = VScale(transform.Forward(), -1.0f);

	// 円錐視野（難易度に応じた角度）と索敵距離（難易度に応じた距離）による発見判定
	const float dotToTarget = Dot3(noseDir, toTargetDir);
	const bool  withinRange = dist <= _detectionRange;
	const bool  withinCone  = dotToTarget >= _detectionHalfAngleCos;
	const bool  canSeeNow   = withinRange && withinCone;

	// 視野角・範囲内にいる間は発見状態を維持し、外れたら直ちに索敵旋回に戻る（≡擒きやすさ）
	_hasSpottedPlayer = canSeeNow;

	if (!_hasSpottedPlayer) {
		// 未発見 or 見失った: その場で旋回して探す（視野角内に入るまでは追尾しない）
		Quaternion rot = transform.LocalRotation();
		rot = Quaternion::FromAxisAngleRad(VGet(0, 1, 0), _searchTurnSpeed * dt) * rot;
		transform.SetLocalRotation(rot.Normalized());
		_currentSpeed = Lerp(_currentSpeed, _minSpeed, (std::min)(dt * 2.0f, 1.0f));
		_gun.Update(dt, selfPos, transform.Forward(), false);
		return;
	}

	// 機首方向をターゲット方向へ回転させる
	const float cosAngle = (std::max)(-1.0f, (std::min)(1.0f, dotToTarget));
	const float angle = std::acosf(cosAngle);
	if (angle > 1e-4f) {
		VECTOR axis = VCross(noseDir, toTargetDir);
		const float axisLen = Len3(axis);
		if (axisLen > 1e-6f) {
			axis = VScale(axis, 1.0f / axisLen);
			const float maxStep = _turnSpeed * dt;
			const float step = (std::min)(angle, maxStep);
			Quaternion rot = transform.LocalRotation();
			rot = Quaternion::FromAxisAngleRad(axis, step) * rot;
			transform.SetLocalRotation(rot.Normalized());
		}
	}

	const float targetSpeed = (dist < 15.0f) ? _minSpeed : _maxSpeed;
	_currentSpeed = Lerp(_currentSpeed, targetSpeed, (std::min)(dt * 2.0f, 1.0f));

	// 射撃判定
	const bool inRange = dist <= _fireRange;
	const bool aimedAtTarget = dotToTarget > 0.9f;
	const bool trigger = inRange && aimedAtTarget;

	// 射撃位置と方向を計算して GunSystem に渡す
	const VECTOR muzzlePos = VAdd(
		transform.WorldPosition(),
		VScale(transform.Forward(), -(_height * 0.5f + _radius * 1.2f)));
	const VECTOR muzzleDir = VScale(transform.Forward(), -1.0f);
	_gun.Update(dt, muzzlePos, muzzleDir, trigger);
}

// 更新
void EnemyAircraft::Update(float dt) {
    if (!IsActive() || dt <= 0.0f) return;
    if (IsDead()) return;

    UpdateAI_(dt);

    // 常に機首方向へ前進
    _physicsBody._velocity = VScale(transform.Forward(), -_currentSpeed);
    _physicsBody.WakeUp();

    if (_compoundCollider) _compoundCollider->UpdateShape();
}

// 描画
void EnemyAircraft::Draw() {
    if (IsDead()) return;
    if (_bodyCollider) {
        _bodyCollider->SetDebugColor(GetColor(255, 60, 60));  // 胴体を赤で描画（敵識別用）
        _bodyCollider->DrawDebug();
    }
    if (_noseCollider) {
        _noseCollider->SetDebugColor(GetColor(255, 150, 60));
        _noseCollider->DrawDebug();
    }
    if (_wingLeftCollider) {
        _wingLeftCollider->SetDebugColor(GetColor(255, 120, 120));
        _wingLeftCollider->DrawDebug();
    }
    if (_wingRightCollider) {
        _wingRightCollider->SetDebugColor(GetColor(255, 120, 120));
        _wingRightCollider->DrawDebug();
    }
    if (_tailVerticalCollider) {
        _tailVerticalCollider->SetDebugColor(GetColor(255, 220, 100));
        _tailVerticalCollider->DrawDebug();
    }
    _gun.Draw();
}

// 終了
void EnemyAircraft::End() {
    UnregisterFromManagers_();
    SetActive(false);
}

// プールから取得された直後の初期化
void EnemyAircraft::OnAcquire(const VariantMap& params) {
    SetActive(true);
    ConfigureFromParams_(params);

    // "difficulty" パラメータ（"easy"/"normal"/"hard"）があれば反映
    auto itDiff = params.find("difficulty");
    if (itDiff != params.end()) {
        const std::string& v = itDiff->second;
        if (v == "easy")        SetDifficulty(EnemyDifficulty::Easy);
        else if (v == "hard")   SetDifficulty(EnemyDifficulty::Hard);
        else                    SetDifficulty(EnemyDifficulty::Normal);
    }

    const VECTOR pos = ParseVector3(params, "position", VGet(0.0f, 0.0f, 0.0f));
    transform.SetLocalPosition(pos);
    transform.SetLocalRotation(Quaternion::Identity());
    transform.SetLocalScale(VGet(1.0f, 1.0f, 1.0f));

    _hp = _maxHp;
    _hasSpottedPlayer = false;
    _currentSpeed = Lerp(_minSpeed, _maxSpeed, 0.6f);
    _physicsBody._velocity        = VGet(0.0f, 0.0f, 0.0f);
    _physicsBody._angularVelocity = VGet(0.0f, 0.0f, 0.0f);
    _physicsBody.WakeUp();

    if (_compoundCollider) _compoundCollider->UpdateShape();
    RegisterToManagers_();
}

// プールに返却される直前の後片付け
void EnemyAircraft::OnRelease() {
    UnregisterFromManagers_();
    SetActive(false);
    _physicsBody._velocity        = VGet(0.0f, 0.0f, 0.0f);
    _physicsBody._angularVelocity = VGet(0.0f, 0.0f, 0.0f);
}

// ダメージを受ける
void EnemyAircraft::TakeDamage(float amount, GameObject* /*instigator*/) {
    if (IsDead()) return;
    _hp = (std::max)(0.0f, _hp - amount);
}

// 外部パラメータから形状・物理値を構築
void EnemyAircraft::ConfigureFromParams_(const VariantMap& params) {
    _radius   = (std::max)(ParseFloat(params, "radius",   _radius),   0.01f);
    _height   = (std::max)(ParseFloat(params, "height",   _height),   0.05f);
    _minSpeed = (std::max)(ParseFloat(params, "minSpeed", _minSpeed), 0.1f);
    _maxSpeed = (std::max)(ParseFloat(params, "maxSpeed", _maxSpeed), _minSpeed + 0.1f);
    _turnSpeed = (std::max)(ParseFloat(params, "turnSpeed", _turnSpeed), 0.1f);
    _maxHp    = (std::max)(ParseFloat(params, "maxHp", _maxHp), 1.0f);
    _detectionRange = (std::max)(ParseFloat(params, "detectionRange", _detectionRange), 1.0f);
    const float mass = (std::max)(ParseFloat(params, "mass", 3.0f), 0.1f);

    if (_bodyCollider) {
        _bodyCollider->_box.center      = VGet(0.0f, 0.0f, 0.0f);
        _bodyCollider->_box.halfExtents = VGet(_radius, _radius * 0.6f, _height * 0.5f);
        _bodyCollider->UpdateShape();
    }
    if (_noseCollider) {
        _noseCollider->_cap.radius = _radius;
        _noseCollider->_cap.bottom = VGet(0.0f, 0.0f, _height * 0.5f);
        _noseCollider->_cap.top    = VGet(0.0f, 0.0f, _height * 0.5f + _radius * 1.5f);
        _noseCollider->UpdateShape();
    }
    if (_wingLeftCollider) {
        _wingLeftCollider->_box.center = VGet(-_radius * 2.0f, 0.0f, 0.0f);
        _wingLeftCollider->_box.halfExtents = VGet(_radius * 1.0f, _radius * 0.1f, _height * 0.2f);
        _wingLeftCollider->UpdateShape();
    }
    if (_wingRightCollider) {
        _wingRightCollider->_box.center = VGet(_radius * 2.0f, 0.0f, 0.0f);
        _wingRightCollider->_box.halfExtents = VGet(_radius * 1.0f, _radius * 0.1f, _height * 0.2f);
        _wingRightCollider->UpdateShape();
    }
    if (_tailVerticalCollider) {
        _tailVerticalCollider->_box.center = VGet(0.0f, _radius * 1.5f, _height * 0.5f);
        _tailVerticalCollider->_box.halfExtents = VGet(_radius * 0.1f, _radius * 0.6f, _height * 0.2f);
        _tailVerticalCollider->UpdateShape();
    }
    if (_compoundCollider) {
        _compoundCollider->owner = this;
        _compoundCollider->layer = layerMask::ENEMY;
        _compoundCollider->mask  = mask::ALL;
        _compoundCollider->UpdateShape();
    }

    _physicsBody._owner          = this;
    _physicsBody._useGravity     = false;
    _physicsBody._isKinematic    = false;
    _physicsBody._freezeRotation = true;
    _physicsBody._linearDamping  = 0.0f;
    _physicsBody._maxLinearSpeed = _maxSpeed;
    _physicsBody._material       = PhysicsMaterial::Default();
    _physicsBody._material.friction    = 0.0f;
    _physicsBody._material.restitution = 0.0f;
    _physicsBody.SetMass(mass);
    if (_compoundCollider) _physicsBody.ComputeInertia(_compoundCollider.get());
}

// 物理本体・コライダーの登録
void EnemyAircraft::RegisterToManagers_() {
    if (_registered) return;
    if (_compoundCollider) ColliderManager::Instance().RegisterCollider(_compoundCollider.get());
    PhysicsManager::Instance().RegisterBody(&_physicsBody);
    _registered = true;
}

// 物理本体・コライダーの登録解除
void EnemyAircraft::UnregisterFromManagers_() {
    if (!_registered) return;
    if (_compoundCollider) ColliderManager::Instance().UnregisterCollider(_compoundCollider.get());
    PhysicsManager::Instance().UnregisterBody(&_physicsBody);
    _registered = false;
}
