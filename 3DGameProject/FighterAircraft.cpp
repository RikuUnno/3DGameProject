#include "FighterAircraft.h"
#include "FighterAircraft.h"
#include <algorithm>
#include <cmath>
#include <string>

#include "BoxCollider.h"
#include "CapsuleCollider.h"
#include "CompoundCollider.h"
#include "ColliderManager.h"
#include "DxLib.h"
#include "KeyInput.h"
#include "LayerMask.h"
#include "PhysicsManager.h"
#include "PhysicsMaterial.h"

// ユーティリティ関数群
namespace {
	// VariantMap から float を取得するユーティリティ関数
    inline float ParseFloat(const VariantMap& params, const std::string& key, float fallback) {
        auto it = params.find(key);
        if (it == params.end()) return fallback;
        try { return std::stof(it->second); } catch (...) { return fallback; }
    }

	// VariantMap から VECTOR を取得するユーティリティ関数
    inline VECTOR ParseVector3(const VariantMap& params, const std::string& prefix, const VECTOR& fallback) {
        return VGet(
            ParseFloat(params, prefix + "x", fallback.x),
            ParseFloat(params, prefix + "y", fallback.y),
            ParseFloat(params, prefix + "z", fallback.z)
        );
    }

	// 線形補間関数
    inline float Lerp(float a, float b, float t) noexcept { return a + (b - a) * t; }
}

// コンストラクタ
FighterAircraft::FighterAircraft() {
	// CompoundCollider を生成（胴体Box + 先頭Capsule を子として持つ）
	auto compound = std::make_unique<CompoundCollider>();
	compound->owner = this;                         // コライダーの所有者を設定
	compound->layer = layerMask::PLAYER;            // コライダーのレイヤーをプレイヤーに設定
	compound->mask  = mask::ALL;                    // コライダーのマスクをすべてのレイヤーに設定
	compound->isTrigger = false;                    // コライダーをトリガーではなく実体として設定
	compound->sendEventsToOwner = true;             // イベントを所有者に送信
	compound->bubbleEventsToParentOwner = false;    // イベントを親の所有者にバブルしない

	// 胴体ボックスコライダー（長方形、機体の中心に配置）
	auto body = std::make_unique<BoxCollider>();
	body->owner = this;
	body->layer = layerMask::PLAYER;
	body->mask  = mask::ALL;
	body->_box.center     = VGet(0.0f, 0.0f, 0.0f);      // 機体中心
	body->_box.halfExtents = VGet(_radius, _radius * 0.6f, _height * 0.f); // 幅・高さ・前後長
	body->_box.axisX = VGet(1, 0, 0);
	body->_box.axisY = VGet(0, 1, 0);
	body->_box.axisZ = VGet(0, 0, 1);
	body->UpdateShape();
	_bodyCollider = body.get();                     // 生ポインタを保存（CompoundCollider が所有権を持つ）
	compound->AddChild(std::move(body));

	// カプセルコライダー（アフターバーナー ,進行方向後方に配置）
	auto nose = std::make_unique<CapsuleCollider>();
	nose->owner = this;
	nose->layer = layerMask::PLAYER;
	nose->mask  = mask::ALL;
	nose->_cap.radius = _radius;                            // ノーズの半径
	nose->_cap.bottom = VGet(0.0f, 0.0f, _height * 0.5f);  // 胴体前端から始まる
	nose->_cap.top    = VGet(0.0f, 0.0f, _height * 0.5f + _radius * 1.0f); // 先端
	nose->UpdateShape();
	_afterCollider = nose.get();                     // 生ポインタを保存
	compound->AddChild(std::move(nose));

	// ボックスコライダー（主翼右側）
	auto wingRight = std::make_unique<BoxCollider>();
	wingRight->owner = this;
	wingRight->layer = layerMask::PLAYER;
	wingRight->mask = mask::ALL;
	wingRight->_box.center = VGet(_radius * 2.0f, 0.0f, 0.0f); // 右側に配置
	wingRight->_box.halfExtents = VGet(_radius * 1.0f, _radius * 0.1f, _height * 0.2f); // 主翼のサイズ
	wingRight->_box.axisX = VGet(1, 0, 0);
	wingRight->_box.axisY = VGet(0, 1, 0);
	wingRight->_box.axisZ = VGet(0, 0, 1);
	wingRight->UpdateShape();
	_wingRightCollider = wingRight.get();                     // 生ポインタを保存（CompoundCollider が所有権を持つ）
	compound->AddChild(std::move(wingRight));

	// ボックスコライダー（主翼左側）
	auto wingLeft = std::make_unique<BoxCollider>();
	wingLeft->owner = this;
	wingLeft->layer = layerMask::PLAYER;
	wingLeft->mask = mask::ALL;
	wingLeft->_box.center = VGet(-_radius * 2.0f, 0.0f, 0.0f); // 左側に配置
	wingLeft->_box.halfExtents = VGet(_radius * 1.0f, _radius * 0.1f, _height * 0.2f); // 主翼のサイズ
	wingLeft->_box.axisX = VGet(1, 0, 0);
	wingLeft->_box.axisY = VGet(0, 1, 0);
	wingLeft->_box.axisZ = VGet(0, 0, 1);
	wingLeft->UpdateShape();
	_wingLeftCollider = wingLeft.get();                     // 生ポインタを保存（CompoundCollider が所有権を持つ）
	compound->AddChild(std::move(wingLeft));

	// ボックスコライダー（垂直尾翼）
	auto tailVertical = std::make_unique<BoxCollider>();
	tailVertical->owner = this;
	tailVertical->layer = layerMask::PLAYER;
	tailVertical->mask = mask::ALL;
	tailVertical->_box.center = VGet(0.0f, _radius * 1.5f, _height * 0.5f); // 上側に配置
	tailVertical->_box.halfExtents = VGet(_radius * 0.1f, _radius * 0.6f, _height * 0.2f); // 垂直尾翼のサイズ
	tailVertical->_box.axisX = VGet(1, 0, 0);
	tailVertical->_box.axisY = VGet(0, 1, 0);
	tailVertical->_box.axisZ = VGet(0, 0, 1);
	tailVertical->UpdateShape();
	_tailVerticalCollider = tailVertical.get();                     // 生ポインタを保存（CompoundCollider が所有権を持つ）
	compound->AddChild(std::move(tailVertical));

	_compoundCollider = std::move(compound);        // CompoundCollider を保持

	// 物理本体の初期化
	_physicsBody._owner = this;                                     // 物理本体の所有者を設定
	_physicsBody.Reset();                                           // 物理本体をリセット
	_physicsBody._useGravity = false;                               // 重力の影響を受けない
	_physicsBody._isKinematic = false;                              // Kinematic 無効化（物理挙動あり）
	_physicsBody._freezeRotation = true;                            // 回転を固定（慣性テンソルを無限大にして回転の物理を無効化）
	_physicsBody._restitution = 0.0f;                               // 反発係数を0に設定（完全非弾性）
	_physicsBody._friction = 0.0f;                                  // 動摩擦係数を0に設定（無摩擦）
	_physicsBody._material = PhysicsMaterial::Default();            // 物理マテリアルをデフォルトに設定
	_physicsBody._material.friction = 0.0f;                         // 動摩擦係数を0に設定（無摩擦）
	_physicsBody._material.restitution = 0.0f;                      // 反発係数を0に設定（完全非弾性）
	_physicsBody._maxLinearSpeed = _maxSpeed;                       // 最大線形速度を最大飛行速度に設定
	_physicsBody.SetMass(3.0f);

	// 初期状態のスロットルと速度を設定
	_throttle = 0.3f;                                       // 初期スロットルを0.3に設定
	_currentSpeed = Lerp(_minSpeed, _maxSpeed, _throttle);  // 初期速度をスロットルに応じて設定

	// デフォルトパラメータで構成
	ConfigureFromParams_(VariantMap{});                      // デフォルトパラメータで構成
	if (_compoundCollider) _compoundCollider->UpdateShape(); // コライダー形状を更新
}

// デストラクタ
FighterAircraft::~FighterAircraft() {
	UnregisterFromManagers_();
}

// 現在使用中の Collider 取得
Collider* FighterAircraft::GetCollider() const noexcept {
	return _compoundCollider.get();
}

// 胴体ボックスコライダーを取得
BoxCollider* FighterAircraft::GetBodyCollider() const noexcept {
	return _bodyCollider;
}

// 後方カプセルコライダーを取得
CapsuleCollider* FighterAircraft::GetAfterCollider() const noexcept {
	return _afterCollider;
}

// 左主翼ボックスコライダーを取得
BoxCollider* FighterAircraft::GetWingLeftCollider() const noexcept {
	return _wingLeftCollider;
}

// 右主翼ボックスコライダーを取得
BoxCollider* FighterAircraft::GetWingRightCollider() const noexcept {
	return _wingRightCollider;
}

// 垂直尾翼ボックスコライダーを取得
BoxCollider* FighterAircraft::GetTailVerticalCollider() const noexcept {
	return _tailVerticalCollider;
}

// 物理本体・コライダーの登録
void FighterAircraft::Start() {
    SetActive(true);
    if (_compoundCollider) _compoundCollider->UpdateShape(); // コライダー形状を更新
    RegisterToManagers_();
}

// 物理本体・コライダーの登録解除
void FighterAircraft::Update(float dt) {
    if (!IsActive() || dt <= 0.0f) return;

	auto& key = KeyInput::Instance();   // キーボード入力のシングルトンインスタンスを取得

    // スロットル制御
	if (key.IsKeyInputHeld(KEY_INPUT_SPACE))  _throttle += _throttleSpeed * dt; // スロットルアップ
	if (key.IsKeyInputHeld(KEY_INPUT_LSHIFT)) _throttle -= _throttleSpeed * dt; // スロットルダウン
	_throttle = (std::max)(0.0f, (std::min)(1.0f, _throttle));                  // スロットルを0〜1にクランプ

	const float targetSpeed = Lerp(_minSpeed, _maxSpeed, _throttle);                // 目標速度をスロットルに応じて計算
	_currentSpeed = Lerp(_currentSpeed, targetSpeed, (std::min)(dt * 4.0f, 1.0f));  // 現在速度を目標速度に向かって補間（滑らかに変化）

    // 姿勢制御
    Quaternion rot = transform.LocalRotation();

    // ピッチ: 自機の Right 軸まわり
    if (key.IsKeyInputHeld(KEY_INPUT_W)) {
        rot = Quaternion::FromAxisAngleRad(rot.RotateVector(VGet(1, 0, 0)),  _pitchSpeed * dt) * rot;
    }
    if (key.IsKeyInputHeld(KEY_INPUT_S)) {
        rot = Quaternion::FromAxisAngleRad(rot.RotateVector(VGet(1, 0, 0)), -_pitchSpeed * dt) * rot;
    }

    // ヨー: 自機の Up 軸まわり
    if (key.IsKeyInputHeld(KEY_INPUT_E)) {
        rot = Quaternion::FromAxisAngleRad(rot.RotateVector(VGet(0, 1, 0)),  _yawSpeed * dt) * rot;
    }
    if (key.IsKeyInputHeld(KEY_INPUT_Q)) {
        rot = Quaternion::FromAxisAngleRad(rot.RotateVector(VGet(0, 1, 0)), -_yawSpeed * dt) * rot;
    }

    // ロール: 自機の Forward 軸まわり
    if (key.IsKeyInputHeld(KEY_INPUT_D)) {
        rot = Quaternion::FromAxisAngleRad(rot.RotateVector(VGet(0, 0, 1)),  _rollSpeed * dt) * rot;
    }
    if (key.IsKeyInputHeld(KEY_INPUT_A)) {
        rot = Quaternion::FromAxisAngleRad(rot.RotateVector(VGet(0, 0, 1)), -_rollSpeed * dt) * rot;
    }

	// 回転を正規化して Transform に反映
    transform.SetLocalRotation(rot.Normalized());

    // 常に Forward 方向へ前進
	_physicsBody._velocity = VScale(transform.Forward(), -_currentSpeed);    // Forward 方向に現在速度で移動
	_physicsBody.WakeUp();                                                  // 物理本体をスリープ解除（移動中なので）

	// コライダー形状を更新
	if (_compoundCollider) _compoundCollider->UpdateShape();

	// 機銃：砲口 = 機体前方（ノーズ先端）の位置
	// _afterCollider の top 方向が後方なので、Forward の逆 = 機首方向が -Forward
	const VECTOR muzzlePos = VAdd(
		transform.WorldPosition(),
		VScale(transform.Forward(), -(_height * 0.5f + _radius * 1.2f)));
	const VECTOR muzzleDir = VScale(transform.Forward(), -1.0f); // 機首方向
	const bool trigger = GetMouseInput() & MOUSE_INPUT_LEFT;       // 左クリックで発射
	_gun.Update(dt, muzzlePos, muzzleDir, trigger);

	//  W / S      : ピッチ（機首 上/下）
	//  A / D      : ヨー（機首 左/右）
	//  Q / E      : ロール（左/右バンク）
	//  Space      : スロットルアップ
	//  LShift     : スロットルダウン
	//  左クリック : 機銃発射
}

// 描画
void FighterAircraft::Draw() {
	if (_bodyCollider) {
		_bodyCollider->SetDebugColor(GetColor(120, 220, 255));  // 胴体ボックスを水色で描画
		_bodyCollider->DrawDebug();
	}
	if (_afterCollider) {
		_afterCollider->SetDebugColor(GetColor(255, 200, 80));   // 後方カプセルをオレンジで描画
		_afterCollider->DrawDebug();
	}
	if (_wingLeftCollider) {
		_wingLeftCollider->SetDebugColor(GetColor(255, 100, 100)); // 左主翼を赤で描画
		_wingLeftCollider->DrawDebug();
	}
	if (_wingRightCollider) {
		_wingRightCollider->SetDebugColor(GetColor(100, 100, 255)); // 右主翼を青で描画
		_wingRightCollider->DrawDebug();
	}
	if (_tailVerticalCollider) {
		_tailVerticalCollider->SetDebugColor(GetColor(255, 255, 0)); // 垂直尾翼を黄色で描画
		_tailVerticalCollider->DrawDebug();
	}
	_gun.Draw(); // 弾の描画
}

// 破棄
void FighterAircraft::End() {
	UnregisterFromManagers_();  // 物理本体・コライダーの登録解除
	SetActive(false);           // 非アクティブ化
}

// プールから取得された直後の初期化
void FighterAircraft::OnAcquire(const VariantMap& params) {
    SetActive(true);
    ConfigureFromParams_(params);
    
	// 初期位置をパラメータから取得（指定がなければ原点）
    const VECTOR pos = ParseVector3(params, "position", VGet(0.0f, 0.0f, 0.0f));
    transform.SetLocalPosition(pos);
    transform.SetLocalRotation(Quaternion::Identity());
    transform.SetLocalScale(VGet(1.0f, 1.0f, 1.0f));
    _throttle     = 0.3f;
    _currentSpeed = Lerp(_minSpeed, _maxSpeed, _throttle);
    _physicsBody._velocity        = VGet(0.0f, 0.0f, 0.0f);
    _physicsBody._angularVelocity = VGet(0.0f, 0.0f, 0.0f);
    _physicsBody.WakeUp();

	// コライダー形状を更新してマネージャーに登録
	if (_compoundCollider) _compoundCollider->UpdateShape();
	RegisterToManagers_();
}

// プールに返却される直前の後片付け
void FighterAircraft::OnRelease() {
    UnregisterFromManagers_();
    SetActive(false);
    _physicsBody._velocity        = VGet(0.0f, 0.0f, 0.0f);
    _physicsBody._angularVelocity = VGet(0.0f, 0.0f, 0.0f);
}

// 外部パラメータから形状・物理値を構築
void FighterAircraft::ConfigureFromParams_(const VariantMap& params) {
	// パラメータを取得し、最小値を保証
    _radius        = (std::max)(ParseFloat(params, "radius",        _radius),        0.01f);
    _height        = (std::max)(ParseFloat(params, "height",        _height),        0.05f);
    _minSpeed      = (std::max)(ParseFloat(params, "minSpeed",      _minSpeed),      0.1f);
    _maxSpeed      = (std::max)(ParseFloat(params, "maxSpeed",      _maxSpeed),      _minSpeed + 0.1f);
    _pitchSpeed    = (std::max)(ParseFloat(params, "pitchSpeed",    _pitchSpeed),    0.1f);
    _yawSpeed      = (std::max)(ParseFloat(params, "yawSpeed",      _yawSpeed),      0.1f);
    _rollSpeed     = (std::max)(ParseFloat(params, "rollSpeed",     _rollSpeed),     0.1f);
    _throttleSpeed = (std::max)(ParseFloat(params, "throttleSpeed", _throttleSpeed), 0.01f);
    const float mass = (std::max)(ParseFloat(params, "mass", 3.0f), 0.1f);

	// コライダー形状を更新
	if (_bodyCollider) {
		_bodyCollider->_box.center      = VGet(0.0f, 0.0f, 0.0f);
		_bodyCollider->_box.halfExtents = VGet(_radius, _radius * 0.6f, _height * 0.5f); // 幅・高さ・前後長
		_bodyCollider->UpdateShape();
	}
	if (_afterCollider) {
		_afterCollider->_cap.radius = _radius;                           // ノーズの半径
		_afterCollider->_cap.bottom = VGet(0.0f, 0.0f, _height * 0.5f); // 胴体前端
		_afterCollider->_cap.top    = VGet(0.0f, 0.0f, _height * 0.5f + _radius * 1.5f); // 先端
		_afterCollider->UpdateShape();
	}
	if (_compoundCollider) {
		_compoundCollider->owner = this;
		_compoundCollider->layer = layerMask::PLAYER;
		_compoundCollider->mask  = mask::ALL;
		_compoundCollider->UpdateShape();
	}

	// 物理本体のパラメータを更新
	_physicsBody._owner          = this;
	_physicsBody._useGravity     = false;   // 重力不使用（飛行機は揚力で浮く）
	_physicsBody._isKinematic    = false;   // Dynamic にして velocity で移動させる
	_physicsBody._freezeRotation = true;
	_physicsBody._linearDamping  = 0.0f;   // 減衰なし
	_physicsBody._maxLinearSpeed = _maxSpeed;
	_physicsBody._material       = PhysicsMaterial::Default();
	_physicsBody._material.friction    = 0.0f;
	_physicsBody._material.restitution = 0.0f;
	_physicsBody.SetMass(mass);
	if (_compoundCollider) _physicsBody.ComputeInertia(_compoundCollider.get());
}

// 物理本体・コライダーの登録
void FighterAircraft::RegisterToManagers_() {
    if (_registered) return;
    if (_compoundCollider) ColliderManager::Instance().RegisterCollider(_compoundCollider.get()); // CompoundCollider を登録
    PhysicsManager::Instance().RegisterBody(&_physicsBody);
    _registered = true;
}

// 物理本体・コライダーの登録解除
void FighterAircraft::UnregisterFromManagers_() {
    if (!_registered) return;
    if (_compoundCollider) ColliderManager::Instance().UnregisterCollider(_compoundCollider.get()); // CompoundCollider を解除
    PhysicsManager::Instance().UnregisterBody(&_physicsBody);
    _registered = false;
}
