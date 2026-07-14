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
	// アクティブ状態でない場合や dt が 0 以下の場合は処理をスキップ
    if (!IsActive() || dt <= 0.0f) return;

	auto& key = KeyInput::Instance();               // キー入力のシングルトンを取得
	VECTOR wish = VGet(0.0f, 0.0f, 0.0f);           // 移動希望ベクトルを初期化
	const VECTOR forward = transform.Forward();     // Transform の前方向ベクトルを取得
	const VECTOR right = transform.Right();         // Transform の右方向ベクトルを取得

	// キー入力に応じて移動希望ベクトルを計算
	if (key.IsKeyInputHeld(KEY_INPUT_W)) {
		wish = VAdd(wish, forward);         // W キーで前方向に移動希望ベクトルを加算
    }
    if (key.IsKeyInputHeld(KEY_INPUT_S)) {
		wish = VSub(wish, forward);         // S キーで後方向に移動希望ベクトルを減算
    } 
    if (key.IsKeyInputHeld(KEY_INPUT_D)) {
        wish = VAdd(wish, right);           // D キーで右方向に移動希望ベクトルを加算
    }
    if (key.IsKeyInputHeld(KEY_INPUT_A)) {
        wish = VSub(wish, right);           // A キーで左方向に移動希望ベクトルを減算
    }

    if (key.IsKeyInputHeld(KEY_INPUT_SPACE)) {
		wish = VAdd(wish, VGet(0.0f, 1.0f, 0.0f));  // SPACE キーで上方向に移動希望ベクトルを加算
    }
    if (key.IsKeyInputHeld(KEY_INPUT_LSHIFT)) {
		wish = VSub(wish, VGet(0.0f, 1.0f, 0.0f));  // LSHIFT キーで下方向に移動希望ベクトルを減算
    }

	// 移動希望ベクトルを正規化（長さが 0 に近い場合は forward を使用）
    const VECTOR flatWish = FlattenY(wish);
    if (flatWish.x != 0.0f || flatWish.y != 0.0f || flatWish.z != 0.0f) {
		wish = SafeNormalize(wish, forward);        // wish が 0 ベクトルの場合は forward を使用
    }

	// PhysicsBody の速度を移動希望ベクトルに基づいて設定
	VECTOR vel = _physicsBody._velocity;                // 現在の速度を取得
	const VECTOR targetVel = VScale(wish, _moveSpeed);  // 移動希望ベクトルに移動速度を掛けて目標速度を計算
    vel.x = targetVel.x;
    vel.z = targetVel.z;
    vel.y = targetVel.y;
	_physicsBody._velocity = vel;                       // PhysicsBody の速度を更新
	_physicsBody.WakeUp();                              // PhysicsBody をスリープ解除して物理挙動を有効化

	// 移動希望ベクトルの方向に向くように回転を設定
    if (flatWish.x != 0.0f || flatWish.z != 0.0f) {
        const float yaw = std::atan2(flatWish.x, flatWish.z);
        transform.SetLocalRotation(Quaternion::FromAxisAngleRad(VGet(0.0f, 1.0f, 0.0f), yaw));
    }

	// コライダーの形状を更新
    if (_capsuleCollider) {
        _capsuleCollider->UpdateShape();
    }
}

// 描画処理
void FighterAircraft::Draw() {
	// デバッグ描画
    if (_capsuleCollider) {
        _capsuleCollider->SetDebugColor(GetColor(120, 220, 255));
        _capsuleCollider->DrawDebug();
    }
}

// 終了処理
void FighterAircraft::End() {
	UnregisterFromManagers_();  // PhysicsBody / Collider を各マネージャーから登録解除
	SetActive(false);           // アクティブ状態を解除
}

// プールから取得された直後の初期化
void FighterAircraft::OnAcquire(const VariantMap& params) {
	SetActive(true);                // アクティブ状態を有効化
	ConfigureFromParams_(params);   // 外部パラメータから形状・物理値を構築

	// 初期位置を設定
	const VECTOR position = ParseVector3(params, "position", VGet(0.0f, 0.0f, 0.0f));   // "position" パラメータが存在しない場合はデフォルト位置を使用
	transform.SetLocalPosition(position);                                               // Transform のローカル位置を設定
	transform.SetLocalRotation(Quaternion::Identity());                                 // Transform のローカル回転を初期化（単位クォータニオン）
	transform.SetLocalScale(VGet(1.0f, 1.0f, 1.0f));                                    // Transform のローカルスケールを初期化（1.0, 1.0, 1.0）
	_physicsBody._velocity = VGet(0.0f, 0.0f, 0.0f);                                    // PhysicsBody の線形速度を初期化
	_physicsBody._angularVelocity = VGet(0.0f, 0.0f, 0.0f);                             // PhysicsBody の角速度を初期化
	_physicsBody.WakeUp();                                                              // PhysicsBody をスリープ解除して物理挙動を有効化

	// PhysicsBody / Collider を各マネージャーに登録
    if (_capsuleCollider) {
		_capsuleCollider->UpdateShape();    // コライダーの形状を更新
    }
	RegisterToManagers_();
}

// プールに返却される直前の後片付け
void FighterAircraft::OnRelease() {
    UnregisterFromManagers_();  // PhysicsBody / Collider を各マネージャーから登録解除
    SetActive(false);           // アクティブ状態を解除
    _physicsBody._velocity = VGet(0.0f, 0.0f, 0.0f);            // PhysicsBody の線形速度を初期化
    _physicsBody._angularVelocity = VGet(0.0f, 0.0f, 0.0f);     // PhysicsBody の角速度を初期化
}

// 外部パラメータから形状・物理値を構築
void FighterAircraft::ConfigureFromParams_(const VariantMap& params) {
	_radius = (std::max)(ParseFloat(params, "radius", _radius), 0.01f);                                 // コライダー半径を設定（最小値 0.01f）
	_height = (std::max)(ParseFloat(params, "height", _height), 0.05f);                                 // コライダー高さを設定（最小値 0.05f）
	_moveSpeed = (std::max)(ParseFloat(params, "moveSpeed", _moveSpeed), 0.1f);                         // 移動速度を設定（最小値 0.1f）
	_turnSpeed = (std::max)(ParseFloat(params, "turnSpeed", _turnSpeed), 0.1f);                         // 回転速度を設定（最小値 0.1f）
	const float mass = (std::max)(ParseFloat(params, "mass", 1.5f), 0.1f);                              // 質量を設定（最小値 0.1f）
	const float gravityScale = ParseFloat(params, "gravityScale", _physicsBody._gravityScale);          // 重力スケールを設定
	const float maxLinearSpeed = ParseFloat(params, "maxLinearSpeed", _physicsBody._maxLinearSpeed);    // 最大線形速度を設定

	// コライダーの形状を更新
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

	// PhysicsBody のパラメータを更新
	_physicsBody._owner = this;                             // PhysicsBody の所有者を設定
	_physicsBody._gravityScale = gravityScale;              // 重力スケールを設定
	_physicsBody._maxLinearSpeed = maxLinearSpeed;          // 最大線形速度を設定
	_physicsBody._useGravity = true;                        // 重力を有効化
	_physicsBody._freezeRotation = true;                    // 回転を固定
	_physicsBody._material = PhysicsMaterial::Default();    // 物理マテリアルをデフォルトに設定
	_physicsBody._material.friction = 0.15f;                // 物理マテリアルの摩擦係数を設定
	_physicsBody._material.staticFriction = 0.2f;           // 物理マテリアルの静止摩擦係数を設定
	_physicsBody._material.restitution = 0.0f;              // 物理マテリアルの反発係数を設定
	_physicsBody.SetMass(mass);                             // 質量を設定（0.1f 以上に制限）
    if (_capsuleCollider) {
        _physicsBody.ComputeInertia(_capsuleCollider.get()); // 慣性テンソルを計算
    }

	// ※この関数の使用例
	// ConfigureFromParams_({radius=0.5, height=1.5, moveSpeed=15.0, turnSpeed=4.0, mass=2.0, gravityScale=0.5, maxLinearSpeed=50.0});
	// (半径0.5、高さ1.5、移動速度15.0、回転速度4.0、質量2.0、重力スケール0.5、最大線形速度50.0)
}

// PhysicsBody / Collider を各マネージャーに登録
void FighterAircraft::RegisterToManagers_() {
    if (_registered) return;
    if (_capsuleCollider) {
		ColliderManager::Instance().RegisterCollider(_capsuleCollider.get());   // コライダーを ColliderManager に登録
    }
	PhysicsManager::Instance().RegisterBody(&_physicsBody); // PhysicsBody を PhysicsManager に登録
	_registered = true;		// 登録状態を更新
}

// PhysicsBody / Collider を各マネージャーから登録解除
void FighterAircraft::UnregisterFromManagers_() {
    if (!_registered) return;
    if (_capsuleCollider) {
		ColliderManager::Instance().UnregisterCollider(_capsuleCollider.get()); // コライダーを ColliderManager から登録解除
    }
	PhysicsManager::Instance().UnregisterBody(&_physicsBody);   // PhysicsBody を PhysicsManager から登録解除
	_registered = false;    // 登録状態を解除
}
