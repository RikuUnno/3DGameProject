#pragma once

#include "DxLib.h"
#include "Math/Quaternion.h"

class GameObject;

// PhysicsBody
// - Collider のように GameObjectへ付与するコンポーネント（データ保持）
class PhysicsBody {
public:
	PhysicsBody() = default;
	virtual ~PhysicsBody() = default;

	// 所有者（Transform を動かすため）
	GameObject* _owner = nullptr;

	//物理挙動の有効/スリープ（プール待機中・非アクティブ中は false にする想定）
	bool _enabled = true;				// 物理挙動全体の有効/無効
	bool _useGravity = true;			// 重力の影響を受けるか
	bool _isKinematic = false;			// 外部からの力や衝突の影響を受けないか（MovePosition/MoveRotation で動かす想定）
	bool _freezeRotation = false;		// 回転の影響を受けないか（衝突で回転させたくない場合など）
	bool _useInterpolation = false;		// 補間を使うか（高速移動物体の衝突安定化や、MovePosition/MoveRotation で滑らかに動かすのに使う想定）
	bool _detectContinuous = true;		// 連続衝突検知を使うか（高速移動物体の衝突安定化に使う想定。重いので必要な場合だけ有効にする）
	bool _isSleeping = false;			// 物理挙動のスリープ状態（衝突や力の加わりがないときに計算を減らすための状態。WakeUp/Sleep で切り替える）

	float _mass = 1.0f;						// 質量（0や極小値は kinematic 扱い）
	float _inverseMass = 1.0f;				// 質量の逆数（計算用。mass から自動計算される。0は kinematic 扱い）
	float _linearDamping = 0.0f;			// 線形減衰（速度に比例して減速する。0は減衰なし）
	float _angularDamping = 0.05f;			// 角度減衰（角速度に比例して減速する。0は減衰なし）
	float _restitution = 0.0f;				// 反発係数（衝突の弾性。0は非弾性、1は完全弾性）
	float _friction = 0.5f;					// 摩擦係数（衝突の摩擦。0は摩擦なし、1は高摩擦）
	float _gravityScale = 1.0f;				// 重力の影響の大きさ（0は重力なし、1は通常の重力、2は倍の重力など）
	float _sleepLinearThreshold = 0.05f;	// スリープ判定の線形速度閾値（この値以下の速度が続くとスリープする）
	float _sleepAngularThreshold = 0.05f;	// スリープ判定の角速度閾値（この値以下の角速度が続くとスリープする）
	float _sleepTimeThreshold = 0.5f;		// スリープ判定の時間閾値（この時間以上スリープ条件が続くとスリープする）
	float _maxLinearSpeed = 100.0f;			// 速度の最大値（これ以上速くならないようにクランプする。高速移動物体の衝突安定化に使う想定）
	float _maxAngularSpeed = 20.0f;			// 角速度の最大値（これ以上速くならないようにクランプする。高速移動物体の衝突安定化に使う想定）

	VECTOR _velocity = VGet(0, 0, 0);							// 速度
	VECTOR _angularVelocity = VGet(0, 0, 0);					// 角速度
	VECTOR _force = VGet(0, 0, 0);								// 力の蓄積（毎フレームリセットされる。AddForce などで加算していく）
	VECTOR _torque = VGet(0, 0, 0);								// トルクの蓄積（毎フレームリセットされる。AddTorque などで加算していく）
	VECTOR _movePositionTarget = VGet(0, 0, 0);					// MovePosition で移動する目標位置
	Quaternion _moveRotationTarget = Quaternion::Identity();	// MoveRotation で回転する目標回転
	VECTOR _previousPosition = VGet(0, 0, 0);					// 前フレームの位置
	Quaternion _previousRotation = Quaternion::Identity();		// 前フレームの回転
	float _sleepTimer = 0.0f;									// スリープ判定のためのタイマー（スリープ条件が続いている時間を測る）
	bool _hasMovePositionTarget = false;						// MovePosition で移動する目標位置が設定されているか
	bool _hasMoveRotationTarget = false;						// MoveRotation で回転する目標回転が設定されているか

public:
	bool IsEnabled() const noexcept { return _enabled; }
	void SetEnabled(bool enabled) noexcept {
		_enabled = enabled;
		if (enabled) {
			WakeUp();
		}
	}

	bool IsSleeping() const noexcept { return _isSleeping; }												// スリープ状態か
	bool IsDynamic() const noexcept { return _enabled && !_isKinematic && _inverseMass > 0.0f; }			// 動的（物理挙動の影響を受ける）か
	float InverseMass() const noexcept { return (_isKinematic || _mass <= 1e-6f) ? 0.0f : _inverseMass; }	// 質量の逆数を返す（kinematic もしくは質量が極小値以下の場合は 0 を返す）

	// mass を設定するときは、inverseMass も自動で計算してセットする。mass が 0 や極小値以下の場合は kinematic 扱いにする。
	void SetMass(float mass) noexcept {
		_mass = (mass > 1e-6f) ? mass : 0.0f;
		_inverseMass = (_mass > 1e-6f) ? (1.0f / _mass) : 0.0f;
		if (_mass <= 1e-6f) {
			_isKinematic = true;
		}
	}

	// 物理挙動のスリープ状態を切り替える。スリープ中は衝突や力の加わりがないときに計算を減らすための状態。WakeUp/Sleep で切り替える。
	void WakeUp() noexcept {
		_isSleeping = false;
		_sleepTimer = 0.0f;
	}

	// 物理挙動のスリープ状態を切り替える。スリープ中は衝突や力の加わりがないときに計算を減らすための状態。WakeUp/Sleep で切り替える。
	void Sleep() noexcept {
		_isSleeping = true;
		_sleepTimer = 0.0f;
		_velocity = VGet(0, 0, 0);
		_angularVelocity = VGet(0, 0, 0);
		_force = VGet(0, 0, 0);
		_torque = VGet(0, 0, 0);
	}

	// 毎フレームの物理更新の最後に呼ばれる。力やトルクの蓄積をリセットする。
	void ClearAccumulators() noexcept {
		_force = VGet(0, 0, 0);
		_torque = VGet(0, 0, 0);
	}

	// 外部から力やトルクを加えるためのメソッド。加えたときは WakeUp する。
	void AddForce(const VECTOR& force) noexcept {
		_force = VAdd(_force, force);
		if (force.x != 0.0f || force.y != 0.0f || force.z != 0.0f) {
			WakeUp();
		}
	}

	// 外部から加速度を加えるためのメソッド。内部で質量を考慮して力に変換して加える。加えたときは WakeUp する。
	void AddAcceleration(const VECTOR& acceleration) noexcept {
		if (InverseMass() <= 0.0f) return;
		AddForce(VScale(acceleration, _mass));
	}

	// 外部から衝撃を加えるためのメソッド。内部で質量を考慮して速度変化に変換して加える。加えたときは WakeUp する。
	void AddImpulse(const VECTOR& impulse) noexcept {
		const float inverseMass = InverseMass();
		if (inverseMass <= 0.0f) return;
		_velocity = VAdd(_velocity, VScale(impulse, inverseMass));
		WakeUp();
	}

	// 外部から速度変化を加えるためのメソッド。直接速度に加算する。加えたときは WakeUp する。
	void AddVelocityChange(const VECTOR& deltaVelocity) noexcept {
		_velocity = VAdd(_velocity, deltaVelocity);
		WakeUp();
	}

	// 外部からトルクを加えるためのメソッド。加えたときは WakeUp する。
	void AddTorque(const VECTOR& torque) noexcept {
		_torque = VAdd(_torque, torque);
		if (torque.x != 0.0f || torque.y != 0.0f || torque.z != 0.0f) {
			WakeUp();
		}
	}

	// 外部から角加速度を加えるためのメソッド。内部で慣性を考慮してトルクに変換して加える。加えたときは WakeUp する。
	void AddAngularImpulse(const VECTOR& angularImpulse) noexcept {
		if (_freezeRotation) return;
		_angularVelocity = VAdd(_angularVelocity, angularImpulse);
		WakeUp();
	}

	// 外部から角速度変化を加えるためのメソッド。直接角速度に加算する。加えたときは WakeUp する。
	void MovePosition(const VECTOR& targetPosition) noexcept {
		_movePositionTarget = targetPosition;
		_hasMovePositionTarget = true;
		WakeUp();
	}

	// 外部から角速度変化を加えるためのメソッド。直接角速度に加算する。加えたときは WakeUp する。
	void MoveRotation(const Quaternion& targetRotation) noexcept {
		_moveRotationTarget = targetRotation.Normalized();
		_hasMoveRotationTarget = true;
		WakeUp();
	}

	// デフォルト値にリセットする。プールから出すときなどに呼ぶ想定。
	void Reset() noexcept {
		_enabled = true;
		_useGravity = true;
		_isKinematic = false;
		_freezeRotation = false;
		_useInterpolation = false;
		_detectContinuous = true;
		_isSleeping = false;
		_mass = 1.0f;
		_inverseMass = 1.0f;
		_linearDamping = 0.0f;
		_angularDamping = 0.05f;
		_restitution = 0.0f;
		_friction = 0.5f;
		_gravityScale = 1.0f;
		_sleepLinearThreshold = 0.05f;
		_sleepAngularThreshold = 0.05f;
		_sleepTimeThreshold = 0.5f;
		_maxLinearSpeed = 100.0f;
		_maxAngularSpeed = 20.0f;
		_velocity = VGet(0, 0, 0);
		_angularVelocity = VGet(0, 0, 0);
		_force = VGet(0, 0, 0);
		_torque = VGet(0, 0, 0);
		_movePositionTarget = VGet(0, 0, 0);
		_moveRotationTarget = Quaternion::Identity();
		_previousPosition = VGet(0, 0, 0);
		_previousRotation = Quaternion::Identity();
		_sleepTimer = 0.0f;
		_hasMovePositionTarget = false;
		_hasMoveRotationTarget = false;
	}
};
