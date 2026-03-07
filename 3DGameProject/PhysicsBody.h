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
	bool _enabled = true; 		//物理挙動の有効/スリープ
	bool _useGravity = true;	// 重力の影響を受けるか
	bool _isKinematic = false;	// true: 外部でTransform更新するだけ
	bool _freezeRotation = false;
	bool _useInterpolation = false;
	bool _detectContinuous = true;
	bool _isSleeping = false;

	float _mass = 1.0f;			// 質量（最低限）
	float _inverseMass = 1.0f;
	float _linearDamping = 0.0f;	// 線形減衰（最低限）
	float _angularDamping = 0.05f;
	float _restitution = 0.0f;	//反発（最低限）
	float _friction = 0.5f;		// 摩擦（最低限）
	float _gravityScale = 1.0f;
	float _sleepLinearThreshold = 0.05f;
	float _sleepAngularThreshold = 0.05f;
	float _sleepTimeThreshold = 0.5f;
	float _maxLinearSpeed = 100.0f;
	float _maxAngularSpeed = 20.0f;

	VECTOR _velocity = VGet(0, 0, 0);			//速度
	VECTOR _angularVelocity = VGet(0, 0, 0);	//角速度
	VECTOR _force = VGet(0, 0, 0);
	VECTOR _torque = VGet(0, 0, 0);
	VECTOR _movePositionTarget = VGet(0, 0, 0);
	Quaternion _moveRotationTarget = Quaternion::Identity();
	VECTOR _previousPosition = VGet(0, 0, 0);
	Quaternion _previousRotation = Quaternion::Identity();
	float _sleepTimer = 0.0f;
	bool _hasMovePositionTarget = false;
	bool _hasMoveRotationTarget = false;

public:
	bool IsEnabled() const noexcept { return _enabled; }
	void SetEnabled(bool enabled) noexcept {
		_enabled = enabled;
		if (enabled) {
			WakeUp();
		}
	}

	bool IsSleeping() const noexcept { return _isSleeping; }
	bool IsDynamic() const noexcept { return _enabled && !_isKinematic && _inverseMass > 0.0f; }
	float InverseMass() const noexcept { return (_isKinematic || _mass <= 1e-6f) ? 0.0f : _inverseMass; }

	void SetMass(float mass) noexcept {
		_mass = (mass > 1e-6f) ? mass : 0.0f;
		_inverseMass = (_mass > 1e-6f) ? (1.0f / _mass) : 0.0f;
		if (_mass <= 1e-6f) {
			_isKinematic = true;
		}
	}

	void WakeUp() noexcept {
		_isSleeping = false;
		_sleepTimer = 0.0f;
	}

	void Sleep() noexcept {
		_isSleeping = true;
		_sleepTimer = 0.0f;
		_velocity = VGet(0, 0, 0);
		_angularVelocity = VGet(0, 0, 0);
		_force = VGet(0, 0, 0);
		_torque = VGet(0, 0, 0);
	}

	void ClearAccumulators() noexcept {
		_force = VGet(0, 0, 0);
		_torque = VGet(0, 0, 0);
	}

	void AddForce(const VECTOR& force) noexcept {
		_force = VAdd(_force, force);
		if (force.x != 0.0f || force.y != 0.0f || force.z != 0.0f) {
			WakeUp();
		}
	}

	void AddAcceleration(const VECTOR& acceleration) noexcept {
		if (InverseMass() <= 0.0f) return;
		AddForce(VScale(acceleration, _mass));
	}

	void AddImpulse(const VECTOR& impulse) noexcept {
		const float inverseMass = InverseMass();
		if (inverseMass <= 0.0f) return;
		_velocity = VAdd(_velocity, VScale(impulse, inverseMass));
		WakeUp();
	}

	void AddVelocityChange(const VECTOR& deltaVelocity) noexcept {
		_velocity = VAdd(_velocity, deltaVelocity);
		WakeUp();
	}

	void AddTorque(const VECTOR& torque) noexcept {
		_torque = VAdd(_torque, torque);
		if (torque.x != 0.0f || torque.y != 0.0f || torque.z != 0.0f) {
			WakeUp();
		}
	}

	void AddAngularImpulse(const VECTOR& angularImpulse) noexcept {
		if (_freezeRotation) return;
		_angularVelocity = VAdd(_angularVelocity, angularImpulse);
		WakeUp();
	}

	void MovePosition(const VECTOR& targetPosition) noexcept {
		_movePositionTarget = targetPosition;
		_hasMovePositionTarget = true;
		WakeUp();
	}

	void MoveRotation(const Quaternion& targetRotation) noexcept {
		_moveRotationTarget = targetRotation.Normalized();
		_hasMoveRotationTarget = true;
		WakeUp();
	}

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
