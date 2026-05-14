#pragma once

#include "DxLib.h"
#include "Math/Quaternion.h"
#include "PhysicsMaterial.h"

class GameObject;
class Collider;

// PhysicsBody
// - GameObject に付与する物理コンポーネント（データ保持）
// - 慣性テンソル（対角近似）を保持し、回転の物理を正しく計算する

// CCD quality level (Havok-style hkpMotionQualityType)
enum class CcdQuality : int {
    Discrete = 0,   // No CCD (fastest, may tunnel)
    Debris   = 1,   // Low-priority CCD: only swept AABB, no TOI backstep
    Default  = 2,   // Standard CCD: swept AABB + velocity clamp + speculative
    Bullet   = 3,   // Full CCD: TOI backstep + sub-step re-integration
    Critical = 4,   // Highest quality: never tunnel (strict TOI + reduced step)
};

class PhysicsBody {
public:
    PhysicsBody() = default;
    virtual ~PhysicsBody() = default;

    GameObject* _owner = nullptr;

    bool _enabled = true;
    bool _useGravity = true;
    bool _isKinematic = false;
    bool _freezeRotation = false;
    bool _useInterpolation = false;
    bool _detectContinuous = false;
    bool _isSleeping = false;

    // CCD quality level (controls how aggressively tunneling is prevented)
    CcdQuality _ccdQuality = CcdQuality::Default;

    // Per-body allowed penetration depth (Havok-style).
    // CCD is auto-triggered when penetration exceeds this threshold.
    // Smaller values = stricter CCD, higher cost. 0 = use collider threshold only.
    float _allowedPenetrationDepth = 0.0f;

    float _mass = 1.0f;
    float _inverseMass = 1.0f;
    float _linearDamping = 0.0f;
    float _angularDamping = 0.05f;
    float _restitution = 0.0f;
    float _friction = 0.5f;
    float _gravityScale = 1.0f;
    float _sleepLinearThreshold = 0.02f;
    float _sleepAngularThreshold = 0.02f;
    float _sleepTimeThreshold = 0.3f;
    float _maxLinearSpeed = 100.0f;
    float _maxAngularSpeed = 20.0f;

    VECTOR _velocity = VGet(0, 0, 0);
    VECTOR _angularVelocity = VGet(0, 0, 0);
    VECTOR _force = VGet(0, 0, 0);
    VECTOR _torque = VGet(0, 0, 0);
    VECTOR _movePositionTarget = VGet(0, 0, 0);
    Quaternion _moveRotationTarget = Quaternion::Identity();
    VECTOR _previousPosition = VGet(0, 0, 0);
    Quaternion _previousRotation = Quaternion::Identity();

    // --- Interpolation state (for sub-step interpolated rendering) ---
    VECTOR _interpPosition = VGet(0, 0, 0);
    Quaternion _interpRotation = Quaternion::Identity();

    float _sleepTimer = 0.0f;
    bool _hasMovePositionTarget = false;
    bool _hasMoveRotationTarget = false;

    // --- Inertia Tensor (full 3x3) ---
    // Inverse inertia tensor in local principal axes (symmetric 3x3).
    // Stored as row-major 3x3. Computed via ComputeInertia().
    // Identity = unit sphere fallback. Zero matrix = kinematic.
    float _inverseInertiaLocal[9] = {
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f
    };

    // --- Physics Material ---
    PhysicsMaterial _material{};

public:
    // マテリアルを適用し、friction/restitution/damping を一括設定する。
    // density > 0 かつ collider != nullptr の場合、コライダー体積から質量を自動計算する。
    void ApplyMaterial(const PhysicsMaterial& mat, Collider* collider = nullptr) noexcept;

    // マテリアルの取得
    const PhysicsMaterial& GetMaterial() const noexcept { return _material; }

public:
    bool IsEnabled() const noexcept { return _enabled; }
    void SetEnabled(bool enabled) noexcept {
        _enabled = enabled;
        if (enabled) WakeUp();
    }

    bool IsSleeping() const noexcept { return _isSleeping; }
    bool IsDynamic() const noexcept { return _enabled && !_isKinematic && _inverseMass > 0.0f; }
    float InverseMass() const noexcept { return (_isKinematic || _mass <= 1e-6f) ? 0.0f : _inverseMass; }

    // Inverse inertia diagonal (returns zero if kinematic/frozen).
    // Backward-compatible: extracts diagonal from full 3x3.
    VECTOR InverseInertiaDiag() const noexcept {
        if (_isKinematic || _mass <= 1e-6f || _freezeRotation) return VGet(0, 0, 0);
        return VGet(_inverseInertiaLocal[0], _inverseInertiaLocal[4], _inverseInertiaLocal[8]);
    }

    // Full 3x3 inverse inertia in local frame (returns zero matrix if kinematic/frozen).
    const float* InverseInertiaLocal3x3() const noexcept {
        static const float zero[9] = {};
        if (_isKinematic || _mass <= 1e-6f || _freezeRotation) return zero;
        return _inverseInertiaLocal;
    }

    // Apply inverse inertia tensor to a world-space vector:
    // I_world^{-1} * v = R * diag(I_local^{-1}) * (R^T * v)
    // where R is the body's current rotation matrix.
    // This correctly accounts for the body's orientation.
    VECTOR ApplyInverseInertia(const VECTOR& v) const noexcept;

    // Average scalar inverse inertia (for approximate rolling friction calculations).
    float AverageInverseInertia() const noexcept {
        const VECTOR ii = InverseInertiaDiag();
        return (ii.x + ii.y + ii.z) / 3.0f;
    }

    void SetMass(float mass) noexcept {
        _mass = (mass > 1e-6f) ? mass : 0.0f;
        _inverseMass = (_mass > 1e-6f) ? (1.0f / _mass) : 0.0f;
        if (_mass <= 1e-6f) {
            _isKinematic = true;
            for (int i = 0; i < 9; ++i) _inverseInertiaLocal[i] = 0.0f;
        }
    }

    // Compute inertia tensor from collider shape. Call after registration or shape change.
    void ComputeInertia(Collider* collider) noexcept;

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
        if (force.x != 0.0f || force.y != 0.0f || force.z != 0.0f) WakeUp();
    }

    void AddAcceleration(const VECTOR& acceleration) noexcept {
        if (InverseMass() <= 0.0f) return;
        AddForce(VScale(acceleration, _mass));
    }

    void AddImpulse(const VECTOR& impulse) noexcept {
        const float invM = InverseMass();
        if (invM <= 0.0f) return;
        _velocity = VAdd(_velocity, VScale(impulse, invM));
        WakeUp();
    }

    void AddVelocityChange(const VECTOR& deltaVelocity) noexcept {
        _velocity = VAdd(_velocity, deltaVelocity);
        WakeUp();
    }

    void AddTorque(const VECTOR& torque) noexcept {
        _torque = VAdd(_torque, torque);
        if (torque.x != 0.0f || torque.y != 0.0f || torque.z != 0.0f) WakeUp();
    }

    // Add angular impulse. Uses inverse inertia to convert to angular velocity change.
    void AddAngularImpulse(const VECTOR& angularImpulse) noexcept {
        if (_freezeRotation) return;
        _angularVelocity = VAdd(_angularVelocity, ApplyInverseInertia(angularImpulse));
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
        _detectContinuous = false;
        _isSleeping = false;
        _ccdQuality = CcdQuality::Default;
        _allowedPenetrationDepth = 0.0f;
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
        _interpPosition = VGet(0, 0, 0);
        _interpRotation = Quaternion::Identity();
        _sleepTimer = 0.0f;
        _hasMovePositionTarget = false;
        _hasMoveRotationTarget = false;
        for (int i = 0; i < 9; ++i) _inverseInertiaLocal[i] = (i % 4 == 0) ? 1.0f : 0.0f;
        _material = PhysicsMaterial{};
    }
};
