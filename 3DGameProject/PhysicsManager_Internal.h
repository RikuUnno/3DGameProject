#pragma once
#pragma once
// 全 PhysicsManager_*.cpp が共有する内部ヘルパー。
// public API には含まれないため、PhysicsManager.h とは別ファイルにしている。

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "PhysicsManager.h"
#include "Assert.h"
#include "PhysicsController.h"
#include "PhysicsBody.h"
#include "PhysicsMaterial.h"
#include "GameObject.h"
#include "ColliderManager.h"
#include "Collider.h"
#include "SphereCollider.h"
#include "BoxCollider.h"
#include "CapsuleCollider.h"
#include "HalfPlaneCollider.h"
#include "CompoundCollider.h"
#include "Transform.h"
#include "ThreadPool.h"
#include "PerformanceMonitor.h"
#include "PhysicsCcd.h"

#if defined(_MSC_VER)
#include <intrin.h>
#endif

// 全実装ファイルが共有する内部ヘルパー関数と物理定数。
// inline で定義するため複数 TU に含めても ODR 違反にならない。
namespace {

    // ---- 不正ポインタ検出（デバッグ用）-------------------------------

    inline bool IsLikelyBadRef_(const void* p) noexcept {
        const auto u = reinterpret_cast<std::uintptr_t>(p);
        if (u == 0 || u == static_cast<std::uintptr_t>(~0ULL)) return true;
#if defined(_WIN64)
        if (u >= 0x0000800000000000ULL) return true;
#endif
        return false;
    }

    // ---- ベクトル演算 ------------------------------------------------

    inline float Dot3(const VECTOR& a, const VECTOR& b) noexcept {
#ifndef NDEBUG
        if (IsLikelyBadRef_(&a) || IsLikelyBadRef_(&b)) {
            ASSERT_MSG(false, "Dot3: invalid VECTOR reference. &a=%p &b=%p", &a, &b);
            return 0.0f;
        }
#endif
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline float LenSq(const VECTOR& v) noexcept { return Dot3(v, v); }
    inline float Len3(const VECTOR& v) noexcept { return std::sqrt((std::max)(LenSq(v), 0.0f)); }

    inline VECTOR SafeNormalize(const VECTOR& v, const VECTOR& fallback = VGet(0,1,0)) noexcept {
        const float len = Len3(v);
        return (len > 1e-6f) ? VScale(v, 1.0f / len) : fallback;
    }

    inline void ClampMagnitude(VECTOR& v, float maxMag) noexcept {
        if (maxMag <= 0.0f) return;
        const float ls = LenSq(v);
        if (ls <= maxMag * maxMag) return;
        v = VScale(v, maxMag / std::sqrt((std::max)(ls, 1e-8f)));
    }

    inline bool IsFinite(float v) noexcept { return std::isfinite(v); }
    inline bool IsFiniteVec(const VECTOR& v) noexcept { return IsFinite(v.x) && IsFinite(v.y) && IsFinite(v.z); }
    inline void SanitizeVec(VECTOR& v) noexcept { if (!IsFiniteVec(v)) v = VGet(0,0,0); }

    // ---- 物理定数 ----------------------------------------------------

    constexpr float kBiasFactor          = 0.2f;
    constexpr float kSlop                = 0.005f;
    constexpr float kMaxPen              = 5.0f;
    constexpr float kMaxCorrection       = 0.4f;
    constexpr float kRestitutionThreshold = 0.05f;
    // Bullet 既定値の 0.85 を採用。1.0 にすると warm-start が「完全復元」になり、
    // 壁に水平速度で衝突した球が衝突フレームの normalLambda/frictionLambda を
    // 維持し続けて空中に張り付く（摩擦が重力を支え続ける安定平衡）が発生する。
    // 0.85 だと毎フレーム ~15% ずつ減衰し、押し付けが消えた接触は数フレームで
    // 摩擦コーンが縮んで自然に滑り落ちる。
    constexpr float kWarmStartFactor     = 0.85f;
    constexpr float kContactMatchDistSq  = 0.01f;
    constexpr float kSplitBiasFactor     = 0.3f;
    constexpr float kSpeculativeMargin   = 0.02f;
    constexpr float kFrictionStaticThreshold = 0.05f;
    constexpr float kMaxMassRatio        = 10000.0f;
    // 転がり抵抗係数。接触法線まわり以外の相対角速度に対して、
    // 「摩擦係数 × レバーアーム × 法線インパルス」で上限を掛けた角インパルスを
    // 逆向きに与え、転がりを徐々に減衰させる。これがないと球が Box の辺や壁に
    // 押し付けられたとき、接触摩擦のレバーアームがトルクを供給し続け、角速度を
    // 抑えるものが微小な角減衰しかないため回転が暴走して張り付き・打ち上げが起きる。
    constexpr float kRollingFrictionFactor = 0.25f;
    // 転がり抵抗の1イテレーションあたりの最大減衰割合。
    // 1.0 にすると相対角速度を一気に打ち消し、接触摩擦と相まって回転・移動が
    // 衝突の瞬間に消えてしまう。0.2 程度にして徐々に減衰させる。
    constexpr float kRollingFrictionMaxStop = 0.2f;
    // Rolling resistance factor for the tangential (rolling) component of the
    // relative angular velocity at a contact. Kept much smaller than
    // kRollingFrictionFactor so the gravity torque on slopes always wins and
    // spheres keep rolling downhill instead of decelerating to a stop.
    constexpr float kRollingResistanceFactor = 0.05f;
    // ---- コライダーの最小半径（CCD トンネリング判定用）--------------

    inline float GetColliderMinHalfExtent(const Collider* col) noexcept {
        if (!col) return 0.5f;
        switch (col->GetKind()) {
        case Collider::Kind::Sphere:
            return static_cast<const SphereCollider*>(col)->GetRadius();
        case Collider::Kind::Box: {
            const VECTOR he = static_cast<const BoxCollider*>(col)->GetHalfExtents();
            return (std::min)({he.x, he.y, he.z});
        }
        case Collider::Kind::Capsule:
            return static_cast<const CapsuleCollider*>(col)->GetRadius();
        default:
            return 0.5f;
        }
    }

    // ---- 接触演算ヘルパー --------------------------------------------

    // 法線 n に直交する接線基底 (t1, t2) を生成（Coulomb 摩擦コーン用）
    inline void ComputeTangentBasis(const VECTOR& n, VECTOR& t1, VECTOR& t2) noexcept {
        t1 = SafeNormalize((std::fabs(n.x) < 0.9f)
            ? VCross(n, VGet(1,0,0))
            : VCross(n, VGet(0,1,0)));
        t2 = VCross(n, t1);
    }

    // 拘束方向 dir に対する有効逆質量（並進 + 回転慣性）
    // K = 1/mA + 1/mB + (rA×dir)^T * IA^-1 * (rA×dir) + ...
    inline float ComputeEffectiveInvMass(
        float invA, float invB,
        PhysicsBody* bodyA, PhysicsBody* bodyB,
        const VECTOR& rA, const VECTOR& rB,
        const VECTOR& dir) noexcept
    {
        float result = invA + invB;
        if (bodyA && invA > 0.0f && !bodyA->_freezeRotation) {
            const VECTOR tmp = bodyA->ApplyInverseInertia(VCross(rA, dir));
            result += Dot3(VCross(tmp, rA), dir);
        }
        if (bodyB && invB > 0.0f && !bodyB->_freezeRotation) {
            const VECTOR tmp = bodyB->ApplyInverseInertia(VCross(rB, dir));
            result += Dot3(VCross(tmp, rB), dir);
        }
        return result;
    }

    // インパルスを2物体に適用（作用反作用）
    //
    // sleeping body に対しては「インパルスが起床閾値を超えるか」で扱いを切替:
    //   - 大きいインパルス (例: 球が箱に衝突): 即 WakeUp() して通常通り速度更新。
    //                                          → ワンテンポ遅れず、反射過剰も起きない。
    //   - 微小インパルス (PGS 反復の jitter): skip。
    //                                          → 静止スタックが誤起床しない。
    // 閾値は _sleepLinearThreshold を流用するため UpdateSleepState のヒステリシスと整合する。
    inline void ApplyImpulse(
        PhysicsBody* bodyA, PhysicsBody* bodyB,
        float invA, float invB,
        const VECTOR& rA, const VECTOR& rB,
        const VECTOR& impulse) noexcept
    {
        if (bodyA && invA > 0.0f) {
            const VECTOR dvA   = VScale(impulse, invA);              // ボディAに加わる速度変化
            const float  dvSqA = Dot3(dvA, dvA);                     // 速度変化の二乗
            const float  thA   = bodyA->_sleepLinearThreshold;       // 起床判定閾値
            const bool   waking = bodyA->_isSleeping && dvSqA > thA * thA;
            if (!bodyA->_isSleeping || waking) {
                if (waking) bodyA->WakeUp();
                bodyA->_velocity = VSub(bodyA->_velocity, dvA);
                if (!bodyA->_freezeRotation)
                    bodyA->_angularVelocity = VSub(bodyA->_angularVelocity,
                        bodyA->ApplyInverseInertia(VCross(rA, impulse)));
            }
        }
        if (bodyB && invB > 0.0f) {
            const VECTOR dvB   = VScale(impulse, invB);              // ボディBに加わる速度変化
            const float  dvSqB = Dot3(dvB, dvB);                     // 速度変化の二乗
            const float  thB   = bodyB->_sleepLinearThreshold;       // 起床判定閾値
            const bool   waking = bodyB->_isSleeping && dvSqB > thB * thB;
            if (!bodyB->_isSleeping || waking) {
                if (waking) bodyB->WakeUp();
                bodyB->_velocity = VAdd(bodyB->_velocity, dvB);
                if (!bodyB->_freezeRotation)
                    bodyB->_angularVelocity = VAdd(bodyB->_angularVelocity,
                        bodyB->ApplyInverseInertia(VCross(rB, impulse)));
            }
        }
    }

    // 法線方向の相対速度（正 = 離れる、負 = 近づく）
    inline float RelNormalVelocity(
        PhysicsBody* bodyA, PhysicsBody* bodyB,
        const VECTOR& rA, const VECTOR& rB,
        const VECTOR& n) noexcept
    {
        const VECTOR vA = VAdd(bodyA ? bodyA->_velocity : VGet(0,0,0),
                               bodyA ? VCross(bodyA->_angularVelocity, rA) : VGet(0,0,0));
        const VECTOR vB = VAdd(bodyB ? bodyB->_velocity : VGet(0,0,0),
                               bodyB ? VCross(bodyB->_angularVelocity, rB) : VGet(0,0,0));
        return Dot3(VSub(vB, vA), n);
    }

    inline float RelDirVelocity(
        PhysicsBody* bodyA, PhysicsBody* bodyB,
        const VECTOR& rA, const VECTOR& rB,
        const VECTOR& d) noexcept
    {
        return RelNormalVelocity(bodyA, bodyB, rA, rB, d);
    }

    // 接触の相対角速度（ωB - ωA）。転がり抵抗の対象。
    inline VECTOR RelAngularVelocity(PhysicsBody* bodyA, PhysicsBody* bodyB) noexcept {
        const VECTOR wA = bodyA ? bodyA->_angularVelocity : VGet(0, 0, 0);
        const VECTOR wB = bodyB ? bodyB->_angularVelocity : VGet(0, 0, 0);
        return VSub(wB, wA);
    }

    // 軸方向の「角」有効逆質量（転がり抵抗用）。
    // axis 方向の相対角速度を変化させるのに必要な角インパルスの逆係数。
    //   K = axis^T * IA^-1 * axis + axis^T * IB^-1 * axis
    inline float AngularEffectiveInvMass(
        PhysicsBody* bodyA, PhysicsBody* bodyB,
        const VECTOR& axis) noexcept
    {
        float result = 0.0f;
        if (bodyA && bodyA->InverseMass() > 0.0f && !bodyA->_freezeRotation)
            result += Dot3(bodyA->ApplyInverseInertia(axis), axis);
        if (bodyB && bodyB->InverseMass() > 0.0f && !bodyB->_freezeRotation)
            result += Dot3(bodyB->ApplyInverseInertia(axis), axis);
        return result;
    }

    // 角インパルスのみを2剛体へ適用（並進速度は変えない・転がり抵抗用）。
    //   ωA -= IA^-1 * L,  ωB += IB^-1 * L
    inline void ApplyAngularImpulse(
        PhysicsBody* bodyA, PhysicsBody* bodyB,
        const VECTOR& angularImpulse) noexcept
    {
        if (bodyA && bodyA->InverseMass() > 0.0f && !bodyA->_freezeRotation && !bodyA->_isSleeping)
            bodyA->_angularVelocity = VSub(bodyA->_angularVelocity,
                bodyA->ApplyInverseInertia(angularImpulse));
        if (bodyB && bodyB->InverseMass() > 0.0f && !bodyB->_freezeRotation && !bodyB->_isSleeping)
            bodyB->_angularVelocity = VAdd(bodyB->_angularVelocity,
                bodyB->ApplyInverseInertia(angularImpulse));
    }

    // 半平面からの符号付き距離（正 = 表側、負 = 裏側）
    inline float HalfPlaneDistance(const VECTOR& point, const VECTOR& n, float d) noexcept {
        return Dot3(point, n) - d;
    }

} // anonymous namespace
