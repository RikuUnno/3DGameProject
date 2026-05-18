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
    constexpr float kWarmStartFactor     = 1.0f;
    constexpr float kContactMatchDistSq  = 0.01f;
    constexpr float kSplitBiasFactor     = 0.3f;
    constexpr float kSpeculativeMargin   = 0.02f;
    constexpr float kFrictionStaticThreshold = 0.05f;
    constexpr float kMaxMassRatio        = 10000.0f;

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
    inline void ApplyImpulse(
        PhysicsBody* bodyA, PhysicsBody* bodyB,
        float invA, float invB,
        const VECTOR& rA, const VECTOR& rB,
        const VECTOR& impulse) noexcept
    {
        const float impSq = Dot3(impulse, impulse);
        const bool hasImpulse = impSq > 1e-12f;

        if (bodyA && invA > 0.0f) {
            bodyA->_velocity = VSub(bodyA->_velocity, VScale(impulse, invA));
            if (!bodyA->_freezeRotation)
                bodyA->_angularVelocity = VSub(bodyA->_angularVelocity,
                    bodyA->ApplyInverseInertia(VCross(rA, impulse)));
            if (hasImpulse) bodyA->WakeUp();
        }
        if (bodyB && invB > 0.0f) {
            bodyB->_velocity = VAdd(bodyB->_velocity, VScale(impulse, invB));
            if (!bodyB->_freezeRotation)
                bodyB->_angularVelocity = VAdd(bodyB->_angularVelocity,
                    bodyB->ApplyInverseInertia(VCross(rB, impulse)));
            if (hasImpulse) bodyB->WakeUp();
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

    // 半平面からの符号付き距離（正 = 表側、負 = 裏側）
    inline float HalfPlaneDistance(const VECTOR& point, const VECTOR& n, float d) noexcept {
        return Dot3(point, n) - d;
    }

} // anonymous namespace
