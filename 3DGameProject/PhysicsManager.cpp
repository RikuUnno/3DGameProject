#include "PhysicsManager.h"

#include <algorithm>
#include <cmath>

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

namespace {
    inline float Dot3(const VECTOR& a, const VECTOR& b) noexcept {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
    inline float LenSq(const VECTOR& v) noexcept { return Dot3(v, v); }
    inline float Len3(const VECTOR& v) noexcept { return std::sqrt((std::max)(LenSq(v), 0.0f)); }

    inline VECTOR SafeNormalize(const VECTOR& v, const VECTOR& fallback = VGet(0,1,0)) noexcept {
        const float len = Len3(v);
        if (len > 1e-6f) return VScale(v, 1.0f / len);
        return fallback;
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

    constexpr float kBiasFactor   = 0.2f;
    constexpr float kSlop         = 0.005f;
    constexpr float kMaxPen       = 5.0f;
    constexpr float kMaxCorrection = 0.5f;
    constexpr float kRestitutionThreshold = 0.05f;
    constexpr float kWarmStartFactor = 0.8f;
    constexpr float kContactMatchDistSq = 0.04f;
    constexpr float kSplitBiasFactor = 0.1f;
    constexpr float kSpeculativeMargin = 0.02f; // speculative contact margin

    inline float GetColliderMinHalfExtent(const Collider* col) noexcept {
        if (!col) return 0.5f;
        switch (col->GetKind()) {
        case Collider::Kind::Sphere: return static_cast<const SphereCollider*>(col)->GetRadius();
        case Collider::Kind::Box: {
            const VECTOR he = static_cast<const BoxCollider*>(col)->GetHalfExtents();
            return (std::min)({he.x, he.y, he.z});
        }
        case Collider::Kind::Capsule: return static_cast<const CapsuleCollider*>(col)->GetRadius();
        default: return 0.5f;
        }
    }

    inline void ComputeTangentBasis(const VECTOR& n, VECTOR& t1, VECTOR& t2) noexcept {
        if (std::fabs(n.x) < 0.9f)
            t1 = SafeNormalize(VCross(n, VGet(1,0,0)));
        else
            t1 = SafeNormalize(VCross(n, VGet(0,1,0)));
        t2 = VCross(n, t1);
    }

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

    inline void ApplyImpulse(
        PhysicsBody* bodyA, PhysicsBody* bodyB,
        float invA, float invB,
        const VECTOR& rA, const VECTOR& rB,
        const VECTOR& impulse) noexcept
    {
        if (bodyA && invA > 0.0f) {
            bodyA->_velocity = VSub(bodyA->_velocity, VScale(impulse, invA));
            if (!bodyA->_freezeRotation)
                bodyA->_angularVelocity = VSub(bodyA->_angularVelocity,
                    bodyA->ApplyInverseInertia(VCross(rA, impulse)));
        }
        if (bodyB && invB > 0.0f) {
            bodyB->_velocity = VAdd(bodyB->_velocity, VScale(impulse, invB));
            if (!bodyB->_freezeRotation)
                bodyB->_angularVelocity = VAdd(bodyB->_angularVelocity,
                    bodyB->ApplyInverseInertia(VCross(rB, impulse)));
        }
    }

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

    // Half-plane distance: signed distance from point to plane (positive = outside solid)
    inline float HalfPlaneDistance(const VECTOR& point, const VECTOR& planeNormal, float planeD) noexcept {
        return Dot3(point, planeNormal) - planeD;
    }
}

// ============================================================
//  Lifecycle
// ============================================================
void PhysicsManager::Shutdown() {
    const bool was = _shuttingDown.exchange(true, std::memory_order_relaxed);
    if (was) return;
    std::lock_guard lk(_mtx);
    _controllers.clear();
    _bodies.clear();
    _solverContacts.clear();
    _prevSolverContacts.clear();
    _islands.clear();
    _bodyIslandMap.clear();
    _accumulator = 0.0f;
}

void PhysicsManager::SetFixedDeltaTime(float fixedDeltaTime) noexcept {
    _fixedDeltaTime = (fixedDeltaTime > 1e-4f) ? fixedDeltaTime : (1.0f / 120.0f);
}
void PhysicsManager::SetMaxSubSteps(int maxSubSteps) noexcept {
    _maxSubSteps = (maxSubSteps > 1) ? maxSubSteps : 1;
}
void PhysicsManager::SetSolverIterations(int solverIterations) noexcept {
    _solverIterations = (solverIterations > 1) ? solverIterations : 1;
}

// Adaptive iteration count: more contacts → more iterations, clamped to [min, max]
int PhysicsManager::ComputeAdaptiveIterations() const noexcept {
    const int contactCount = static_cast<int>(_solverContacts.size());
    // Heuristic: base iterations + 1 per 10 contacts
    int adaptive = _minSolverIterations + contactCount / 10;
    adaptive = (std::max)(adaptive, _minSolverIterations);
    adaptive = (std::min)(adaptive, _maxSolverIterations);
    // Also respect explicit _solverIterations if it's within range
    return (std::max)(adaptive, (std::min)(_solverIterations, _maxSolverIterations));
}

void PhysicsManager::Update(float dt) {
    if (IsShuttingDown()) return;
    if (dt < 0.0f) dt = 0.0f;

    // コントローラーリストのスナップショットをロック下で取得。
    // Update 中に Register/Unregister が呼ばれてもレースしない。
    std::vector<PhysicsController*> ctrlSnapshot;
    {
        std::lock_guard lk(_mtx);
        ctrlSnapshot = _controllers;
    }
    for (auto* c : ctrlSnapshot) {
        if (!c) continue;
        c->Update(dt);
    }

    const float maxDt = _fixedDeltaTime * static_cast<float>(_maxSubSteps);
    _accumulator += (std::min)(dt, maxDt);

    int sub = 0;
    while (_accumulator + 1e-6f >= _fixedDeltaTime && sub < _maxSubSteps) {
        StepSimulation(_fixedDeltaTime);
        _accumulator -= _fixedDeltaTime;
        ++sub;
    }
    if (_accumulator < 0.0f) _accumulator = 0.0f;

    // Compute interpolated transforms for rendering
    ComputeInterpolation();
}

// ============================================================
//  Lookup caches
// ============================================================
void PhysicsManager::BuildLookupCaches() {
    _bodyByOwner.clear();
    _bodyByOwner.reserve(_bodies.size());
    for (auto* body : _bodies) {
        if (!body || !body->_owner) continue;
        _bodyByOwner[body->_owner] = body;
    }
    _colliderByOwner.clear();
    const auto& colliders = ColliderManager::Instance().GetColliders();
    _colliderByOwner.reserve(colliders.size());
    for (auto* col : colliders) {
        if (!col || !col->owner) continue;
        _colliderByOwner[col->owner] = col;
    }
}

PhysicsBody* PhysicsManager::CachedFindBody(GameObject* owner) const {
    if (!owner) return nullptr;
    auto it = _bodyByOwner.find(owner);
    return (it != _bodyByOwner.end()) ? it->second : nullptr;
}

Collider* PhysicsManager::CachedFindCollider(GameObject* owner) const {
    if (!owner) return nullptr;
    auto it = _colliderByOwner.find(owner);
    return (it != _colliderByOwner.end()) ? it->second : nullptr;
}

// ============================================================
//  Step
// ============================================================
void PhysicsManager::StepSimulation(float stepDt) {
    BuildLookupCaches();
    IntegrateBodies(stepDt);
    ColliderManager::Instance().Update(stepDt);
    BuildLookupCaches();

    // Resolve TOI events: detect CCD-enabled bodies that tunneled,
    // sort by earliest TOI, backstep and re-simulate remainder.
    ResolveToiEvents(stepDt);

    BuildSolverContacts(stepDt);
    if (_speculativeCcdEnabled) GenerateSpeculativeContacts(stepDt);
    BuildIslands();
    WarmStart();

    const int iterations = ComputeAdaptiveIterations();
    // Override _solverIterations for this step
    const int savedIter = _solverIterations;
    _solverIterations = iterations;
    SolveAllIslands(stepDt);
    _solverIterations = savedIter;

    if (_splitImpulseEnabled) {
        SplitImpulseCorrection(stepDt);
    } else {
        PositionalCorrection(stepDt);
    }
    _prevSolverContacts = _solverContacts;

    PropagateIslandSleep();

    for (auto* body : _bodies) {
        if (!body) continue;
        if (body->_isSleeping) {
            if (LenSq(body->_velocity) > 1e-8f || LenSq(body->_angularVelocity) > 1e-8f)
                body->WakeUp();
        }
        ApplyBodyConstraints(body);
        UpdateSleepState(body, stepDt);
    }
}

// ============================================================
//  IntegrateBodies ? arbitrary gravity vector, half-plane ground
// ============================================================
void PhysicsManager::IntegrateBodies(float stepDt) {
    const size_t bodyCount = _bodies.size();
    if (bodyCount == 0) return;

    const bool groundEnabled = _groundPlaneEnabled;
    const VECTOR groundN = _groundPlaneNormal;
    const float groundD = _groundPlaneD;
    const VECTOR gravity = _gravity;

    ThreadPool::Instance().ParallelFor(0, bodyCount, [&](size_t idx) {
        PhysicsBody* body = _bodies[idx];
        if (!body || !body->_enabled || !body->_owner || !body->_owner->IsActive()) return;

        body->_previousPosition = body->_owner->transform.LocalPosition();
        body->_previousRotation = body->_owner->transform.LocalRotation();

        if (body->_isKinematic) {
            if (body->_hasMovePositionTarget) {
                body->_owner->transform.SetLocalPosition(body->_movePositionTarget);
                body->_hasMovePositionTarget = false;
            }
            if (body->_hasMoveRotationTarget) {
                body->_owner->transform.SetLocalRotation(body->_moveRotationTarget);
                body->_hasMoveRotationTarget = false;
            }
            body->ClearAccumulators();
            body->_velocity = VGet(0,0,0);
            body->_angularVelocity = VGet(0,0,0);
            return;
        }

        if (body->_isSleeping && LenSq(body->_force) <= 1e-8f && LenSq(body->_torque) <= 1e-8f) return;
        if (LenSq(body->_force) > 1e-8f || LenSq(body->_torque) > 1e-8f) body->WakeUp();

        const float inverseMass = body->InverseMass();
        if (inverseMass <= 0.0f) { body->ClearAccumulators(); return; }

        // Arbitrary gravity vector
        VECTOR acceleration = VScale(body->_force, inverseMass);
        if (body->_useGravity) {
            acceleration = VAdd(acceleration, VScale(gravity, body->_gravityScale));
        }
        body->_velocity = VAdd(body->_velocity, VScale(acceleration, stepDt));

        if (!body->_freezeRotation) {
            const VECTOR angAcc = body->ApplyInverseInertia(body->_torque);
            body->_angularVelocity = VAdd(body->_angularVelocity, VScale(angAcc, stepDt));
        }

        if (body->_linearDamping > 0.0f)
            body->_velocity = VScale(body->_velocity, 1.0f / (1.0f + body->_linearDamping * stepDt));
        if (!body->_freezeRotation && body->_angularDamping > 0.0f)
            body->_angularVelocity = VScale(body->_angularVelocity, 1.0f / (1.0f + body->_angularDamping * stepDt));

        ApplyBodyConstraints(body);
        ClampMagnitude(body->_velocity, body->_maxLinearSpeed);
        if (!body->_freezeRotation) ClampMagnitude(body->_angularVelocity, body->_maxAngularSpeed);

        // TOI-based CCD with backstep + angular consideration:
        // Respects per-body CcdQuality: Discrete=skip, Debris=AABB only (no clamp),
        // Default/Bullet/Critical = velocity+angular clamp.
        if (body->_detectContinuous && body->_ccdQuality >= CcdQuality::Default) {
            Collider* col = CachedFindCollider(body->_owner);
            if (col) {
                const float minHE = GetColliderMinHalfExtent(col);
                // Use allowedPenetrationDepth if set, otherwise use collider thickness
                const float allowedPen = (body->_allowedPenetrationDepth > 0.0f)
                    ? body->_allowedPenetrationDepth : (minHE * 0.8f);
                const float linearSpeed = Len3(body->_velocity);
                const float angularSpeed = Len3(body->_angularVelocity);
                float maxHE = minHE;
                if (col->GetKind() == Collider::Kind::Box) {
                    const auto* boxCol = static_cast<const BoxCollider*>(col);
                    const VECTOR he = boxCol->GetHalfExtents();
                    maxHE = std::sqrt(he.x * he.x + he.y * he.y + he.z * he.z);
                } else if (col->GetKind() == Collider::Kind::Capsule) {
                    const auto* capCol = static_cast<const CapsuleCollider*>(col);
                    maxHE = capCol->GetRadius() + Len3(VSub(capCol->GetTop(), capCol->GetBottom())) * 0.5f;
                }
                const float angularSurfaceSpeed = angularSpeed * maxHE;
                const float effectiveSpeed = linearSpeed + angularSurfaceSpeed;
                if (stepDt > 1e-6f && effectiveSpeed > 1e-6f) {
                    const float maxDisplacement = allowedPen;
                    const float toi = maxDisplacement / effectiveSpeed;
                    if (toi < stepDt) {
                        // Critical quality: halve the allowed displacement for stricter CCD
                        const float qualityScale = (body->_ccdQuality == CcdQuality::Critical) ? 0.5f : 1.0f;
                        const float clampDisp = maxDisplacement * qualityScale;
                        const float linearFraction = (effectiveSpeed > 1e-6f) ? (linearSpeed / effectiveSpeed) : 1.0f;
                        ClampMagnitude(body->_velocity, clampDisp * linearFraction / stepDt);
                        if (!body->_freezeRotation && angularSurfaceSpeed > 1e-6f) {
                            const float angFraction = 1.0f - linearFraction;
                            const float maxAngDisp = clampDisp * angFraction;
                            if (maxHE > 1e-6f) {
                                const float maxAngSpeed = maxAngDisp / (maxHE * stepDt);
                                ClampMagnitude(body->_angularVelocity, maxAngSpeed);
                            }
                        }
                    }
                }
            }
        }

        VECTOR pos = body->_owner->transform.LocalPosition();
        pos = VAdd(pos, VScale(body->_velocity, stepDt));

        // Half-plane ground constraint
        if (groundEnabled) {
            const float dist = HalfPlaneDistance(pos, groundN, groundD);
            if (dist < 0.0f) {
                // Push back onto plane
                pos = VSub(pos, VScale(groundN, dist));
                // Remove velocity into the plane
                const float vn = Dot3(body->_velocity, groundN);
                if (vn < 0.0f) {
                    body->_velocity = VSub(body->_velocity, VScale(groundN, vn));
                }
                // Ground friction on angular velocity
                if (!body->_freezeRotation && body->_friction > 0.0f) {
                    body->_angularVelocity = VScale(body->_angularVelocity,
                        1.0f / (1.0f + body->_friction * 0.5f * stepDt));
                }
            }
        }
        body->_owner->transform.SetLocalPosition(pos);

        if (!body->_freezeRotation) {
            // Quaternion-differential angular integration:
            // q' = q + 0.5 * Quaternion(0, ω) * q * dt
            // More accurate than axis-angle for large angular velocities.
            const VECTOR& w = body->_angularVelocity;
            const float angSpd = Len3(w);
            if (angSpd > 1e-6f) {
                Quaternion q = body->_owner->transform.LocalRotation();
                // Quaternion(0, ω) = (wx, wy, wz, 0) ? pure quaternion
                Quaternion wq(w.x * 0.5f * stepDt, w.y * 0.5f * stepDt, w.z * 0.5f * stepDt, 0.0f);
                // dq = wq * q
                Quaternion dq = Quaternion::Multiply(wq, q);
                q.x += dq.x;
                q.y += dq.y;
                q.z += dq.z;
                q.w += dq.w;
                body->_owner->transform.SetLocalRotation(q.Normalized());
            }
        }

        body->_hasMovePositionTarget = false;
        body->_hasMoveRotationTarget = false;
        body->ClearAccumulators();

        SanitizeVec(body->_velocity);
        SanitizeVec(body->_angularVelocity);
        {
            VECTOR p = body->_owner->transform.LocalPosition();
            if (!IsFiniteVec(p)) {
                body->_owner->transform.SetLocalPosition(body->_previousPosition);
                body->_velocity = VGet(0,0,0);
                body->_angularVelocity = VGet(0,0,0);
            }
        }
    }, 4);
}

// ============================================================
//  BuildSolverContacts ? static/kinetic friction, persistent cache
// ============================================================
void PhysicsManager::BuildSolverContacts(float stepDt) {
    const auto& rawContacts = ColliderManager::Instance().GetContacts();
    _solverContacts.clear();
    _solverContacts.reserve(rawContacts.size());

    const float invDt = (stepDt > 1e-6f) ? (1.0f / stepDt) : 0.0f;

    for (const auto& ct : rawContacts) {
        if (!ct.a || !ct.b) continue;
        if (!IsFinite(ct.penetration) || ct.penetration <= 0.0f) continue;
        if (!IsFiniteVec(ct.normal)) continue;

        GameObject* ownerA = ct.a->owner;
        GameObject* ownerB = ct.b->owner;
        PhysicsBody* bodyA = CachedFindBody(ownerA);
        PhysicsBody* bodyB = CachedFindBody(ownerB);

        const float invA = (bodyA && bodyA->IsDynamic() && ownerA && ownerA->IsActive()) ? bodyA->InverseMass() : 0.0f;
        const float invB = (bodyB && bodyB->IsDynamic() && ownerB && ownerB->IsActive()) ? bodyB->InverseMass() : 0.0f;
        if (invA + invB <= 1e-8f) continue;

        SolverContact sc{};
        sc.colA = ct.a;
        sc.colB = ct.b;
        sc.normal = SafeNormalize(ct.normal, VGet(0,1,0));
        sc.point = ct.point;
        sc.penetration = (std::min)(ct.penetration, kMaxPen);
        sc.bodyA = bodyA;
        sc.bodyB = bodyB;
        sc.invA = invA;
        sc.invB = invB;

        const VECTOR centerA = ownerA ? ownerA->transform.WorldPosition() : VGet(0,0,0);
        const VECTOR centerB = ownerB ? ownerB->transform.WorldPosition() : VGet(0,0,0);
        sc.rA = VSub(sc.point, centerA);
        sc.rB = VSub(sc.point, centerB);

        ComputeTangentBasis(sc.normal, sc.tangent1, sc.tangent2);
        sc.effectiveInvMassN  = ComputeEffectiveInvMass(invA, invB, bodyA, bodyB, sc.rA, sc.rB, sc.normal);
        sc.effectiveInvMassT1 = ComputeEffectiveInvMass(invA, invB, bodyA, bodyB, sc.rA, sc.rB, sc.tangent1);
        sc.effectiveInvMassT2 = ComputeEffectiveInvMass(invA, invB, bodyA, bodyB, sc.rA, sc.rB, sc.tangent2);
        if (sc.effectiveInvMassN <= 1e-8f) continue;
        if (sc.effectiveInvMassT1 <= 1e-8f) sc.effectiveInvMassT1 = 1e-8f;
        if (sc.effectiveInvMassT2 <= 1e-8f) sc.effectiveInvMassT2 = 1e-8f;

        // Restitution
        float rest = 0.0f;
        if (bodyA && bodyB) rest = PhysicsMaterial::CombineRestitution(bodyA->_material, bodyB->_material);
        else if (bodyA) rest = bodyA->_restitution;
        else if (bodyB) rest = bodyB->_restitution;
        const float vn = RelNormalVelocity(bodyA, bodyB, sc.rA, sc.rB, sc.normal);
        if (std::fabs(vn) < kRestitutionThreshold) rest = 0.0f;
        sc.restitution = rest;

        // Dynamic friction
        if (bodyA && bodyB) sc.friction = PhysicsMaterial::CombineFriction(bodyA->_material, bodyB->_material);
        else if (bodyA) sc.friction = (std::max)(0.0f, bodyA->_friction);
        else if (bodyB) sc.friction = (std::max)(0.0f, bodyB->_friction);

        // Static friction (separate from kinetic)
        if (bodyA && bodyB) sc.staticFriction = PhysicsMaterial::CombineStaticFriction(bodyA->_material, bodyB->_material);
        else if (bodyA) sc.staticFriction = sc.friction * 1.2f;
        else if (bodyB) sc.staticFriction = sc.friction * 1.2f;

        // Baumgarte velocity bias (used when split impulse is disabled)
        sc.normalBias = kBiasFactor * invDt * (std::max)(sc.penetration - kSlop, 0.0f);
        if (vn < -kRestitutionThreshold) {
            sc.normalBias += rest * (-vn);
        }

        // Split impulse bias (position correction separated from velocity)
        sc.splitBias = kSplitBiasFactor * invDt * (std::max)(sc.penetration - kSlop, 0.0f);
        sc.splitNormalLambda = 0.0f;
        sc.speculative = false;

        // Local coordinates for persistent matching
        sc.localA = ownerA ? VSub(sc.point, centerA) : sc.point;
        sc.localB = ownerB ? VSub(sc.point, centerB) : sc.point;

        // Warm-start: match with previous frame's contacts
        sc.normalLambda = 0.0f;
        sc.frictionLambda1 = 0.0f;
        sc.frictionLambda2 = 0.0f;
        for (const auto& prev : _prevSolverContacts) {
            if (prev.colA != sc.colA || prev.colB != sc.colB) continue;
            if (LenSq(VSub(prev.localA, sc.localA)) > kContactMatchDistSq) continue;
            if (LenSq(VSub(prev.localB, sc.localB)) > kContactMatchDistSq) continue;
            sc.normalLambda    = prev.normalLambda;
            sc.frictionLambda1 = prev.frictionLambda1;
            sc.frictionLambda2 = prev.frictionLambda2;
            break;
        }

        _solverContacts.push_back(sc);
    }
}

// ============================================================
//  BuildIslands ? Union-Find on contact graph
// ============================================================
void PhysicsManager::BuildIslands() {
    _islands.clear();
    _bodyIslandMap.clear();
    if (_bodies.empty()) return;

    std::unordered_map<PhysicsBody*, int> bodyIndex;
    bodyIndex.reserve(_bodies.size());
    for (int i = 0; i < static_cast<int>(_bodies.size()); ++i) {
        if (_bodies[i]) bodyIndex[_bodies[i]] = i;
    }

    const int n = static_cast<int>(_bodies.size());
    std::vector<int> parent(n);
    for (int i = 0; i < n; ++i) parent[i] = i;

    auto Find = [&](int x) -> int {
        while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
        return x;
    };
    auto Unite = [&](int a, int b) {
        a = Find(a); b = Find(b);
        if (a != b) parent[a] = b;
    };

    for (int ci = 0; ci < static_cast<int>(_solverContacts.size()); ++ci) {
        auto& sc = _solverContacts[ci];
        int ia = -1, ib = -1;
        if (sc.bodyA) { auto it = bodyIndex.find(sc.bodyA); if (it != bodyIndex.end()) ia = it->second; }
        if (sc.bodyB) { auto it = bodyIndex.find(sc.bodyB); if (it != bodyIndex.end()) ib = it->second; }
        if (ia >= 0 && ib >= 0) Unite(ia, ib);
    }

    std::unordered_map<int, int> rootToIsland;
    rootToIsland.reserve(n);
    for (int i = 0; i < n; ++i) {
        if (!_bodies[i]) continue;
        const int root = Find(i);
        auto it = rootToIsland.find(root);
        int islandIdx;
        if (it == rootToIsland.end()) {
            islandIdx = static_cast<int>(_islands.size());
            rootToIsland[root] = islandIdx;
            _islands.emplace_back();
        } else {
            islandIdx = it->second;
        }
        _islands[islandIdx].bodies.push_back(_bodies[i]);
        _bodyIslandMap[_bodies[i]] = islandIdx;
    }

    for (int ci = 0; ci < static_cast<int>(_solverContacts.size()); ++ci) {
        auto& sc = _solverContacts[ci];
        PhysicsBody* repBody = sc.bodyA ? sc.bodyA : sc.bodyB;
        if (!repBody) continue;
        auto it = _bodyIslandMap.find(repBody);
        if (it == _bodyIslandMap.end()) continue;
        sc.islandId = it->second;
        _islands[it->second].contactIndices.push_back(ci);
    }

    for (auto& island : _islands) {
        bool allSleep = true;
        for (auto* body : island.bodies) {
            if (body && body->IsDynamic() && !body->_isSleeping) { allSleep = false; break; }
        }
        island.allSleeping = allSleep;
    }
}

// ============================================================
//  WarmStart
// ============================================================
void PhysicsManager::WarmStart() {
    for (auto& sc : _solverContacts) {
        if (sc.effectiveInvMassN <= 1e-8f) continue;

        const VECTOR warmN  = VScale(sc.normal,   sc.normalLambda    * kWarmStartFactor);
        const VECTOR warmT1 = VScale(sc.tangent1, sc.frictionLambda1 * kWarmStartFactor);
        const VECTOR warmT2 = VScale(sc.tangent2, sc.frictionLambda2 * kWarmStartFactor);
        const VECTOR totalImpulse = VAdd(VAdd(warmN, warmT1), warmT2);

        ApplyImpulse(sc.bodyA, sc.bodyB, sc.invA, sc.invB, sc.rA, sc.rB, totalImpulse);

        if (sc.bodyA) sc.bodyA->WakeUp();
        if (sc.bodyB) sc.bodyB->WakeUp();
    }
}

// ============================================================
//  SolveIsland ? circular friction cone + split impulse
// ============================================================
void PhysicsManager::SolveIsland(const PhysicsIsland& island, float /*stepDt*/) {
    for (int ci : island.contactIndices) {
        SolverContact& sc = _solverContacts[ci];
        if (sc.effectiveInvMassN <= 1e-8f) continue;

        // --- Normal constraint with accumulated clamping ---
        {
            const float vn = RelNormalVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.normal);
            // When split impulse is enabled, the Baumgarte bias is separated out;
            // only restitution bias goes into the velocity solver.
            float bias = 0.0f;
            if (!_splitImpulseEnabled) {
                bias = sc.normalBias;
            } else {
                // Only restitution bias in velocity solver when split impulse is active
                if (vn < -kRestitutionThreshold) {
                    bias = sc.restitution * (-vn);
                }
            }

            // For speculative contacts, only allow non-negative normal impulse
            // and clamp bias to only remove the approaching velocity component.
            // This prevents speculative contacts from injecting separation energy.
            if (sc.speculative) {
                // Havok-style: bias = min(normalBias, -vn) to only stop approach
                const float approachSpeed = (std::max)(-vn, 0.0f);
                bias = (std::min)(sc.normalBias, approachSpeed);
            }

            float deltaLambda = (-vn + bias) / sc.effectiveInvMassN;
            const float oldLambda = sc.normalLambda;
            sc.normalLambda = (std::max)(oldLambda + deltaLambda, 0.0f);
            deltaLambda = sc.normalLambda - oldLambda;

            ApplyImpulse(sc.bodyA, sc.bodyB, sc.invA, sc.invB, sc.rA, sc.rB,
                VScale(sc.normal, deltaLambda));
        }

        // --- Friction: circular Coulomb cone (2D) ---
        // Compute tangent relative velocities
        const float vt1 = RelDirVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.tangent1);
        const float vt2 = RelDirVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.tangent2);
        const float tangentSpeed = std::sqrt(vt1 * vt1 + vt2 * vt2);

        // Use static friction if tangent speed is low, otherwise kinetic
        const float frictionToUse = (tangentSpeed < 0.1f) ? sc.staticFriction : sc.friction;
        const float maxFriction = frictionToUse * sc.normalLambda;

        // Solve both tangent axes
        float dLambda1 = -vt1 / sc.effectiveInvMassT1;
        float dLambda2 = -vt2 / sc.effectiveInvMassT2;
        float newF1 = sc.frictionLambda1 + dLambda1;
        float newF2 = sc.frictionLambda2 + dLambda2;

        // Circular cone clamp: √(f1? + f2?) ? μN
        const float fMag = std::sqrt(newF1 * newF1 + newF2 * newF2);
        if (fMag > maxFriction && fMag > 1e-8f) {
            const float scale = maxFriction / fMag;
            newF1 *= scale;
            newF2 *= scale;
        }

        dLambda1 = newF1 - sc.frictionLambda1;
        dLambda2 = newF2 - sc.frictionLambda2;
        sc.frictionLambda1 = newF1;
        sc.frictionLambda2 = newF2;

        const VECTOR frictionImpulse = VAdd(
            VScale(sc.tangent1, dLambda1),
            VScale(sc.tangent2, dLambda2));
        ApplyImpulse(sc.bodyA, sc.bodyB, sc.invA, sc.invB, sc.rA, sc.rB, frictionImpulse);
    }
}

// ============================================================
//  SolveAllIslands ? parallel, adaptive iterations
// ============================================================
void PhysicsManager::SolveAllIslands(float stepDt) {
    if (_islands.empty()) return;

    for (int iter = 0; iter < _solverIterations; ++iter) {
        const size_t islandCount = _islands.size();
        ThreadPool::Instance().ParallelFor(0, islandCount, [&](size_t i) {
            if (_islands[i].allSleeping) return;
            if (_islands[i].contactIndices.empty()) return;
            SolveIsland(_islands[i], stepDt);
        }, 1);
    }

    for (auto* body : _bodies) {
        if (!body) continue;
        SanitizeVec(body->_velocity);
        SanitizeVec(body->_angularVelocity);
    }
}

// ============================================================
//  PositionalCorrection ? half-plane aware (fallback when split impulse is off)
// ============================================================
void PhysicsManager::PositionalCorrection(float /*stepDt*/) {
    for (const auto& sc : _solverContacts) {
        if (sc.penetration <= kSlop) continue;
        const float invSum = sc.invA + sc.invB;
        if (invSum <= 1e-8f) continue;

        const float clampedPen = (std::min)(sc.penetration, kMaxPen);
        float correctionMag = kBiasFactor * (std::max)(clampedPen - kSlop, 0.0f) / invSum;
        correctionMag = (std::min)(correctionMag, kMaxCorrection);
        if (correctionMag <= 1e-6f) continue;

        GameObject* ownerA = sc.colA ? sc.colA->owner : nullptr;
        GameObject* ownerB = sc.colB ? sc.colB->owner : nullptr;

        if (sc.bodyA && sc.invA > 0.0f && ownerA) {
            VECTOR p = ownerA->transform.LocalPosition();
            p = VSub(p, VScale(sc.normal, correctionMag * sc.invA));
            if (IsFiniteVec(p)) {
                if (_groundPlaneEnabled && HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD) < 0.0f)
                    p = VSub(p, VScale(_groundPlaneNormal, HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD)));
                ownerA->transform.SetLocalPosition(p);
            }
        }
        if (sc.bodyB && sc.invB > 0.0f && ownerB) {
            VECTOR p = ownerB->transform.LocalPosition();
            p = VAdd(p, VScale(sc.normal, correctionMag * sc.invB));
            if (IsFiniteVec(p)) {
                if (_groundPlaneEnabled && HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD) < 0.0f)
                    p = VSub(p, VScale(_groundPlaneNormal, HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD)));
                ownerB->transform.SetLocalPosition(p);
            }
        }
    }
}

// ============================================================
//  Sleep / Constraints
// ============================================================
void PhysicsManager::UpdateSleepState(PhysicsBody* body, float stepDt) {
    if (!body || !body->IsDynamic()) return;
    if (body->_hasMovePositionTarget || body->_hasMoveRotationTarget) { body->WakeUp(); return; }
    if (LenSq(body->_force) > 1e-8f || LenSq(body->_torque) > 1e-8f) { body->WakeUp(); return; }

    const float lSq = body->_sleepLinearThreshold * body->_sleepLinearThreshold;
    const float aSq = body->_sleepAngularThreshold * body->_sleepAngularThreshold;
    if (LenSq(body->_velocity) > lSq || LenSq(body->_angularVelocity) > aSq) { body->WakeUp(); return; }

    body->_sleepTimer += stepDt;
    if (body->_sleepTimer >= body->_sleepTimeThreshold) body->Sleep();
}

// ============================================================
//  SplitImpulseCorrection ? position correction without affecting velocity
// ============================================================
// Uses a separate "pseudo-velocity" channel to resolve penetration.
// This avoids the energy-injection problem of Baumgarte stabilization
// while still preventing visual overlap.
void PhysicsManager::SplitImpulseCorrection(float /*stepDt*/) {
    // Per-body pseudo-velocity for position correction only
    std::unordered_map<PhysicsBody*, VECTOR> pseudoVel;
    pseudoVel.reserve(_bodies.size());
    for (auto* body : _bodies) {
        if (body) pseudoVel[body] = VGet(0, 0, 0);
    }

    // Iterative solve (fewer iterations needed than velocity solver)
    const int posIter = (std::max)(_solverIterations / 2, 2);
    for (int iter = 0; iter < posIter; ++iter) {
        for (auto& sc : _solverContacts) {
            if (sc.penetration <= kSlop) continue;
            if (sc.effectiveInvMassN <= 1e-8f) continue;

            // Compute pseudo relative normal velocity
            VECTOR pvA = pseudoVel.count(sc.bodyA) ? pseudoVel[sc.bodyA] : VGet(0,0,0);
            VECTOR pvB = pseudoVel.count(sc.bodyB) ? pseudoVel[sc.bodyB] : VGet(0,0,0);
            const float pvn = Dot3(VSub(pvB, pvA), sc.normal);

            float deltaLambda = (-pvn + sc.splitBias) / sc.effectiveInvMassN;
            const float oldLambda = sc.splitNormalLambda;
            sc.splitNormalLambda = (std::max)(oldLambda + deltaLambda, 0.0f);
            deltaLambda = sc.splitNormalLambda - oldLambda;
            if (std::fabs(deltaLambda) < 1e-8f) continue;

            const VECTOR impulse = VScale(sc.normal, deltaLambda);
            if (sc.bodyA && sc.invA > 0.0f)
                pseudoVel[sc.bodyA] = VSub(pseudoVel[sc.bodyA], VScale(impulse, sc.invA));
            if (sc.bodyB && sc.invB > 0.0f)
                pseudoVel[sc.bodyB] = VAdd(pseudoVel[sc.bodyB], VScale(impulse, sc.invB));
        }
    }

    // Apply accumulated pseudo-velocity as position correction
    for (auto& [body, pv] : pseudoVel) {
        if (!body || !body->_owner) continue;
        if (LenSq(pv) < 1e-10f) continue;
        VECTOR p = body->_owner->transform.LocalPosition();
        // Scale by fixedDeltaTime to convert pseudo-velocity to displacement
        VECTOR correction = VScale(pv, _fixedDeltaTime);
        ClampMagnitude(correction, kMaxCorrection);
        p = VAdd(p, correction);
        if (IsFiniteVec(p)) {
            if (_groundPlaneEnabled && HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD) < 0.0f)
                p = VSub(p, VScale(_groundPlaneNormal, HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD)));
            body->_owner->transform.SetLocalPosition(p);
        }
    }
}

// ============================================================
//  GenerateSpeculativeContacts ? predict future contacts (all pairs)
// ============================================================
// For CCD-enabled bodies, predict where they will be next step and
// create "virtual" contacts to prevent tunneling before it happens.
// Supports: ground plane + all registered collider pairs.
void PhysicsManager::GenerateSpeculativeContacts(float stepDt) {
    if (stepDt <= 1e-6f) return;
    const float invDt = 1.0f / stepDt;

    for (auto* body : _bodies) {
        if (!body || !body->IsDynamic() || !body->_detectContinuous) continue;
        if (!body->_owner || !body->_owner->IsActive()) continue;

        const float speed = Len3(body->_velocity);
        if (speed < 1.0f) continue; // Only for fast-moving bodies

        Collider* col = CachedFindCollider(body->_owner);
        if (!col) continue;

        const float minHE = GetColliderMinHalfExtent(col);
        const float predictedDisplacement = speed * stepDt;
        if (predictedDisplacement < minHE) continue; // Not fast enough to tunnel

        // Predict future position
        const VECTOR curPos = body->_owner->transform.WorldPosition();
        const VECTOR predictedPos = VAdd(curPos, VScale(body->_velocity, stepDt));

        // Predicted AABB: union of current and predicted position enlarged by minHE
        AABB predictedAABB;
        {
            const AABB& curAABB = col->GetAABB();
            const VECTOR offset = VScale(body->_velocity, stepDt);
            predictedAABB.min.x = (std::min)(curAABB.min.x, curAABB.min.x + offset.x) - kSpeculativeMargin;
            predictedAABB.min.y = (std::min)(curAABB.min.y, curAABB.min.y + offset.y) - kSpeculativeMargin;
            predictedAABB.min.z = (std::min)(curAABB.min.z, curAABB.min.z + offset.z) - kSpeculativeMargin;
            predictedAABB.max.x = (std::max)(curAABB.max.x, curAABB.max.x + offset.x) + kSpeculativeMargin;
            predictedAABB.max.y = (std::max)(curAABB.max.y, curAABB.max.y + offset.y) + kSpeculativeMargin;
            predictedAABB.max.z = (std::max)(curAABB.max.z, curAABB.max.z + offset.z) + kSpeculativeMargin;
            predictedAABB.center = VScale(VAdd(predictedAABB.min, predictedAABB.max), 0.5f);
        }

        // Check against ground plane if enabled
        if (_groundPlaneEnabled) {
            const float dist = HalfPlaneDistance(predictedPos, _groundPlaneNormal, _groundPlaneD);
            if (dist < minHE + kSpeculativeMargin) {
                SolverContact sc{};
                sc.colA = col;
                sc.colB = nullptr;
                sc.normal = _groundPlaneNormal;
                sc.point = VSub(predictedPos, VScale(_groundPlaneNormal, dist));
                sc.penetration = minHE + kSpeculativeMargin - dist;
                sc.bodyA = body;
                sc.bodyB = nullptr;
                sc.invA = body->InverseMass();
                sc.invB = 0.0f;
                sc.rA = VSub(sc.point, curPos);
                sc.rB = VGet(0,0,0);
                ComputeTangentBasis(sc.normal, sc.tangent1, sc.tangent2);
                sc.effectiveInvMassN = ComputeEffectiveInvMass(sc.invA, 0.0f, body, nullptr, sc.rA, sc.rB, sc.normal);
                sc.effectiveInvMassT1 = ComputeEffectiveInvMass(sc.invA, 0.0f, body, nullptr, sc.rA, sc.rB, sc.tangent1);
                sc.effectiveInvMassT2 = ComputeEffectiveInvMass(sc.invA, 0.0f, body, nullptr, sc.rA, sc.rB, sc.tangent2);
                if (sc.effectiveInvMassN <= 1e-8f) continue;
                if (sc.effectiveInvMassT1 <= 1e-8f) sc.effectiveInvMassT1 = 1e-8f;
                if (sc.effectiveInvMassT2 <= 1e-8f) sc.effectiveInvMassT2 = 1e-8f;
                sc.friction = body->_friction;
                sc.staticFriction = sc.friction * 1.2f;
                sc.restitution = 0.0f;
                sc.normalBias = kSplitBiasFactor * invDt * (std::max)(sc.penetration - kSlop, 0.0f);
                sc.splitBias = sc.normalBias;
                sc.localA = sc.rA;
                sc.localB = VGet(0,0,0);
                sc.speculative = true;
                _solverContacts.push_back(sc);
            }
        }

        // Check against all other colliders for speculative contacts
        const auto& allColliders = ColliderManager::Instance().GetColliders();
        for (auto* otherCol : allColliders) {
            if (!otherCol || otherCol == col) continue;
            if (otherCol->owner == body->_owner) continue; // Skip self

            // Deduplication: skip if a real contact already exists for this pair
            bool alreadyHasContact = false;
            for (const auto& existing : _solverContacts) {
                if (existing.speculative) continue;
                if ((existing.colA == col && existing.colB == otherCol) ||
                    (existing.colA == otherCol && existing.colB == col)) {
                    alreadyHasContact = true;
                    break;
                }
            }
            if (alreadyHasContact) continue;

            // AABB overlap with predicted AABB
            const AABB& otherAABB = otherCol->GetAABB();
            if (predictedAABB.min.x > otherAABB.max.x || predictedAABB.max.x < otherAABB.min.x) continue;
            if (predictedAABB.min.y > otherAABB.max.y || predictedAABB.max.y < otherAABB.min.y) continue;
            if (predictedAABB.min.z > otherAABB.max.z || predictedAABB.max.z < otherAABB.min.z) continue;

            // Compute approximate closest approach direction and distance
            const VECTOR otherCenter = otherCol->GetCenter();
            const VECTOR toPredicted = VSub(predictedPos, otherCenter);
            const float approachDist = Len3(toPredicted);
            const float otherMinHE = GetColliderMinHalfExtent(otherCol);
            const float combinedExtent = minHE + otherMinHE + kSpeculativeMargin;

            if (approachDist > combinedExtent * 3.0f) continue; // Too far for speculative

            // Estimate speculative penetration: how much the bodies will overlap
            const VECTOR toOther = VSub(otherCenter, curPos);
            const float distNow = Len3(toOther);
            const float closingSpeed = -Dot3(body->_velocity, SafeNormalize(toOther)) * stepDt;
            const float specPen = combinedExtent - (distNow - closingSpeed);
            if (specPen <= 0.0f) continue;

            // Generate speculative contact
            const VECTOR normal = SafeNormalize(VSub(curPos, otherCenter), VGet(0, 1, 0));
            const VECTOR contactPt = VAdd(otherCenter, VScale(normal, otherMinHE));

            PhysicsBody* otherBody = CachedFindBody(otherCol->owner);
            const float otherInv = (otherBody && otherBody->IsDynamic() && otherCol->owner && otherCol->owner->IsActive())
                ? otherBody->InverseMass() : 0.0f;

            if (body->InverseMass() + otherInv <= 1e-8f) continue;

            SolverContact sc{};
            sc.colA = col;
            sc.colB = otherCol;
            sc.normal = normal;
            sc.point = contactPt;
            sc.penetration = (std::min)(specPen, kMaxPen);
            sc.bodyA = body;
            sc.bodyB = otherBody;
            sc.invA = body->InverseMass();
            sc.invB = otherInv;
            sc.rA = VSub(sc.point, curPos);
            sc.rB = VSub(sc.point, otherCenter);
            ComputeTangentBasis(sc.normal, sc.tangent1, sc.tangent2);
            sc.effectiveInvMassN = ComputeEffectiveInvMass(sc.invA, sc.invB, body, otherBody, sc.rA, sc.rB, sc.normal);
            sc.effectiveInvMassT1 = ComputeEffectiveInvMass(sc.invA, sc.invB, body, otherBody, sc.rA, sc.rB, sc.tangent1);
            sc.effectiveInvMassT2 = ComputeEffectiveInvMass(sc.invA, sc.invB, body, otherBody, sc.rA, sc.rB, sc.tangent2);
            if (sc.effectiveInvMassN <= 1e-8f) continue;
            if (sc.effectiveInvMassT1 <= 1e-8f) sc.effectiveInvMassT1 = 1e-8f;
            if (sc.effectiveInvMassT2 <= 1e-8f) sc.effectiveInvMassT2 = 1e-8f;

            if (body && otherBody)
                sc.friction = PhysicsMaterial::CombineFriction(body->_material, otherBody->_material);
            else
                sc.friction = body->_friction;
            sc.staticFriction = sc.friction * 1.2f;
            sc.restitution = 0.0f;
            sc.normalBias = kSplitBiasFactor * invDt * (std::max)(sc.penetration - kSlop, 0.0f);
            sc.splitBias = sc.normalBias;
            sc.localA = sc.rA;
            sc.localB = sc.rB;
            sc.speculative = true;

            // Contact welding: skip if too close to an existing speculative contact
            // for the same body pair (prevents solver artifacts from duplicate contacts)
            bool welded = false;
            constexpr float kWeldDistSq = 0.01f; // 0.1 world units
            for (const auto& existing : _solverContacts) {
                if (!existing.speculative) continue;
                if (existing.bodyA != body && existing.bodyB != body) continue;
                if (LenSq(VSub(existing.point, sc.point)) < kWeldDistSq &&
                    Dot3(existing.normal, sc.normal) > 0.9f) {
                    welded = true;
                    break;
                }
            }
            if (welded) continue;

            _solverContacts.push_back(sc);
        }
    }
}

// ============================================================
//  ResolveToiEvents ? sub-TOI stepping with time-sorted events
// ============================================================
// Detect CCD bodies that have tunneled through thin geometry by
// comparing displacement to collider thickness. Collect TOI events,
// sort by earliest time, backstep bodies to TOI positions, and
// re-integrate the remaining time fraction.
void PhysicsManager::ResolveToiEvents(float stepDt) {
    if (stepDt <= 1e-6f) return;

    struct ToiEvent {
        PhysicsBody* body = nullptr;
        float toi = 1.0f;          // Fraction [0..1] within stepDt
        VECTOR toiPosition{};       // Position at TOI
        VECTOR clampedVelocity{};   // Velocity after backstep
    };

    std::vector<ToiEvent> events;

    for (auto* body : _bodies) {
        if (!body || !body->IsDynamic() || !body->_detectContinuous) continue;
        // Only Bullet and Critical quality get TOI backstep
        if (body->_ccdQuality < CcdQuality::Bullet) continue;
        if (!body->_owner || !body->_owner->IsActive()) continue;
        if (body->_isSleeping) continue;

        const float speed = Len3(body->_velocity);
        if (speed < 1e-4f) continue;

        Collider* col = CachedFindCollider(body->_owner);
        if (!col) continue;

        const float minHE = GetColliderMinHalfExtent(col);
        // Use allowedPenetrationDepth if set
        const float allowedPen = (body->_allowedPenetrationDepth > 0.0f)
            ? body->_allowedPenetrationDepth : (minHE * 0.8f);
        const float displacement = speed * stepDt;
        if (displacement <= allowedPen) continue;

        const float toi = allowedPen / (speed * stepDt);
        if (toi >= 1.0f) continue;

        ToiEvent ev;
        ev.body = body;
        ev.toi = (std::max)(0.0f, toi);
        ev.toiPosition = VAdd(body->_previousPosition, VScale(body->_velocity, ev.toi * stepDt));
        ev.clampedVelocity = body->_velocity;
        events.push_back(ev);
    }

    if (events.empty()) return;

    // Sort by earliest TOI first
    std::sort(events.begin(), events.end(), [](const ToiEvent& a, const ToiEvent& b) {
        return a.toi < b.toi;
    });

    // Process events in time order
    for (auto& ev : events) {
        if (!ev.body || !ev.body->_owner) continue;

        // Backstep body to TOI position
        ev.body->_owner->transform.SetLocalPosition(ev.toiPosition);

        // Re-integrate remaining time: (1 - toi) * stepDt
        const float remainDt = (1.0f - ev.toi) * stepDt;
        if (remainDt > 1e-6f) {
            Collider* col = CachedFindCollider(ev.body->_owner);
            const float allowedPen = col
                ? ((ev.body->_allowedPenetrationDepth > 0.0f)
                    ? ev.body->_allowedPenetrationDepth : (GetColliderMinHalfExtent(col) * 0.8f))
                : 0.4f;
            const float remainSpeed = Len3(ev.body->_velocity);
            if (remainSpeed * remainDt > allowedPen) {
                ClampMagnitude(ev.body->_velocity, allowedPen / remainDt);
            }

            // Advance by remainder
            VECTOR pos = ev.body->_owner->transform.LocalPosition();
            pos = VAdd(pos, VScale(ev.body->_velocity, remainDt));

            // Ground plane clamp
            if (_groundPlaneEnabled) {
                const float dist = HalfPlaneDistance(pos, _groundPlaneNormal, _groundPlaneD);
                if (dist < 0.0f) {
                    pos = VSub(pos, VScale(_groundPlaneNormal, dist));
                    const float vn = Dot3(ev.body->_velocity, _groundPlaneNormal);
                    if (vn < 0.0f) {
                        // Reflect velocity component into the plane (TOI collision response)
                        ev.body->_velocity = VSub(ev.body->_velocity, VScale(_groundPlaneNormal, vn * (1.0f + ev.body->_restitution)));
                    }
                }
            }
            ev.body->_owner->transform.SetLocalPosition(pos);
        }

        // Update collider shape after repositioning
        Collider* col = CachedFindCollider(ev.body->_owner);
        if (col) col->UpdateShape();
    }

    // Re-run narrow-phase for repositioned bodies
    ColliderManager::Instance().Update(stepDt);
    BuildLookupCaches();
}

// ============================================================
//  PropagateIslandSleep ? if any body in island is awake, wake all
// ============================================================
void PhysicsManager::PropagateIslandSleep() {
    for (auto& island : _islands) {
        // Check if any body in the island is awake
        bool anyAwake = false;
        for (auto* body : island.bodies) {
            if (body && body->IsDynamic() && !body->_isSleeping) {
                anyAwake = true;
                break;
            }
        }
        // If at least one body is awake, wake all bodies in the island
        if (anyAwake) {
            for (auto* body : island.bodies) {
                if (body && body->IsDynamic() && body->_isSleeping) {
                    body->WakeUp();
                }
            }
        }
    }
}

// ============================================================
//  Interpolation
// ============================================================
float PhysicsManager::InterpolationAlpha() const noexcept {
    if (_fixedDeltaTime <= 1e-6f) return 1.0f;
    return (std::min)(_accumulator / _fixedDeltaTime, 1.0f);
}

void PhysicsManager::ComputeInterpolation() noexcept {
    const float alpha = InterpolationAlpha();
    for (auto* body : _bodies) {
        if (!body || !body->_owner) continue;
        if (!body->_useInterpolation) {
            body->_interpPosition = body->_owner->transform.LocalPosition();
            body->_interpRotation = body->_owner->transform.LocalRotation();
            continue;
        }
        // Lerp position, slerp rotation
        const VECTOR curPos = body->_owner->transform.LocalPosition();
        body->_interpPosition = VAdd(
            VScale(body->_previousPosition, 1.0f - alpha),
            VScale(curPos, alpha));
        body->_interpRotation = Quaternion::Slerp(
            body->_previousRotation,
            body->_owner->transform.LocalRotation(),
            alpha);
    }
}

void PhysicsManager::ApplyBodyConstraints(PhysicsBody* body) const {
    if (!body) return;
    if (body->_freezeRotation) body->_angularVelocity = VGet(0,0,0);
    if (body->_isSleeping) { body->_velocity = VGet(0,0,0); body->_angularVelocity = VGet(0,0,0); }
}

PhysicsBody* PhysicsManager::FindBodyByOwner(GameObject* owner) const {
    if (!owner) return nullptr;
    for (auto* body : _bodies) {
        if (body && body->_owner == owner) return body;
    }
    return nullptr;
}

// ============================================================
//  Registration
// ============================================================
void PhysicsManager::Register(PhysicsController* controller) {
    if (IsShuttingDown() || !controller) return;
    std::lock_guard lk(_mtx);
    if (std::find(_controllers.begin(), _controllers.end(), controller) != _controllers.end()) return;
    _controllers.push_back(controller);
}

void PhysicsManager::Unregister(PhysicsController* controller) {
    if (IsShuttingDown() || !controller) return;
    std::lock_guard lk(_mtx);
    auto it = std::remove(_controllers.begin(), _controllers.end(), controller);
    _controllers.erase(it, _controllers.end());
}

void PhysicsManager::RegisterBody(PhysicsBody* body) {
    if (IsShuttingDown() || !body) return;
    std::lock_guard lk(_mtx);
    if (std::find(_bodies.begin(), _bodies.end(), body) != _bodies.end()) return;
    _bodies.push_back(body);
    if (body->_owner) {
        body->_previousPosition = body->_owner->transform.LocalPosition();
        body->_previousRotation = body->_owner->transform.LocalRotation();
        Collider* col = ColliderManager::Instance().FindColliderByOwner(body->_owner);
        body->ComputeInertia(col);
    }
}

void PhysicsManager::UnregisterBody(PhysicsBody* body) {
    if (IsShuttingDown() || !body) return;
    std::lock_guard lk(_mtx);
    auto it = std::remove(_bodies.begin(), _bodies.end(), body);
    _bodies.erase(it, _bodies.end());
}
