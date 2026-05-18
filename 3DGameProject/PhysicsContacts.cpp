#include "PhysicsManager_Internal.h"

void PhysicsManager::BuildSolverContacts(float stepDt) {
    const auto& rawContacts = ColliderManager::Instance().GetContacts();
    _solverContacts.clear();

    const size_t rawCount = rawContacts.size();
    if (rawCount == 0) return;

    const float invDt = (stepDt > 1e-6f) ? (1.0f / stepDt) : 0.0f;

    auto& prevMap = _prevContactMapBuf;
    prevMap.clear();
    const size_t needed = _prevSolverContacts.size();
    if (needed > 0 && prevMap.bucket_count() < needed * 2)
        prevMap.reserve(needed);
    for (size_t i = 0; i < _prevSolverContacts.size(); ++i) {
        const auto& p = _prevSolverContacts[i];
        prevMap.emplace(PrevKey{p.colA, p.colB}, i);
    }

    auto& results = _buildContactResultsBuf;
    results.resize(rawCount);
    for (size_t i = 0; i < rawCount; ++i) results[i].valid = false;

    ThreadPool::Instance().ParallelForBarrier(0, rawCount, [&](size_t idx) {
        const auto& ct = rawContacts[idx];
        if (!ct.a || !ct.b) return;
        if (!IsFinite(ct.penetration) || ct.penetration <= 0.0f) return;
        if (!IsFiniteVec(ct.normal)) return;

        GameObject* ownerA = ct.a->owner;
        GameObject* ownerB = ct.b->owner;
        PhysicsBody* bodyA = CachedFindBody(ownerA);
        PhysicsBody* bodyB = CachedFindBody(ownerB);

        const float invA = (bodyA && bodyA->IsDynamic() && ownerA && ownerA->IsActive()) ? bodyA->InverseMass() : 0.0f;
        const float invB = (bodyB && bodyB->IsDynamic() && ownerB && ownerB->IsActive()) ? bodyB->InverseMass() : 0.0f;
        if (invA + invB <= 1e-8f) return;

        float clampedInvA = invA;
        float clampedInvB = invB;
        if (invA > 0.0f && invB > 0.0f) {
            const float massRatio = (std::max)(invA / invB, invB / invA);
            if (massRatio > kMaxMassRatio) {
                const float scale = std::sqrt(kMaxMassRatio / massRatio);
                if (invA > invB) {
                    clampedInvA *= scale;
                } else {
                    clampedInvB *= scale;
                }
            }
        }

        SolverContact sc{};
        sc.colA       = ct.a;
        sc.colB       = ct.b;
        sc.normal     = SafeNormalize(ct.normal, VGet(0,1,0));
        sc.point      = ct.point;
        sc.penetration = (std::min)(ct.penetration, kMaxPen);
        sc.bodyA = bodyA;
        sc.bodyB = bodyB;
        sc.invA  = clampedInvA;
        sc.invB  = clampedInvB;

        const VECTOR centerA = ownerA ? ownerA->transform.WorldPosition() : VGet(0,0,0);
        const VECTOR centerB = ownerB ? ownerB->transform.WorldPosition() : VGet(0,0,0);
        sc.rA = VSub(sc.point, centerA);
        sc.rB = VSub(sc.point, centerB);

        ComputeTangentBasis(sc.normal, sc.tangent1, sc.tangent2);

        sc.effectiveInvMassN  = ComputeEffectiveInvMass(clampedInvA, clampedInvB, bodyA, bodyB, sc.rA, sc.rB, sc.normal);
        sc.effectiveInvMassT1 = ComputeEffectiveInvMass(clampedInvA, clampedInvB, bodyA, bodyB, sc.rA, sc.rB, sc.tangent1);
        sc.effectiveInvMassT2 = ComputeEffectiveInvMass(clampedInvA, clampedInvB, bodyA, bodyB, sc.rA, sc.rB, sc.tangent2);
        if (sc.effectiveInvMassN <= 1e-8f) return;
        if (sc.effectiveInvMassT1 <= 1e-8f) sc.effectiveInvMassT1 = 1e-8f;
        if (sc.effectiveInvMassT2 <= 1e-8f) sc.effectiveInvMassT2 = 1e-8f;

        float rest = 0.0f;
        if (bodyA && bodyB) rest = PhysicsMaterial::CombineRestitution(bodyA->_material, bodyB->_material);
        else if (bodyA) rest = bodyA->_restitution;
        else if (bodyB) rest = bodyB->_restitution;
        const float vn = RelNormalVelocity(bodyA, bodyB, sc.rA, sc.rB, sc.normal);
        if (std::fabs(vn) < kRestitutionThreshold) rest = 0.0f;
        sc.restitution = rest;

        if (bodyA && bodyB)  sc.friction = PhysicsMaterial::CombineFriction(bodyA->_material, bodyB->_material);
        else if (bodyA)      sc.friction = (std::max)(0.0f, bodyA->_friction);
        else if (bodyB)      sc.friction = (std::max)(0.0f, bodyB->_friction);
        if (bodyA && bodyB)  sc.staticFriction = PhysicsMaterial::CombineStaticFriction(bodyA->_material, bodyB->_material);
        else                 sc.staticFriction  = sc.friction * 1.2f;

        sc.normalBias = kBiasFactor * invDt * (std::max)(sc.penetration - kSlop, 0.0f);
        if (vn < -kRestitutionThreshold) sc.normalBias += rest * (-vn);
        sc.splitBias         = kSplitBiasFactor * invDt * (std::max)(sc.penetration - kSlop, 0.0f);
        sc.splitNormalLambda = 0.0f;
        sc.speculative       = false;

        sc.localA = ownerA ? VSub(sc.point, centerA) : sc.point;
        sc.localB = ownerB ? VSub(sc.point, centerB) : sc.point;

        // Warm-start: 前フレームの累積インパルスを照合して復元
        sc.normalLambda    = 0.0f;
        sc.frictionLambda1 = 0.0f;
        sc.frictionLambda2 = 0.0f;
        auto range = prevMap.equal_range(PrevKey{sc.colA, sc.colB});
        for (auto it = range.first; it != range.second; ++it) {
            const auto& prev = _prevSolverContacts[it->second];
            if (LenSq(VSub(prev.localA, sc.localA)) > kContactMatchDistSq) continue;
            if (LenSq(VSub(prev.localB, sc.localB)) > kContactMatchDistSq) continue;
            sc.normalLambda    = prev.normalLambda;
            sc.frictionLambda1 = prev.frictionLambda1;
            sc.frictionLambda2 = prev.frictionLambda2;
            break;
        }

        results[idx] = { sc, true };
    }, 64);

    _solverContacts.reserve(rawCount);
    for (auto& r : results) {
        if (r.valid) _solverContacts.push_back(std::move(r.sc));
    }
}

void PhysicsManager::WarmStart() {
    for (auto& sc : _solverContacts) {
        if (sc.effectiveInvMassN <= 1e-8f) continue;

        const bool aSleeping = !sc.bodyA || sc.bodyA->_isSleeping;
        const bool bSleeping = !sc.bodyB || sc.bodyB->_isSleeping;
        if (aSleeping && bSleeping) continue;

        const VECTOR warmN  = VScale(sc.normal,   sc.normalLambda    * kWarmStartFactor);
        const VECTOR warmT1 = VScale(sc.tangent1, sc.frictionLambda1 * kWarmStartFactor);
        const VECTOR warmT2 = VScale(sc.tangent2, sc.frictionLambda2 * kWarmStartFactor);

        ApplyImpulse(sc.bodyA, sc.bodyB, sc.invA, sc.invB, sc.rA, sc.rB,
            VAdd(VAdd(warmN, warmT1), warmT2));
    }
}
