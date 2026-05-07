#include "PhysicsManager_Internal.h"

// ---- GenerateSpeculativeContacts ------------------------------------

void PhysicsManager::GenerateSpeculativeContacts(float stepDt) {
    if (stepDt <= 1e-6f) return;
    const float invDt = 1.0f / stepDt;

    // 既存の非投機的接触ペアを O(1) で確認するためのハッシュセット
    struct ColPairKey {
        Collider* a; Collider* b;
        bool operator==(const ColPairKey& o) const noexcept {
            return (a == o.a && b == o.b) || (a == o.b && b == o.a);
        }
    };
    struct ColPairHash {
        size_t operator()(const ColPairKey& k) const noexcept {
            auto pa = reinterpret_cast<size_t>(k.a);
            auto pb = reinterpret_cast<size_t>(k.b);
            if (pa > pb) std::swap(pa, pb);
            return (pa >> 4) ^ (pb << 1);
        }
    };
    std::unordered_set<ColPairKey, ColPairHash> existingPairs;
    existingPairs.reserve(_solverContacts.size());
    for (const auto& sc : _solverContacts) {
        if (!sc.speculative && sc.colA && sc.colB)
            existingPairs.insert({sc.colA, sc.colB});
    }

    for (auto* body : _bodies) {
        if (!body || !body->IsDynamic() || !body->_detectContinuous) continue;
        if (!body->_owner || !body->_owner->IsActive()) continue;

        const float speed = Len3(body->_velocity);
        if (speed < 1.0f) continue;

        Collider* col = CachedFindCollider(body->_owner);
        if (!col) continue;

        const float minHE = GetColliderMinHalfExtent(col);
        if (speed * stepDt < minHE) continue;

        const VECTOR curPos       = body->_owner->transform.WorldPosition();
        const VECTOR predictedPos = VAdd(curPos, VScale(body->_velocity, stepDt));

        AABB predictedAABB;
        {
            const AABB& cur    = col->GetAABB();
            const VECTOR offset = VScale(body->_velocity, stepDt);
            predictedAABB.min.x = (std::min)(cur.min.x, cur.min.x + offset.x) - kSpeculativeMargin;
            predictedAABB.min.y = (std::min)(cur.min.y, cur.min.y + offset.y) - kSpeculativeMargin;
            predictedAABB.min.z = (std::min)(cur.min.z, cur.min.z + offset.z) - kSpeculativeMargin;
            predictedAABB.max.x = (std::max)(cur.max.x, cur.max.x + offset.x) + kSpeculativeMargin;
            predictedAABB.max.y = (std::max)(cur.max.y, cur.max.y + offset.y) + kSpeculativeMargin;
            predictedAABB.max.z = (std::max)(cur.max.z, cur.max.z + offset.z) + kSpeculativeMargin;
            predictedAABB.center = VScale(VAdd(predictedAABB.min, predictedAABB.max), 0.5f);
        }

        // 地面平面との投機的接触
        if (_groundPlaneEnabled) {
            const float dist = HalfPlaneDistance(predictedPos, _groundPlaneNormal, _groundPlaneD);
            if (dist < minHE + kSpeculativeMargin) {
                SolverContact sc{};
                sc.colA = col; sc.colB = nullptr;
                sc.normal     = _groundPlaneNormal;
                sc.point      = VSub(predictedPos, VScale(_groundPlaneNormal, dist));
                sc.penetration = minHE + kSpeculativeMargin - dist;
                sc.bodyA = body; sc.bodyB = nullptr;
                sc.invA  = body->InverseMass(); sc.invB = 0.0f;
                sc.rA    = VSub(sc.point, curPos); sc.rB = VGet(0,0,0);
                ComputeTangentBasis(sc.normal, sc.tangent1, sc.tangent2);
                sc.effectiveInvMassN  = ComputeEffectiveInvMass(sc.invA, 0.0f, body, nullptr, sc.rA, sc.rB, sc.normal);
                sc.effectiveInvMassT1 = ComputeEffectiveInvMass(sc.invA, 0.0f, body, nullptr, sc.rA, sc.rB, sc.tangent1);
                sc.effectiveInvMassT2 = ComputeEffectiveInvMass(sc.invA, 0.0f, body, nullptr, sc.rA, sc.rB, sc.tangent2);
                if (sc.effectiveInvMassN > 1e-8f) {
                    if (sc.effectiveInvMassT1 <= 1e-8f) sc.effectiveInvMassT1 = 1e-8f;
                    if (sc.effectiveInvMassT2 <= 1e-8f) sc.effectiveInvMassT2 = 1e-8f;
                    sc.friction = body->_friction; sc.staticFriction = sc.friction * 1.2f;
                    sc.normalBias = kSplitBiasFactor * invDt * (std::max)(sc.penetration - kSlop, 0.0f);
                    sc.splitBias  = sc.normalBias;
                    sc.localA = sc.rA; sc.speculative = true;
                    _solverContacts.push_back(sc);
                }
            }
        }

        // 他コライダーとの投機的接触（一時バッファに収集してから一括追加）
        auto& specContacts = _specContactsBuf;
        specContacts.clear();

        for (auto* otherCol : ColliderManager::Instance().GetColliders()) {
            if (!otherCol || otherCol == col || otherCol->owner == body->_owner) continue;
            if (existingPairs.count({col, otherCol})) continue;

            const AABB& otherAABB = otherCol->GetAABB();
            if (predictedAABB.min.x > otherAABB.max.x || predictedAABB.max.x < otherAABB.min.x) continue;
            if (predictedAABB.min.y > otherAABB.max.y || predictedAABB.max.y < otherAABB.min.y) continue;
            if (predictedAABB.min.z > otherAABB.max.z || predictedAABB.max.z < otherAABB.min.z) continue;

            const VECTOR otherCenter  = otherCol->GetCenter();
            const float  otherMinHE   = GetColliderMinHalfExtent(otherCol);
            const float  combinedExt  = minHE + otherMinHE + kSpeculativeMargin;
            if (Len3(VSub(predictedPos, otherCenter)) > combinedExt * 3.0f) continue;

            const VECTOR toOther     = VSub(otherCenter, curPos);
            const float  distNow     = Len3(toOther);
            const float  closingSpd  = -Dot3(body->_velocity, SafeNormalize(toOther)) * stepDt;
            const float  specPen     = combinedExt - (distNow - closingSpd);
            if (specPen <= 0.0f) continue;

            const VECTOR normal     = SafeNormalize(VSub(curPos, otherCenter), VGet(0,1,0));
            const VECTOR contactPt  = VAdd(otherCenter, VScale(normal, otherMinHE));
            PhysicsBody* otherBody  = CachedFindBody(otherCol->owner);
            const float  otherInv   = (otherBody && otherBody->IsDynamic() && otherCol->owner && otherCol->owner->IsActive())
                                        ? otherBody->InverseMass() : 0.0f;
            if (body->InverseMass() + otherInv <= 1e-8f) continue;

            SolverContact sc{};
            sc.colA = col; sc.colB = otherCol;
            sc.normal     = normal; sc.point = contactPt;
            sc.penetration = (std::min)(specPen, kMaxPen);
            sc.bodyA = body; sc.bodyB = otherBody;
            sc.invA  = body->InverseMass(); sc.invB = otherInv;
            sc.rA    = VSub(sc.point, curPos); sc.rB = VSub(sc.point, otherCenter);
            ComputeTangentBasis(sc.normal, sc.tangent1, sc.tangent2);
            sc.effectiveInvMassN  = ComputeEffectiveInvMass(sc.invA, sc.invB, body, otherBody, sc.rA, sc.rB, sc.normal);
            sc.effectiveInvMassT1 = ComputeEffectiveInvMass(sc.invA, sc.invB, body, otherBody, sc.rA, sc.rB, sc.tangent1);
            sc.effectiveInvMassT2 = ComputeEffectiveInvMass(sc.invA, sc.invB, body, otherBody, sc.rA, sc.rB, sc.tangent2);
            if (sc.effectiveInvMassN <= 1e-8f) continue;
            if (sc.effectiveInvMassT1 <= 1e-8f) sc.effectiveInvMassT1 = 1e-8f;
            if (sc.effectiveInvMassT2 <= 1e-8f) sc.effectiveInvMassT2 = 1e-8f;
            sc.friction = (body && otherBody)
                ? PhysicsMaterial::CombineFriction(body->_material, otherBody->_material)
                : body->_friction;
            sc.staticFriction = sc.friction * 1.2f;
            sc.normalBias = kSplitBiasFactor * invDt * (std::max)(sc.penetration - kSlop, 0.0f);
            sc.splitBias  = sc.normalBias;
            sc.localA = sc.rA; sc.localB = sc.rB; sc.speculative = true;

            // 溶接: 同ペアで近すぎる既存投機的接触はスキップ
            constexpr float kWeldDistSq = 0.01f;
            bool welded = false;
            for (const auto& ex : _solverContacts) {
                if (!ex.speculative || (ex.bodyA != body && ex.bodyB != body)) continue;
                if (LenSq(VSub(ex.point, sc.point)) < kWeldDistSq && Dot3(ex.normal, sc.normal) > 0.9f)
                    { welded = true; break; }
            }
            if (!welded) {
                for (const auto& ex : specContacts) {
                    if (ex.bodyA != body && ex.bodyB != body) continue;
                    if (LenSq(VSub(ex.point, sc.point)) < kWeldDistSq && Dot3(ex.normal, sc.normal) > 0.9f)
                        { welded = true; break; }
                }
            }
            if (!welded) specContacts.push_back(sc);
        }

        for (auto& sc : specContacts) _solverContacts.push_back(std::move(sc));
    }
}

// ---- ResolveToiEvents -----------------------------------------------

void PhysicsManager::ResolveToiEvents(float stepDt) {
    if (stepDt <= 1e-6f) return;

    auto& events = _toiEventsBuf;
    events.clear();

    // 投機的接触でカバー済みのボディはバックステップ不要
    auto& specCovered = _specCoveredBodiesBuf;
    specCovered.clear();
    if (_speculativeCcdEnabled) {
        for (const auto& sc : _solverContacts) {
            if (!sc.speculative) continue;
            if (sc.bodyA) specCovered.insert(sc.bodyA);
            if (sc.bodyB) specCovered.insert(sc.bodyB);
        }
    }

    for (auto* body : _bodies) {
        if (!body || !body->IsDynamic() || !body->_detectContinuous) continue;
        if (body->_ccdQuality < CcdQuality::Bullet) continue;
        if (!body->_owner || !body->_owner->IsActive() || body->_isSleeping) continue;
        if (specCovered.count(body)) continue;

        const float speed = Len3(body->_velocity);
        if (speed < 1e-4f) continue;

        Collider* col = CachedFindCollider(body->_owner);
        if (!col) continue;

        const float minHE      = GetColliderMinHalfExtent(col);
        const float allowedPen = (body->_allowedPenetrationDepth > 0.0f)
                                    ? body->_allowedPenetrationDepth : (minHE * 0.8f);
        if (speed * stepDt <= allowedPen) continue;

        const float toi = allowedPen / (speed * stepDt);
        if (toi >= 1.0f) continue;

        ToiEvent ev;
        ev.body           = body;
        ev.toi            = (std::max)(0.0f, toi);
        ev.toiPosition    = VAdd(body->_previousPosition, VScale(body->_velocity, ev.toi * stepDt));
        ev.clampedVelocity = body->_velocity;
        events.push_back(ev);
    }

    if (events.empty()) return;

    std::sort(events.begin(), events.end(), [](const ToiEvent& a, const ToiEvent& b) {
        return a.toi < b.toi;
    });

    for (auto& ev : events) {
        if (!ev.body || !ev.body->_owner) continue;

        ev.body->_owner->transform.SetLocalPosition(ev.toiPosition);

        const float remainDt = (1.0f - ev.toi) * stepDt;
        if (remainDt > 1e-6f) {
            Collider* col = CachedFindCollider(ev.body->_owner);
            const float allowedPen = col
                ? ((ev.body->_allowedPenetrationDepth > 0.0f)
                    ? ev.body->_allowedPenetrationDepth
                    : (GetColliderMinHalfExtent(col) * 0.8f))
                : 0.4f;
            const float remainSpeed = Len3(ev.body->_velocity);
            if (remainSpeed * remainDt > allowedPen)
                ClampMagnitude(ev.body->_velocity, allowedPen / remainDt);

            VECTOR pos = VAdd(ev.body->_owner->transform.LocalPosition(),
                              VScale(ev.body->_velocity, remainDt));

            if (_groundPlaneEnabled) {
                const float dist = HalfPlaneDistance(pos, _groundPlaneNormal, _groundPlaneD);
                if (dist < 0.0f) {
                    pos = VSub(pos, VScale(_groundPlaneNormal, dist));
                    const float vn = Dot3(ev.body->_velocity, _groundPlaneNormal);
                    if (vn < 0.0f)
                        ev.body->_velocity = VSub(ev.body->_velocity,
                            VScale(_groundPlaneNormal, vn * (1.0f + ev.body->_restitution)));
                }
            }
            ev.body->_owner->transform.SetLocalPosition(pos);
        }

        Collider* col = CachedFindCollider(ev.body->_owner);
        if (col) col->UpdateShape();
    }

    ColliderManager::Instance().Update(stepDt);
    BuildLookupCaches();
}
