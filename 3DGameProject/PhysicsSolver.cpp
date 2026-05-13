#include "PhysicsManager_Internal.h"

// ---- SolveIsland ----------------------------------------------------

void PhysicsManager::SolveIsland(const PhysicsIsland& island, float /*stepDt*/) {
    for (int ci : island.contactIndices) {
        if (ci < 0 || static_cast<size_t>(ci) >= _solverContacts.size()) {
            ASSERT_MSG(false, "SolveIsland: contact index out of range. ci=%d size=%zu", ci, _solverContacts.size());
            continue;
        }
        SolverContact& sc = _solverContacts[ci];
        if (sc.effectiveInvMassN <= 1e-8f) continue;
        const bool aStill = !sc.bodyA || sc.bodyA->_isSleeping;
        const bool bStill = !sc.bodyB || sc.bodyB->_isSleeping;
        if (aStill && bStill) continue;

        // 法線拘束（累積クランプ付き）
        {
            const float vn = RelNormalVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.normal);
            float bias = 0.0f;
            if (!_splitImpulseEnabled) {
                bias = sc.normalBias;
            } else if (vn < -kRestitutionThreshold) {
                bias = sc.restitution * (-vn);
            }
            if (sc.speculative) {
                bias = (std::min)(sc.normalBias, (std::max)(-vn, 0.0f));
            }
            float dl = (-vn + bias) / sc.effectiveInvMassN;
            const float oldL = sc.normalLambda;
            sc.normalLambda = (std::max)(oldL + dl, 0.0f);
            dl = sc.normalLambda - oldL;
            ApplyImpulse(sc.bodyA, sc.bodyB, sc.invA, sc.invB, sc.rA, sc.rB, VScale(sc.normal, dl));
        }

        // 摩擦（円形クーロンコーン）
        {
            const float vt1 = RelDirVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.tangent1);
            const float vt2 = RelDirVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.tangent2);
            const float tSpd = std::sqrt(vt1*vt1 + vt2*vt2);
            const float fri = (tSpd < 0.1f) ? sc.staticFriction : sc.friction;
            const float maxF = fri * sc.normalLambda;
            float d1 = -vt1 / sc.effectiveInvMassT1;
            float d2 = -vt2 / sc.effectiveInvMassT2;
            float n1 = sc.frictionLambda1 + d1;
            float n2 = sc.frictionLambda2 + d2;
            const float fMag = std::sqrt(n1*n1 + n2*n2);
            if (fMag > maxF && fMag > 1e-8f) { const float s = maxF / fMag; n1 *= s; n2 *= s; }
            d1 = n1 - sc.frictionLambda1; d2 = n2 - sc.frictionLambda2;
            sc.frictionLambda1 = n1; sc.frictionLambda2 = n2;
            ApplyImpulse(sc.bodyA, sc.bodyB, sc.invA, sc.invB, sc.rA, sc.rB,
                VAdd(VScale(sc.tangent1, d1), VScale(sc.tangent2, d2)));
        }
    }
}

// ---- SolveAllIslands ------------------------------------------------

void PhysicsManager::SolveAllIslands(float stepDt) {
    if (_islands.empty()) return;

    const int    iterations  = _solverIterations;
    const size_t islandCount = _islands.size();

    // 接触数降順でソート（重いアイランドを先に処理してテール待機を削減）
    // キャッシュ: 前回と同じサイズなら再ソートスキップ
    auto& islandOrder = _islandOrderBuf;
    static size_t prevIslandCount = 0;
    const bool needsSort = (islandCount != prevIslandCount);
    prevIslandCount = islandCount;

    if (needsSort || islandOrder.size() != islandCount) {
        islandOrder.resize(islandCount);
        for (size_t i = 0; i < islandCount; ++i) islandOrder[i] = i;
        std::sort(islandOrder.begin(), islandOrder.end(), [&](size_t a, size_t b) {
            return _islands[a].contactIndices.size() > _islands[b].contactIndices.size();
        });
    }

    // パス1: 小アイランド（constraintBatches なし）→ アイランド単位で並列
    ThreadPool::Instance().ParallelForBarrierHeavy(0, islandCount, [&](size_t orderIdx) {
        const size_t i = islandOrder[orderIdx];
        if (i >= _islands.size()) {
            ASSERT_MSG(false, "SolveAllIslands: index out of range. i=%zu size=%zu", i, _islands.size());
            return;
        }
        if (_islands[i].allSleeping) return;
        if (_islands[i].contactIndices.empty()) return;
        if (!_islands[i].constraintBatches.empty()) return;
        for (int iter = 0; iter < iterations; ++iter) SolveIsland(_islands[i], stepDt);
    }, 1);

    // パス2: 大アイランド（constraintBatches あり）→ バッチ単位で並列
    // kBatchParallelThreshold 未満の小バッチはシリアル実行してバリアオーバーヘッドを排除
    auto solveOneContact = [&](size_t batchIdx, const std::vector<int>& batchRef) {
        const int ci = batchRef[batchIdx];
        if (ci < 0 || static_cast<size_t>(ci) >= _solverContacts.size()) return;
        SolverContact& sc = _solverContacts[ci];
        if (sc.effectiveInvMassN <= 1e-8f) return;
        if ((!sc.bodyA || sc.bodyA->_isSleeping) && (!sc.bodyB || sc.bodyB->_isSleeping)) return;

        {
            const float vn = RelNormalVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.normal);
            float bias = 0.0f;
            if (!_splitImpulseEnabled) bias = sc.normalBias;
            else if (vn < -kRestitutionThreshold) bias = sc.restitution * (-vn);
            if (sc.speculative) bias = (std::min)(sc.normalBias, (std::max)(-vn, 0.0f));
            float dl = (-vn + bias) / sc.effectiveInvMassN;
            const float old = sc.normalLambda;
            sc.normalLambda = (std::max)(old + dl, 0.0f);
            dl = sc.normalLambda - old;
            ApplyImpulse(sc.bodyA, sc.bodyB, sc.invA, sc.invB, sc.rA, sc.rB, VScale(sc.normal, dl));
        }
        {
            const float vt1 = RelDirVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.tangent1);
            const float vt2 = RelDirVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.tangent2);
            const float tSpd = std::sqrt(vt1*vt1 + vt2*vt2);
            const float fri  = (tSpd < 0.1f) ? sc.staticFriction : sc.friction;
            const float maxF = fri * sc.normalLambda;
            float d1 = -vt1 / sc.effectiveInvMassT1, d2 = -vt2 / sc.effectiveInvMassT2;
            float n1 = sc.frictionLambda1 + d1, n2 = sc.frictionLambda2 + d2;
            const float fMag = std::sqrt(n1*n1 + n2*n2);
            if (fMag > maxF && fMag > 1e-8f) { const float s = maxF/fMag; n1 *= s; n2 *= s; }
            d1 = n1 - sc.frictionLambda1; d2 = n2 - sc.frictionLambda2;
            sc.frictionLambda1 = n1; sc.frictionLambda2 = n2;
            ApplyImpulse(sc.bodyA, sc.bodyB, sc.invA, sc.invB, sc.rA, sc.rB,
                VAdd(VScale(sc.tangent1, d1), VScale(sc.tangent2, d2)));
        }
    };

    for (size_t orderIdx = 0; orderIdx < islandCount; ++orderIdx) {
        const size_t i = islandOrder[orderIdx];
        if (i >= _islands.size() || _islands[i].allSleeping || _islands[i].constraintBatches.empty()) continue;
        for (int iter = 0; iter < iterations; ++iter) {
            for (const auto& batch : _islands[i].constraintBatches) {
                if (batch.size() < static_cast<size_t>(kBatchParallelThreshold)) {
                    for (size_t j = 0; j < batch.size(); ++j) solveOneContact(j, batch);
                } else {
                    ThreadPool::Instance().ParallelForBarrierHeavy(0, batch.size(), [&](size_t bi) {
                        solveOneContact(bi, batch);
                    }, 1);
                }
            }
        }
    }

    const size_t bodyCount = _bodies.size();
    ThreadPool::Instance().ParallelForBarrier(0, bodyCount, [&](size_t idx) {
        PhysicsBody* body = _bodies[idx];
        if (!body) return;
        SanitizeVec(body->_velocity);
        SanitizeVec(body->_angularVelocity);
    }, 64);
}

// ---- PositionalCorrection -------------------------------------------

void PhysicsManager::PositionalCorrection(float /*stepDt*/) {
    for (const auto& sc : _solverContacts) {
        if (sc.penetration <= kSlop) continue;
        const float invSum = sc.invA + sc.invB;
        if (invSum <= 1e-8f) continue;

        const float clampedPen = (std::min)(sc.penetration, kMaxPen);
        float mag = kBiasFactor * (std::max)(clampedPen - kSlop, 0.0f) / invSum;
        mag = (std::min)(mag, kMaxCorrection);
        if (mag <= 1e-6f) continue;

        GameObject* ownerA = sc.colA ? sc.colA->owner : nullptr;
        GameObject* ownerB = sc.colB ? sc.colB->owner : nullptr;

        if (sc.bodyA && sc.invA > 0.0f && ownerA) {
            VECTOR p = VSub(ownerA->transform.LocalPosition(), VScale(sc.normal, mag * sc.invA));
            if (IsFiniteVec(p)) {
                if (_groundPlaneEnabled && HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD) < 0.0f)
                    p = VSub(p, VScale(_groundPlaneNormal, HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD)));
                ownerA->transform.SetLocalPosition(p);
            }
        }
        if (sc.bodyB && sc.invB > 0.0f && ownerB) {
            VECTOR p = VAdd(ownerB->transform.LocalPosition(), VScale(sc.normal, mag * sc.invB));
            if (IsFiniteVec(p)) {
                if (_groundPlaneEnabled && HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD) < 0.0f)
                    p = VSub(p, VScale(_groundPlaneNormal, HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD)));
                ownerB->transform.SetLocalPosition(p);
            }
        }
    }
}

// ---- UpdateSleepState -----------------------------------------------

void PhysicsManager::UpdateSleepState(PhysicsBody* body, float stepDt) {
    if (!body || !body->IsDynamic()) return;
    if (body->_hasMovePositionTarget || body->_hasMoveRotationTarget) { body->WakeUp(); return; }
    if (LenSq(body->_force) > 1e-8f || LenSq(body->_torque) > 1e-8f)  { body->WakeUp(); return; }

    const float lSq = body->_sleepLinearThreshold  * body->_sleepLinearThreshold;
    const float aSq = body->_sleepAngularThreshold * body->_sleepAngularThreshold;
    if (LenSq(body->_velocity) > lSq || LenSq(body->_angularVelocity) > aSq) { body->WakeUp(); return; }

    body->_sleepTimer += stepDt;
    if (body->_sleepTimer >= body->_sleepTimeThreshold) body->Sleep();
}

// ---- SplitImpulseCorrection -----------------------------------------

void PhysicsManager::SplitImpulseCorrection(float /*stepDt*/) {
    const size_t bodyCount = _bodies.size();

    auto& pseudoVel = _pseudoVelBuf;
    pseudoVel.assign(bodyCount, VGet(0, 0, 0));

    auto& bodyIdx = _bodyIdxBuf;
    bodyIdx.clear();
    if (bodyIdx.bucket_count() < bodyCount) bodyIdx.reserve(bodyCount);
    for (size_t i = 0; i < bodyCount; ++i) {
        if (_bodies[i]) bodyIdx[_bodies[i]] = i;
    }

    const int    posIter    = (std::max)(_solverIterations / 2, 2);
    const size_t islandCount = _islands.size();
    for (int iter = 0; iter < posIter; ++iter) {
        ThreadPool::Instance().ParallelForBarrierHeavy(0, islandCount, [&](size_t islandIdx) {
            if (_islands[islandIdx].allSleeping) return;
            for (int ci : _islands[islandIdx].contactIndices) {
                auto& sc = _solverContacts[ci];
                if (sc.penetration <= kSlop || sc.effectiveInvMassN <= 1e-8f) continue;

                const size_t idxA = sc.bodyA ? (bodyIdx.count(sc.bodyA) ? bodyIdx.at(sc.bodyA) : SIZE_MAX) : SIZE_MAX;
                const size_t idxB = sc.bodyB ? (bodyIdx.count(sc.bodyB) ? bodyIdx.at(sc.bodyB) : SIZE_MAX) : SIZE_MAX;
                const VECTOR pvA  = (idxA < bodyCount) ? pseudoVel[idxA] : VGet(0,0,0);
                const VECTOR pvB  = (idxB < bodyCount) ? pseudoVel[idxB] : VGet(0,0,0);
                const float  pvn  = Dot3(VSub(pvB, pvA), sc.normal);

                float dl = (-pvn + sc.splitBias) / sc.effectiveInvMassN;
                const float oldL = sc.splitNormalLambda;
                sc.splitNormalLambda = (std::max)(oldL + dl, 0.0f);
                dl = sc.splitNormalLambda - oldL;
                if (std::fabs(dl) < 1e-8f) return;

                const VECTOR impulse = VScale(sc.normal, dl);
                if (idxA < bodyCount && sc.invA > 0.0f)
                    pseudoVel[idxA] = VSub(pseudoVel[idxA], VScale(impulse, sc.invA));
                if (idxB < bodyCount && sc.invB > 0.0f)
                    pseudoVel[idxB] = VAdd(pseudoVel[idxB], VScale(impulse, sc.invB));
            }
        }, 1);
    }

    ThreadPool::Instance().ParallelForBarrier(0, bodyCount, [&](size_t idx) {
        if (LenSq(pseudoVel[idx]) < 1e-10f) return;
        PhysicsBody* body = _bodies[idx];
        if (!body || !body->_owner) return;
        VECTOR p = body->_owner->transform.LocalPosition();
        VECTOR correction = VScale(pseudoVel[idx], _fixedDeltaTime);
        ClampMagnitude(correction, kMaxCorrection);
        p = VAdd(p, correction);
        if (IsFiniteVec(p)) {
            if (_groundPlaneEnabled && HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD) < 0.0f)
                p = VSub(p, VScale(_groundPlaneNormal, HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD)));
            body->_owner->transform.SetLocalPosition(p);
        }
    }, 64);
}

// ---- PropagateIslandSleep -------------------------------------------

void PhysicsManager::PropagateIslandSleep() {
    const size_t islandCount = _islands.size();
    if (islandCount > 0) {
        auto& islands = _islands;  // ローカル参照を作成
        ThreadPool::Instance().ParallelForBarrierHeavy(0, islandCount, [&islands](size_t i) {
            auto& island = islands[i];
            bool anyAwake = false;
            for (auto* body : island.bodies) {
                if (body && body->IsDynamic() && !body->_isSleeping) { anyAwake = true; break; }
            }
            if (anyAwake) {
                for (auto* body : island.bodies) {
                    if (body && body->IsDynamic() && body->_isSleeping) body->WakeUp();
                }
            }
        }, 1);
    }

    const size_t bodyCount = _bodies.size();
    if (bodyCount > 0) {
        const float stepDt = _fixedDeltaTime;
        auto& bodies = _bodies;  // ローカル参照を作成
        ThreadPool::Instance().ParallelForBarrier(0, bodyCount, [this, &bodies, stepDt](size_t idx) {
            PhysicsBody* body = bodies[idx];
            if (!body) return;
            if (body->_isSleeping) {
                if (LenSq(body->_velocity) > 1e-8f || LenSq(body->_angularVelocity) > 1e-8f)
                    body->WakeUp();
            }
            ApplyBodyConstraints(body);
            UpdateSleepState(body, stepDt);
        }, 64);
    }
}

// ---- 補間 -----------------------------------------------------------

float PhysicsManager::InterpolationAlpha() const noexcept {
    if (_fixedDeltaTime <= 1e-6f) return 1.0f;
    return (std::min)(_accumulator / _fixedDeltaTime, 1.0f);
}

void PhysicsManager::ComputeInterpolation() noexcept {
    const float  alpha     = InterpolationAlpha();
    const size_t bodyCount = _bodies.size();
    if (bodyCount == 0) return;

    ThreadPool::Instance().ParallelForBarrier(0, bodyCount, [&](size_t idx) {
        PhysicsBody* body = _bodies[idx];
        if (!body || !body->_owner) return;
        if (!body->_useInterpolation) {
            body->_interpPosition = body->_owner->transform.LocalPosition();
            body->_interpRotation = body->_owner->transform.LocalRotation();
            return;
        }
        const VECTOR curPos = body->_owner->transform.LocalPosition();
        body->_interpPosition = VAdd(
            VScale(body->_previousPosition, 1.0f - alpha),
            VScale(curPos, alpha));
        body->_interpRotation = Quaternion::Slerp(
            body->_previousRotation,
            body->_owner->transform.LocalRotation(),
            alpha);
    }, 64);
}

void PhysicsManager::ApplyBodyConstraints(PhysicsBody* body) const {
    if (!body) return;
    if (body->_freezeRotation) body->_angularVelocity = VGet(0,0,0);
    if (body->_isSleeping)     { body->_velocity = VGet(0,0,0); body->_angularVelocity = VGet(0,0,0); }
}
