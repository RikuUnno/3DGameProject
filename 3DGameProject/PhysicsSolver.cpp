#include "PhysicsManager_Internal.h"

// SolveIsland - アイランド内の接触制約を解く
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

        // 法線方向の拘束解法（インパルス累積クランプ付き）
        {
            const float vn = RelNormalVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.normal);
            float bias = 0.0f;
            if (!_splitImpulseEnabled) {
                bias = sc.normalBias;
            } else if (vn < -kRestitutionThreshold) {
                bias = sc.restitution * (-vn);
            }
            if (sc.speculative) {
                const float speculativeBias = (std::min)(sc.normalBias, (std::max)(-vn, 0.0f));
                if (vn < -kRestitutionThreshold) {
                    bias = (std::max)(speculativeBias, sc.restitution * (-vn));
                } else {
                    bias = speculativeBias;
                }
            }
            float dl = (-vn + bias) / sc.effectiveInvMassN;
            const float oldL = sc.normalLambda;
            sc.normalLambda = (std::max)(oldL + dl, 0.0f);
            dl = sc.normalLambda - oldL;
            ApplyImpulse(sc.bodyA, sc.bodyB, sc.invA, sc.invB, sc.rA, sc.rB, VScale(sc.normal, dl));
        }

        // 摩擦力の拘束解法（円形クーロンコーン摩擦モデル）
        {
            const float vt1 = RelDirVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.tangent1);
            const float vt2 = RelDirVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.tangent2);
            const float tSpd = std::sqrt(vt1*vt1 + vt2*vt2);
            const float fri = (tSpd < kFrictionStaticThreshold) ? sc.staticFriction : sc.friction;
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

// SolveAllIslands - すべてのアイランドの制約を反復的に解く
//
// 処理の流れ:
//   (a) アイランドを接触数順にソートし、重いものからスレッドに乗せる。
//   (b) パス1: constraintBatches を持たない小規模アイランドを PGS で反復解。
//   (c) パス2: constraintBatches を持つ大規模アイランドをグラフ着色単位で並列解。
//   (d) 仕上げに NaN/Inf を除去する sanitize パス。
void PhysicsManager::SolveAllIslands(float stepDt) {
    if (IsShuttingDown()) return;
    if (_islands.empty()) return;

    const size_t islandCount = _islands.size();

    // (a) アイランドを接触数順にソート。アイランド数が変わらないフレームは
    //     前回の順序を使い回してソートコストを削減。
    auto& islandOrder = _islandOrderBuf;
    const bool needsSort = (islandCount != _prevIslandCount);
    _prevIslandCount = islandCount;

    if (needsSort || islandOrder.size() != islandCount) {
        islandOrder.resize(islandCount);
        for (size_t i = 0; i < islandCount; ++i) islandOrder[i] = i;
        std::sort(islandOrder.begin(), islandOrder.end(), [&](size_t a, size_t b) {
            if (a >= _islands.size() || b >= _islands.size()) return false;
            return _islands[a].contactIndices.size() > _islands[b].contactIndices.size();
        });
    }

    // パス1: 制約バッチなし (小規模) アイランド → アイランド単位で並列実行
    ThreadPool::Instance().ParallelForBarrierHeavy(0, islandCount, [this, &islandOrder, islandCount, stepDt](size_t orderIdx) {
        if (IsShuttingDown()) return;
        if (orderIdx >= islandCount) return;
        if (orderIdx >= islandOrder.size()) return;
        const size_t i = islandOrder[orderIdx];
        if (i >= _islands.size()) {
            ASSERT_MSG(false, "SolveAllIslands: index out of range. i=%zu size=%zu", i, _islands.size());
            return;
        }
        const auto& island = _islands[i];
        if (island.allSleeping) return;
        if (island.contactIndices.empty()) return;
        if (!island.constraintBatches.empty()) return;

        // イテレーション数は ComputeAdaptiveIterations() で既に
        // contactCount / bodies を考慮した値が _solverIterations に入っているため
        // ここで重ねて加算しない（以前は二重加算で 1.25x 余分に動いていた）。
        const int iterations = _solverIterations;

        for (int iter = 0; iter < iterations; ++iter) {
            if (IsShuttingDown()) return;
            SolveIsland(island, stepDt);
        }
    }, 1);

    // パス2: 制約バッチあり (大規模) アイランド → バッチ単位で並列実行。
    // 同一バッチ内はボディを共有しないためロック不要でインパルスを適用できる。
    auto solveOneContact = [&](size_t batchIdx, const std::vector<int>& batchRef) {
        if (IsShuttingDown()) return;
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
            if (sc.speculative) {
                const float speculativeBias = (std::min)(sc.normalBias, (std::max)(-vn, 0.0f));
                if (vn < -kRestitutionThreshold) bias = (std::max)(speculativeBias, sc.restitution * (-vn));
                else bias = speculativeBias;
            }
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
            const float fri  = (tSpd < kFrictionStaticThreshold) ? sc.staticFriction : sc.friction;
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

    // アイランド間はボディを共有しないため並列化可能。
    // アイランド内の batch 順は同一ボディを重複して更新するため順序を保つ。
    ThreadPool::Instance().ParallelForBarrierHeavy(0, islandCount, [this, &islandOrder, islandCount, stepDt, &solveOneContact](size_t orderIdx) {
        if (IsShuttingDown()) return;
        if (orderIdx >= islandCount) return;
        if (orderIdx >= islandOrder.size()) return;
        const size_t i = islandOrder[orderIdx];
        if (i >= _islands.size()) return;
        const auto& island = _islands[i];
        if (island.allSleeping || island.constraintBatches.empty()) return;

        const int iterations = _solverIterations;

        for (int iter = 0; iter < iterations; ++iter) {
            if (IsShuttingDown()) return;
            for (const auto& batch : island.constraintBatches) {
                // 不要な ParallelFor バリアは走らず、ここでは常にシリアル処理する。
                // （アイランド単位で既にワーカーに負荷を振っているため、
                //   さらにネストしても ReentrantSerial に落ちるので意味がない）
                for (size_t j = 0; j < batch.size(); ++j) solveOneContact(j, batch);
            }
        }
    }, 1);

    // (d) 仕上げ: ソルバ反復で生じた NaN/Inf を除去して次ステップへ渡す。
    const size_t bodyCount = _bodies.size();
    ThreadPool::Instance().ParallelForBarrier(0, bodyCount, [&](size_t idx) {
        PhysicsBody* body = _bodies[idx];
        if (!body) return;
        SanitizeVec(body->_velocity);
        SanitizeVec(body->_angularVelocity);
    }, 16);
}

// PositionalCorrection - ペネトレーション修正による位置補正
//
// 接触ごとに transform を直接書き換えると、同一ボディが N 個の接触に関わるとき
// (床 + 壁 + 他球、Box-Box SAT の 4 点接触など) 補正量が N 倍に肥大化し、
// スタックが打ち上げられたり空中に張り付いたりする。そこで:
//   1. いったんボディごとに補正ベクトルを累積。
//   2. ループ後に kMaxCorrection でクランプして一括適用。
// SplitImpulseCorrection と同じ「累積→適用」構造となり動作が整合する。
void PhysicsManager::PositionalCorrection(float /*stepDt*/, float depthThreshold, float biasScale) {
    if (_solverContacts.empty()) return;

    const size_t bodyCount = _bodies.size();
    if (bodyCount == 0) return;

    auto& bodyIdx = _bodyIdxBuf;
    bodyIdx.clear();
    if (bodyIdx.bucket_count() < bodyCount) bodyIdx.reserve(bodyCount);
    for (size_t i = 0; i < bodyCount; ++i) {
        if (_bodies[i]) bodyIdx[_bodies[i]] = i;
    }

    auto& correction = _posCorrectionBuf;
    correction.assign(bodyCount, VGet(0, 0, 0));

    const float effectiveBias = kBiasFactor * biasScale;
    const float effectiveSlop = (std::max)(depthThreshold, kSlop);

    // パス1: 各接触のめり込み量からボディ単位の補正ベクトルを累積する。
    for (const auto& sc : _solverContacts) {
        if (sc.penetration <= effectiveSlop) continue;
        const float invSum = sc.invA + sc.invB;
        if (invSum <= 1e-8f) continue;

        const float clampedPen = (std::min)(sc.penetration, kMaxPen);
        const float correctionDist = (std::min)(
            effectiveBias * (std::max)(clampedPen - effectiveSlop, 0.0f),
            kMaxCorrection);
        if (correctionDist <= 1e-6f) continue;

        const float shareA = sc.invA / invSum;
        const float shareB = sc.invB / invSum;

        if (sc.bodyA && sc.invA > 0.0f) {
            auto it = bodyIdx.find(sc.bodyA);
            if (it != bodyIdx.end() && it->second < bodyCount) {
                correction[it->second] = VSub(correction[it->second],
                    VScale(sc.normal, correctionDist * shareA));
            }
        }
        if (sc.bodyB && sc.invB > 0.0f) {
            auto it = bodyIdx.find(sc.bodyB);
            if (it != bodyIdx.end() && it->second < bodyCount) {
                correction[it->second] = VAdd(correction[it->second],
                    VScale(sc.normal, correctionDist * shareB));
            }
        }
    }

    // ボディ単位で一括適用。1 ボディあたりの合計補正量を kMaxCorrection で
    // クランプし、多接触時の暴走を抑える。
    for (size_t i = 0; i < bodyCount; ++i) {
        VECTOR c = correction[i];
        if (LenSq(c) < 1e-12f) continue;
        PhysicsBody* body = _bodies[i];
        if (!body || !body->_owner) continue;
        if (body->InverseMass() <= 0.0f) continue;

        ClampMagnitude(c, kMaxCorrection);
        VECTOR p = VAdd(body->_owner->transform.LocalPosition(), c);
        if (!IsFiniteVec(p)) continue;
        if (_groundPlaneEnabled && HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD) < 0.0f)
            p = VSub(p, VScale(_groundPlaneNormal, HalfPlaneDistance(p, _groundPlaneNormal, _groundPlaneD)));
        body->_owner->transform.SetLocalPosition(p);
    }
}

// UpdateSleepState - 剛体のスリープ状態を更新（EMA + ヒステリシス）
void PhysicsManager::UpdateSleepState(PhysicsBody* body, float stepDt) {
    if (!body || !body->IsDynamic()) return;
    if (body->_hasMovePositionTarget || body->_hasMoveRotationTarget) { body->WakeUp(); return; }
    if (LenSq(body->_force) > 1e-8f || LenSq(body->_torque) > 1e-8f)  { body->WakeUp(); return; }

    const float vSq           = LenSq(body->_velocity);                                          // 並進速度の二乗
    const float wSq           = LenSq(body->_angularVelocity);                                   // 角速度の二乗
    const float invM          = body->InverseMass();                                             // 逆質量
    const float invI          = body->AverageInverseInertia();                                   // 平均逆慣性
    const float angWeight     = (invI > 1e-8f && invM > 1e-8f) ? (invM / invI) : 0.0f;           // 角速度→並進相当の重み
    const float currentEnergy = vSq + angWeight * wSq;                                           // 今ステップの運動エネルギー代理量

    const float tau   = 0.1f;                                                                    // EMA 時定数 [s]
    const float alpha = stepDt / (tau + stepDt);                                                 // EMA 係数
    body->_kineticEnergyEMA = (1.0f - alpha) * body->_kineticEnergyEMA + alpha * currentEnergy;  // 平滑化更新

    const float linTh   = body->_sleepLinearThreshold;                                           // 並進スリープ閾値
    const float angTh   = body->_sleepAngularThreshold;                                          // 角速度スリープ閾値
    const float enterTh = linTh * linTh + angWeight * angTh * angTh;                             // sleep へ入る下限
    const float exitTh  = 4.0f * enterTh;                                                        // wake する上限（2倍速相当）
    const float ema     = body->_kineticEnergyEMA;                                               // 平滑化エネルギー

    if (ema > exitTh) { body->_sleepTimer = 0.0f; return; }    // 明確に動作中: timer リセット
    if (ema > enterTh) return;                                  // ヒステリシス帯: timer 維持

    body->_sleepTimer += stepDt;
    if (body->_sleepTimer >= body->_sleepTimeThreshold) body->Sleep();
}

// SplitImpulseCorrection - Split Impulse 手法による位置的修正
//
// 「本體の速度」とは独立に「疑似速度 pseudoVel」を用いてめり込みを押し出す手法。
// 本體速度をバイアスで汚さず、名目としてほぼ転がりによる位置修正だけを行う。
//   1. 複数回反復しながら pseudoVel を PGS で収束させる。
//   2. 最後に pseudoVel * dt を位置に加算。kMaxCorrection で上限を掛ける。
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

    // 位置修正用反復数は速度ソルバの半分程度で十分 (計算コスト押さえ)。
    const int    posIter    = (std::max)(_solverIterations / 2, 2);
    const size_t islandCount = _islands.size();
    // アイランド単位並列で pseudoVel を反復更新する。
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

    // 疑似速度ベクトルから位置を更新
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

// PropagateIslandSleep - 接触経由で隣接ボディへ wake/sleep を伝播
//
// (a) 接触ごとに awake 側の EMA が exit 閾値超なら隣接 sleeper を起こす。
//     ソルバ振動や PositionalCorrection の残りは EMA で平滑化されるため誰も起こさない。
// (b) 全ボディに対して sleep 判定 (UpdateSleepState) を並列で回す。
//     sleeping body の velocity が何らかの理由で動いた場合は wake をフォールバック。
void PhysicsManager::PropagateIslandSleep() {
    // (a) 接触伝搬による wake 伝播。
    const size_t contactCount = _solverContacts.size();
    for (size_t i = 0; i < contactCount; ++i) {
        auto& sc = _solverContacts[i];
        PhysicsBody* a = sc.bodyA;
        PhysicsBody* b = sc.bodyB;
        if (!a || !b) continue;
        const bool aSleep = a->_isSleeping;
        const bool bSleep = b->_isSleeping;
        if (aSleep == bSleep) continue;

        PhysicsBody* awake   = aSleep ? b : a;
        PhysicsBody* sleeper = aSleep ? a : b;
        if (!sleeper->IsDynamic()) continue;

        const float linTh     = awake->_sleepLinearThreshold;                            // 並進スリープ閾値
        const float angTh     = awake->_sleepAngularThreshold;                           // 角速度スリープ閾値
        const float invM      = awake->InverseMass();                                    // 逆質量
        const float invI      = awake->AverageInverseInertia();                          // 平均逆慣性
        const float angWeight = (invI > 1e-8f && invM > 1e-8f) ? (invM / invI) : 0.0f;   // 角速度→並進相当の重み
        const float enterTh   = linTh * linTh + angWeight * angTh * angTh;               // sleep へ入る下限
        const float exitTh    = 4.0f * enterTh;                                          // wake する上限
        if (awake->_kineticEnergyEMA > exitTh) {
            sleeper->WakeUp();
        }
    }

    // (b) 各ボディに sleep 判定を適用。
    const size_t bodyCount = _bodies.size();
    if (bodyCount > 0) {
        const float stepDt = _fixedDeltaTime;
        auto& bodies = _bodies;
        ThreadPool::Instance().ParallelForBarrier(0, bodyCount, [this, &bodies, stepDt](size_t idx) {
            PhysicsBody* body = bodies[idx];
            if (!body) return;
            // sleeping body の velocity が万一動いていたら起こしておくセーフティネット。
            if (body->_isSleeping) {
                const float lSq = body->_sleepLinearThreshold * body->_sleepLinearThreshold;
                const float aSq = body->_sleepAngularThreshold * body->_sleepAngularThreshold;
                if (LenSq(body->_velocity) > lSq || LenSq(body->_angularVelocity) > aSq)
                    body->WakeUp();
            }
            ApplyBodyConstraints(body);
            UpdateSleepState(body, stepDt);
        }, 64);
    }
}

// 補間 - 前フレームから現フレームへの線形補間
float PhysicsManager::InterpolationAlpha() const noexcept {
    if (_fixedDeltaTime <= 1e-6f) return 1.0f;
    return (std::min)(_accumulator / _fixedDeltaTime, 1.0f);
}

// ComputeInterpolation - 各剛体の補間位置と回転を計算
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

// ApplyBodyConstraints - 剛体の拘束条件を適用 (例: 回転の固定、スリープ状態の速度ゼロ化)
void PhysicsManager::ApplyBodyConstraints(PhysicsBody* body) const {
    if (!body) return;
    if (body->_freezeRotation) body->_angularVelocity = VGet(0,0,0);
    if (body->_isSleeping)     { body->_velocity = VGet(0,0,0); body->_angularVelocity = VGet(0,0,0); }
}
