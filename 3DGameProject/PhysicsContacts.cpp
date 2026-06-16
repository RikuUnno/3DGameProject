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
        // 注意: frictionLambda1/2 は前フレームの tangent1/2 基底に対する係数。
        // 球が Box の辺や面の境界をまたいで転がるとき、コライダー側の closest
        // 点が辺へジャンプすることで「接触点はほぼ同じ位置だが接触法線が
        // 大きく変わる」状況が発生する。
        //   - 接触点ベース (localA/B) でのマッチ距離 (kContactMatchDistSq) は
        //     球半径ぶんしかずれないので閾値内で素通り。
        //   - しかし前フレームの摩擦インパルス (世界空間ベクトル) は
        //     前フレーム法線と直交していたものなので、新しい法線の下では
        //     「摩擦」として再投影できず、低速・静摩擦域では大きな上限まで
        //     許容されてしまう。これがレバーアーム経由で角速度に化けて
        //     球が低速転がり中に突然飛ぶ原因になる。
        // → 法線が有意に変化していたら摩擦ラムダのみ破棄し、法線ラムダだけ
        //   再利用する (法線ラムダは基底に依存しない)。
        sc.normalLambda    = 0.0f;
        sc.frictionLambda1 = 0.0f;
        sc.frictionLambda2 = 0.0f;
        auto range = prevMap.equal_range(PrevKey{sc.colA, sc.colB});
        for (auto it = range.first; it != range.second; ++it) {
            const auto& prev = _prevSolverContacts[it->second];
            if (LenSq(VSub(prev.localA, sc.localA)) > kContactMatchDistSq) continue;
            if (LenSq(VSub(prev.localB, sc.localB)) > kContactMatchDistSq) continue;
            // Normal changed beyond 60 deg: the contact switched to another face
            // (e.g. a sphere crossing a box edge). Discard every warm-start
            // lambda. Carrying the old normalLambda over applies a large impulse
            // along the new normal and inflates the friction cone, which makes
            // spheres stick to edges and spin up.
            const float normalAlignment = Dot3(prev.normal, sc.normal);
            if (normalAlignment < 0.5f) break;
            // warm-start factor を保存時点で適用しておく。
            // こうすることで WarmStart() / SolveIsland() で読まれる
            // 累積ラムダは「前フレーム値 × factor」となり、ソルバ内の
            // クランプ・累積処理と整合する。WarmStart() でさらに factor を
            // 掛けると二重適用になり、保存値とソルバ累積値がずれる。
            sc.normalLambda = prev.normalLambda * kWarmStartFactor;
            // 法線の整合性を確認。cos(15°) ? 0.966 を閾値とする。
            // それ以下なら接触面が切り替わったとみなし、摩擦は warm-start
            // しない (ゼロから蓄積させる)。
            if (normalAlignment > 0.966f) {
                // 摩擦インパルスをワールド空間に復元してから新しい tangent 基底へ射影
                const VECTOR prevFrictionImpulse = VAdd(
                    VScale(prev.tangent1, prev.frictionLambda1),
                    VScale(prev.tangent2, prev.frictionLambda2));
                sc.frictionLambda1 = Dot3(prevFrictionImpulse, sc.tangent1) * kWarmStartFactor;
                sc.frictionLambda2 = Dot3(prevFrictionImpulse, sc.tangent2) * kWarmStartFactor;
            }
            break;
        }

        results[idx] = { sc, true };
    }, 8);

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

        // 接触法線方向の相対速度を確認する。
        // vn > 0 (離れていく) かつ penetration が小さい場合、その接触は
        // 「実際には押し付け合っていない」可能性が高い。
        // このとき前フレームの normalLambda をそのまま warm-start すると、
        // 押し付けていないのに大きな摩擦コーン (mu*normalLambda) が発生し、
        // 重力で落下しようとする球の鉛直速度を摩擦が完全にゼロ化して
        // 壁に張り付かせる現象が起きる。
        // → 浅い接触 (penetration <= kSlop) で離れていく接触は warm-start を破棄。
        const float vn = RelNormalVelocity(sc.bodyA, sc.bodyB, sc.rA, sc.rB, sc.normal);
        if (sc.penetration <= kSlop && vn > 0.0f) {
            sc.normalLambda    = 0.0f;
            sc.frictionLambda1 = 0.0f;
            sc.frictionLambda2 = 0.0f;
            continue;
        }

        // 摩擦ラムダは「現フレームの法線ラムダから決まる摩擦円錐」内に
        // 必ず収まっていなければならない。前フレームの normalLambda は
        // 衝撃で一時的に大きくなるため、そのまま warm-start に流すと
        // 現フレームの円錐を超える接線インパルスが注入され、球が辺/面で
        // 接触したまま転がる際に余剰トルクが残り、回転が暴走して飛ぶ。
        // → warm-start 前に摩擦円錐へクランプし、保存値も縮める。
        const float maxF = sc.friction * sc.normalLambda;
        const float fMagSq = sc.frictionLambda1 * sc.frictionLambda1
                           + sc.frictionLambda2 * sc.frictionLambda2;
        if (fMagSq > maxF * maxF && fMagSq > 1e-16f) {
            const float fMag = std::sqrt(fMagSq); // クランプが必要な時だけ sqrt
            const float s = maxF / fMag;
            sc.frictionLambda1 *= s;
            sc.frictionLambda2 *= s;
        }

        // factor は既に BuildSolverContacts で適用済みなのでここでは掛けない。
        const VECTOR warmN  = VScale(sc.normal,   sc.normalLambda);
        const VECTOR warmT1 = VScale(sc.tangent1, sc.frictionLambda1);
        const VECTOR warmT2 = VScale(sc.tangent2, sc.frictionLambda2);

        ApplyImpulse(sc.bodyA, sc.bodyB, sc.invA, sc.invB, sc.rA, sc.rB,
            VAdd(VAdd(warmN, warmT1), warmT2));
    }
}
