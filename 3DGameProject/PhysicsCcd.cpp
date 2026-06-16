#include "PhysicsCcd.h"
#include "PhysicsManager_Internal.h"
#include "SphereCollider.h"
#include "BoxCollider.h"
#include "CapsuleCollider.h"
#include "GameObject.h"
#include <algorithm>
#include <cmath>
#include <cfloat>

namespace {
    inline float Clamp(float value, float minVal, float maxVal) noexcept {
        return (std::max)(minVal, (std::min)(value, maxVal));
    }

    inline PhysicsBody* FindDynamicBodyByOwner(GameObject* owner) noexcept {
        if (!owner) return nullptr;
        const auto& bodies = PhysicsManager::Instance().GetBodies();
        for (auto* b : bodies) {
            if (!b) continue;
            if (b->_owner != owner) continue;
            if (!b->IsDynamic()) continue;
            return b;
        }
        return nullptr;
    }
}

// 球-球の衝突時刻（TOI）を計算
// 二次方程式を用いて、運動する2つの球の衝突時刻を求める
float PhysicsCcd::ComputeTOI_SphereSphere(
    const VECTOR& centerA0, const VECTOR& centerA1, float radiusA,
    const VECTOR& centerB0, const VECTOR& centerB1, float radiusB,
    VECTOR* outHitNormal,
    VECTOR* outHitPoint
) {
    const VECTOR relPos = VSub(centerA0, centerB0); // 相対位置
    const VECTOR relVel = VSub(VSub(centerA1, centerA0), VSub(centerB1, centerB0)); // 相対速度
    const float radiusSum = radiusA + radiusB; // 接触判定用の半径合計

    // 二次方程式の係数を計算: a*t^2 + b*t + c = 0
    const float a = LenSq(relVel); // 二次項の係数
    const float b = 2.0f * Dot3(relPos, relVel); // 一次項の係数
    const float c = LenSq(relPos) - radiusSum * radiusSum; // 定数項

    if (a < 1e-8f) {
        return (c < 0.0f) ? 0.0f : 1.0f;
    }

    const float discriminant = b * b - 4.0f * a * c; // 判別式
    if (discriminant < 0.0f) {
        return 1.0f;
    }

    const float sqrtD = std::sqrt(discriminant); // 判別式の平方根
    const float t1 = (-b - sqrtD) / (2.0f * a); // 最初の解
    const float t2 = (-b + sqrtD) / (2.0f * a); // 2番目の解

    float toi = 1.0f;
    if (t1 >= 0.0f && t1 <= 1.0f) {
        toi = t1;
    } else if (t2 >= 0.0f && t2 <= 1.0f) {
        toi = t2;
    }

    if (toi < 1.0f && outHitNormal) {
        const VECTOR posA = VAdd(centerA0, VScale(VSub(centerA1, centerA0), toi)); // TOI時刻での球Aの位置
        const VECTOR posB = VAdd(centerB0, VScale(VSub(centerB1, centerB0), toi)); // TOI時刻での球Bの位置
        *outHitNormal = SafeNormalize(VSub(posA, posB), VGet(0, 1, 0));

        if (outHitPoint) {
            *outHitPoint = VAdd(posB, VScale(*outHitNormal, radiusB));
        }
    }

    return toi;
}

// 球-立方体の衝突時刻を計算
// スラブテスト（Slab Test）を使用して、3軸それぞれで最小接触時刻を求める
float PhysicsCcd::ComputeTOI_SphereBox(
    const VECTOR& sphereStart, const VECTOR& sphereEnd, float radius,
    const VECTOR& boxCenter, const VECTOR& boxHalfExtents,
    const VECTOR& boxAxisX, const VECTOR& boxAxisY, const VECTOR& boxAxisZ,
    VECTOR* outHitNormal,
    VECTOR* outHitPoint
) {
    const VECTOR localStart = VGet(
        Dot3(VSub(sphereStart, boxCenter), boxAxisX),
        Dot3(VSub(sphereStart, boxCenter), boxAxisY),
        Dot3(VSub(sphereStart, boxCenter), boxAxisZ)
    ); // 球の軌跡をボックスのローカル座標系に変換（開始位置）

    const VECTOR localEnd = VGet(
        Dot3(VSub(sphereEnd, boxCenter), boxAxisX),
        Dot3(VSub(sphereEnd, boxCenter), boxAxisY),
        Dot3(VSub(sphereEnd, boxCenter), boxAxisZ)
    ); // 球の軌跡をボックスのローカル座標系に変換（終了位置）

    const VECTOR dir = VSub(localEnd, localStart); // 移動方向ベクトル
    const VECTOR extents = VAdd(boxHalfExtents, VGet(radius, radius, radius)); // 球の半径を含む拡張範囲

    float tMin = 0.0f; // スラブテストの最小交点時刻
    float tMax = 1.0f; // スラブテストの最大交点時刻
    VECTOR hitNormalLocal = VGet(0, 0, 0); // ローカル座標系での衝突法線

    for (int axis = 0; axis < 3; ++axis) {
        const float origin = (axis == 0) ? localStart.x : (axis == 1) ? localStart.y : localStart.z; // 軸ごとの開始位置
        const float direction = (axis == 0) ? dir.x : (axis == 1) ? dir.y : dir.z; // 軸ごとの移動方向
        const float extent = (axis == 0) ? extents.x : (axis == 1) ? extents.y : extents.z; // 軸ごとの範囲

        if (std::fabs(direction) < 1e-6f) {
            if (origin < -extent || origin > extent) {
                return 1.0f;
            }
            continue;
        }

        float t1 = (-extent - origin) / direction; // スラブの下側との交点
        float t2 = (extent - origin) / direction; // スラブの上側との交点

        VECTOR nearNormal = VGet(0, 0, 0);
        if (axis == 0) nearNormal.x = (t1 <= t2) ? -1.0f : 1.0f;
        else if (axis == 1) nearNormal.y = (t1 <= t2) ? -1.0f : 1.0f;
        else nearNormal.z = (t1 <= t2) ? -1.0f : 1.0f;

        if (t1 > t2) std::swap(t1, t2); // t1 <= t2 に正規化

        if (t1 > tMin) {
            tMin = t1;
            hitNormalLocal = nearNormal;
        }
        tMax = (std::min)(tMax, t2);

        if (tMin > tMax) {
            return 1.0f;
        }
    }

    if (tMin < 0.0f || tMin > 1.0f) {
        return 1.0f;
    }

    if (outHitNormal) {
        *outHitNormal = SafeNormalize(
            VAdd(VAdd(VScale(boxAxisX, hitNormalLocal.x), VScale(boxAxisY, hitNormalLocal.y)), VScale(boxAxisZ, hitNormalLocal.z)),
            VGet(0, 1, 0)
        );
    }

    if (outHitPoint) {
        *outHitPoint = VAdd(sphereStart, VScale(VSub(sphereEnd, sphereStart), tMin));
    }

    return tMin;
}

// 点 p から線分 ab への最近点を計算
VECTOR PhysicsCcd::ClosestPointOnSegment(const VECTOR& p, const VECTOR& a, const VECTOR& b) {
    const VECTOR ab = VSub(b, a); // 線分の方向ベクトル
    const float t = Clamp(Dot3(VSub(p, a), ab) / (std::max)(LenSq(ab), 1e-8f), 0.0f, 1.0f); // 線分上の位置パラメータ
    return VAdd(a, VScale(ab, t));
}

// 2つの線分間の最近点を計算
void PhysicsCcd::ClosestPointSegmentSegment(
    const VECTOR& p1, const VECTOR& q1,
    const VECTOR& p2, const VECTOR& q2,
    float& s, float& t,
    VECTOR& c1, VECTOR& c2
) {
    const VECTOR d1 = VSub(q1, p1); // 線分1の方向ベクトル
    const VECTOR d2 = VSub(q2, p2); // 線分2の方向ベクトル
    const VECTOR r = VSub(p1, p2); // 線分1の開始点から線分2の開始点への相対ベクトル

    const float a = LenSq(d1); // 線分1の長さの二乗
    const float e = LenSq(d2); // 線分2の長さの二乗
    const float f = Dot3(d2, r); // d2 と r のドット積

    if (a <= 1e-8f && e <= 1e-8f) {
        s = t = 0.0f;
        c1 = p1;
        c2 = p2;
        return;
    }

    if (a <= 1e-8f) {
        s = 0.0f;
        t = Clamp(f / e, 0.0f, 1.0f);
    } else {
        const float c = Dot3(d1, r); // d1 と r のドット積
        if (e <= 1e-8f) {
            t = 0.0f;
            s = Clamp(-c / a, 0.0f, 1.0f);
        } else {
            const float b = Dot3(d1, d2); // d1 と d2 のドット積
            const float denom = a * e - b * b; // 分母

            if (denom != 0.0f) {
                s = Clamp((b * f - c * e) / denom, 0.0f, 1.0f);
            } else {
                s = 0.0f;
            }

            t = (b * s + f) / e;

            if (t < 0.0f) {
                t = 0.0f;
                s = Clamp(-c / a, 0.0f, 1.0f);
            } else if (t > 1.0f) {
                t = 1.0f;
                s = Clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    c1 = VAdd(p1, VScale(d1, s)); // 線分1上の最近点
    c2 = VAdd(p2, VScale(d2, t)); // 線分2上の最近点
}

// 球-カプセルの衝突時刻を計算
// サンプリング手法を使用して軌跡上の複数点で距離をチェック
float PhysicsCcd::ComputeTOI_SphereCapsule(
    const VECTOR& sphereStart, const VECTOR& sphereEnd, float sphereRadius,
    const VECTOR& capsuleP0, const VECTOR& capsuleP1, float capsuleRadius,
    VECTOR* outHitNormal,
    VECTOR* outHitPoint
) {
    const float combinedRadius = sphereRadius + capsuleRadius; // 衝突判定用の半径合計
    float minTOI = 1.0f; // 最小のTOI値

    const int samples = 10; // サンプリング分割数
    for (int i = 0; i <= samples; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(samples); // サンプル時刻
        const VECTOR spherePos = VAdd(sphereStart, VScale(VSub(sphereEnd, sphereStart), t)); // サンプル時刻での球の位置
        const VECTOR closest = ClosestPointOnSegment(spherePos, capsuleP0, capsuleP1); // カプセル上での最近点
        const float dist = Len3(VSub(spherePos, closest)); // 球からカプセルへの距離

        if (dist < combinedRadius) {
            minTOI = (std::min)(minTOI, t);
            if (outHitNormal) {
                *outHitNormal = SafeNormalize(VSub(spherePos, closest), VGet(0, 1, 0));
            }
            if (outHitPoint) {
                *outHitPoint = VAdd(closest, VScale(*outHitNormal, capsuleRadius));
            }
            break;
        }
    }

    return minTOI;
}

// 球と複数の衝突体の最初の衝突時刻を計算
// 複数の衝突体の種類（球、立方体、カプセル）に対応
TOIResult PhysicsCcd::ComputeTOI_Sphere(
    const VECTOR& startPos,
    const VECTOR& endPos,
    float radius,
    Collider* movingCollider,
    const std::vector<Collider*>& staticColliders,
    float allowedPenetration
) {
    TOIResult result;
    result.toi = 1.0f;

    for (auto* col : staticColliders) {
        if (!col || col == movingCollider) continue;
        if (!col->owner || !col->owner->IsActive()) continue;

        float toi = 1.0f; // このコライダーとの衝突時刻
        VECTOR hitNormal = VGet(0, 1, 0); // 衝突法線
        VECTOR hitPoint = VGet(0, 0, 0); // 衝突点

        if (col->GetKind() == Collider::Kind::Sphere) {
            auto* sphere = static_cast<SphereCollider*>(col);
            const VECTOR center = sphere->GetCenter();
            toi = ComputeTOI_SphereSphere(
                startPos, endPos, radius,
                center, center, sphere->GetRadius() + allowedPenetration,
                &hitNormal, &hitPoint
            );
        } else if (col->GetKind() == Collider::Kind::Box) {
            auto* box = static_cast<BoxCollider*>(col);
            toi = ComputeTOI_SphereBox(
                startPos, endPos, radius,
                box->GetCenter(), box->GetHalfExtents(),
                box->GetAxisX(), box->GetAxisY(), box->GetAxisZ(),
                &hitNormal, &hitPoint
            );
        } else if (col->GetKind() == Collider::Kind::Capsule) {
            auto* capsule = static_cast<CapsuleCollider*>(col);
            toi = ComputeTOI_SphereCapsule(
                startPos, endPos, radius,
                capsule->GetBottom(), capsule->GetTop(), capsule->GetRadius() + allowedPenetration,
                &hitNormal, &hitPoint
            );
        }

        if (toi < result.toi) {
            result.toi = toi;
            result.hit = true;
            result.hitNormal = hitNormal;
            result.hitPoint = hitPoint;
            result.hitCollider = col;
        }
    }

    return result;
}

// 立方体と複数の衝突体の最初の衝突時刻を計算
// 立方体の外接球を使用して高速な TOI 推定を行う
TOIResult PhysicsCcd::ComputeTOI_Box(
    const VECTOR& startPos,
    const VECTOR& endPos,
    const VECTOR& halfExtents,
    const VECTOR& axisX,
    const VECTOR& axisY,
    const VECTOR& axisZ,
    Collider* movingCollider,
    const std::vector<Collider*>& staticColliders,
    float allowedPenetration
) {
    const float boundingSphereRadius = Len3(halfExtents); // 外接球の半径
    return ComputeTOI_Sphere(startPos, endPos, boundingSphereRadius, movingCollider, staticColliders, allowedPenetration);
}

// カプセルと複数の衝突体の最初の衝突時刻を計算
// カプセルの外接球を使用して高速な TOI 推定を行う
TOIResult PhysicsCcd::ComputeTOI_Capsule(
    const VECTOR& startPos,
    const VECTOR& endPos,
    float radius,
    float halfHeight,
    const VECTOR& axis,
    Collider* movingCollider,
    const std::vector<Collider*>& staticColliders,
    float allowedPenetration
) {
    const float boundingSphereRadius = radius + halfHeight; // 外接球の半径
    return ComputeTOI_Sphere(startPos, endPos, boundingSphereRadius, movingCollider, staticColliders, allowedPenetration);
}

// 連続衝突検出（CCD）を処理
// CCD 品質レベルに応じて、衝突回避や位置補正を実行
void PhysicsCcd::ProcessCCD(
    PhysicsBody* body,
    Collider* collider,
    float stepDt,
    const std::vector<Collider*>& allColliders
) {
    if (!body || !collider || !body->_owner) return;
    if (!body->IsDynamic()) return;

    const CcdQuality quality = body->_ccdQuality; // CCD品質レベル

    if (quality == CcdQuality::Discrete) {
        return;
    }

    const VECTOR currentPos = body->_owner->transform.WorldPosition(); // 現在位置
    const VECTOR predictedPos = VAdd(currentPos, VScale(body->_velocity, stepDt)); // 予測位置
    const float displacementSq = LenSq(VSub(predictedPos, currentPos)); // 移動距離の二乗（sqrt不要）

    float minRadius = 0.5f; // コライダーの最小サイズ
    if (collider->GetKind() == Collider::Kind::Sphere) {
        minRadius = static_cast<SphereCollider*>(collider)->GetRadius();
    } else if (collider->GetKind() == Collider::Kind::Box) {
        const VECTOR he = static_cast<BoxCollider*>(collider)->GetHalfExtents();
        minRadius = (std::min)({he.x, he.y, he.z});
    } else if (collider->GetKind() == Collider::Kind::Capsule) {
        minRadius = static_cast<CapsuleCollider*>(collider)->GetRadius();
    }

    const float threshold = (quality == CcdQuality::Debris) ? (minRadius * 2.0f) : // CCD品質に応じた速度制限の閾値
                            (quality == CcdQuality::Default) ? (minRadius * 1.0f) :
                            (quality == CcdQuality::Bullet) ? (minRadius * 0.5f) :
                            (minRadius * 0.2f);

    if (displacementSq < threshold * threshold) {
        return;
    }

    TOIResult toiResult; // TOI計算の結果
    if (collider->GetKind() == Collider::Kind::Sphere) {
        auto* sphere = static_cast<SphereCollider*>(collider);
        toiResult = ComputeTOI_Sphere(
            currentPos, predictedPos, sphere->GetRadius(),
            collider, allColliders, body->_allowedPenetrationDepth
        );
    } else if (collider->GetKind() == Collider::Kind::Box) {
        auto* box = static_cast<BoxCollider*>(collider);
        toiResult = ComputeTOI_Box(
            currentPos, predictedPos, box->GetHalfExtents(),
            box->GetAxisX(), box->GetAxisY(), box->GetAxisZ(),
            collider, allColliders, body->_allowedPenetrationDepth
        );
    } else if (collider->GetKind() == Collider::Kind::Capsule) {
        auto* capsule = static_cast<CapsuleCollider*>(collider);
        const VECTOR axis = SafeNormalize(VSub(capsule->GetTop(), capsule->GetBottom()), VGet(0, 1, 0));
        const float halfHeight = Len3(VSub(capsule->GetTop(), capsule->GetBottom())) * 0.5f; // カプセルの半高さ
        toiResult = ComputeTOI_Capsule(
            currentPos, predictedPos, capsule->GetRadius(), halfHeight, axis,
            collider, allColliders, body->_allowedPenetrationDepth
        );
    }

    if (!toiResult.hit) {
        return;
    }

    if (quality == CcdQuality::Debris) {
        return;
    }

    if (quality == CcdQuality::Default) {
        const float speedLimit = minRadius / stepDt; // 速度の上限
        const float speedLimitSq = speedLimit * speedLimit;
        const float currentSpeedSq = LenSq(body->_velocity); // 二乗で早期判定
        if (currentSpeedSq > speedLimitSq) {
            const float currentSpeed = std::sqrt(currentSpeedSq); // スケール限りのため sqrt
            body->_velocity = VScale(body->_velocity, speedLimit / currentSpeed);
        }
        return;
    }

    if (quality == CcdQuality::Bullet || quality == CcdQuality::Critical) {
        const bool hitStatic = !toiResult.hitCollider || !toiResult.hitCollider->owner || toiResult.hitCollider->owner->isStatic; // 静的ジオメトリかどうかの判定

        if (hitStatic) {
            const float safeTOI = (std::max)(toiResult.toi - 0.01f, 0.0f); // 安全なTOI時刻
            const VECTOR safePos = VAdd(currentPos, VScale(VSub(predictedPos, currentPos), safeTOI)); // 安全な位置
            body->_owner->transform.SetLocalPosition(safePos);

            const float vn = Dot3(body->_velocity, toiResult.hitNormal); // 法線方向の速度成分
            if (vn < 0.0f) {
                const VECTOR vt = VSub(body->_velocity, VScale(toiResult.hitNormal, vn)); // 接線方向の速度
                const float e = std::clamp(body->_restitution, 0.0f, 1.0f); // 反発係数
                const float friction = std::clamp(body->_friction, 0.0f, 1.0f); // 摩擦係数
                const float tangentialScale = 1.0f / (1.0f + friction); // 接線速度のスケール
                const VECTOR vtDamped = VScale(vt, tangentialScale); // 摩擦で減衰した接線速度
                body->_velocity = VAdd(vtDamped, VScale(toiResult.hitNormal, -vn * e));
            }
        } else {
            const float safeTOI = (std::max)(toiResult.toi - 0.01f, 0.0f);
            const VECTOR safePos = VAdd(currentPos, VScale(VSub(predictedPos, currentPos), safeTOI));
            body->_owner->transform.SetLocalPosition(safePos);
        }
    }
}

// 予測接触を生成
void PhysicsManager::GenerateSpeculativeContacts(float stepDt) {
    if (stepDt <= 1e-6f) return;
    if (!_havokCcdEnabled && !_speculativeCcdEnabled) return;

    const auto& allColliders = ColliderManager::Instance().GetColliders();

    for (auto* body : _bodies) {
        if (!body || !body->IsDynamic()) continue;
        if (!body->_owner || !body->_owner->IsActive()) continue;

        Collider* col = CachedFindCollider(body->_owner);
        if (!col) continue;

        if (body->_ccdQuality == CcdQuality::Discrete) continue;

        PhysicsCcd::ProcessCCD(body, col, stepDt, allColliders);
    }
}
