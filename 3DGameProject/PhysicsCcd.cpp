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

float PhysicsCcd::ComputeTOI_SphereSphere(
    const VECTOR& centerA0, const VECTOR& centerA1, float radiusA,
    const VECTOR& centerB0, const VECTOR& centerB1, float radiusB,
    VECTOR* outHitNormal,
    VECTOR* outHitPoint
) {
    const VECTOR relPos = VSub(centerA0, centerB0);
    const VECTOR relVel = VSub(VSub(centerA1, centerA0), VSub(centerB1, centerB0));
    const float radiusSum = radiusA + radiusB;

    const float a = LenSq(relVel);
    const float b = 2.0f * Dot3(relPos, relVel);
    const float c = LenSq(relPos) - radiusSum * radiusSum;

    if (a < 1e-8f) {
        return (c < 0.0f) ? 0.0f : 1.0f;
    }

    const float discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0.0f) {
        return 1.0f;
    }

    const float sqrtD = std::sqrt(discriminant);
    const float t1 = (-b - sqrtD) / (2.0f * a);
    const float t2 = (-b + sqrtD) / (2.0f * a);

    float toi = 1.0f;
    if (t1 >= 0.0f && t1 <= 1.0f) {
        toi = t1;
    } else if (t2 >= 0.0f && t2 <= 1.0f) {
        toi = t2;
    }

    if (toi < 1.0f && outHitNormal) {
        const VECTOR posA = VAdd(centerA0, VScale(VSub(centerA1, centerA0), toi));
        const VECTOR posB = VAdd(centerB0, VScale(VSub(centerB1, centerB0), toi));
        *outHitNormal = SafeNormalize(VSub(posA, posB), VGet(0, 1, 0));

        if (outHitPoint) {
            *outHitPoint = VAdd(posB, VScale(*outHitNormal, radiusB));
        }
    }

    return toi;
}

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
    );
    const VECTOR localEnd = VGet(
        Dot3(VSub(sphereEnd, boxCenter), boxAxisX),
        Dot3(VSub(sphereEnd, boxCenter), boxAxisY),
        Dot3(VSub(sphereEnd, boxCenter), boxAxisZ)
    );

    const VECTOR dir = VSub(localEnd, localStart);
    const VECTOR extents = VAdd(boxHalfExtents, VGet(radius, radius, radius));

    float tMin = 0.0f;
    float tMax = 1.0f;
    VECTOR hitNormalLocal = VGet(0, 0, 0);

    for (int axis = 0; axis < 3; ++axis) {
        const float origin = (axis == 0) ? localStart.x : (axis == 1) ? localStart.y : localStart.z;
        const float direction = (axis == 0) ? dir.x : (axis == 1) ? dir.y : dir.z;
        const float extent = (axis == 0) ? extents.x : (axis == 1) ? extents.y : extents.z;

        if (std::fabs(direction) < 1e-6f) {
            if (origin < -extent || origin > extent) {
                return 1.0f;
            }
            continue;
        }

        float t1 = (-extent - origin) / direction;
        float t2 = (extent - origin) / direction;

        VECTOR nearNormal = VGet(0, 0, 0);
        if (axis == 0) nearNormal.x = (t1 <= t2) ? -1.0f : 1.0f;
        else if (axis == 1) nearNormal.y = (t1 <= t2) ? -1.0f : 1.0f;
        else nearNormal.z = (t1 <= t2) ? -1.0f : 1.0f;

        if (t1 > t2) std::swap(t1, t2);

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

VECTOR PhysicsCcd::ClosestPointOnSegment(const VECTOR& p, const VECTOR& a, const VECTOR& b) {
    const VECTOR ab = VSub(b, a);
    const float t = Clamp(Dot3(VSub(p, a), ab) / (std::max)(LenSq(ab), 1e-8f), 0.0f, 1.0f);
    return VAdd(a, VScale(ab, t));
}

void PhysicsCcd::ClosestPointSegmentSegment(
    const VECTOR& p1, const VECTOR& q1,
    const VECTOR& p2, const VECTOR& q2,
    float& s, float& t,
    VECTOR& c1, VECTOR& c2
) {
    const VECTOR d1 = VSub(q1, p1);
    const VECTOR d2 = VSub(q2, p2);
    const VECTOR r = VSub(p1, p2);

    const float a = LenSq(d1);
    const float e = LenSq(d2);
    const float f = Dot3(d2, r);

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
        const float c = Dot3(d1, r);
        if (e <= 1e-8f) {
            t = 0.0f;
            s = Clamp(-c / a, 0.0f, 1.0f);
        } else {
            const float b = Dot3(d1, d2);
            const float denom = a * e - b * b;

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

    c1 = VAdd(p1, VScale(d1, s));
    c2 = VAdd(p2, VScale(d2, t));
}

float PhysicsCcd::ComputeTOI_SphereCapsule(
    const VECTOR& sphereStart, const VECTOR& sphereEnd, float sphereRadius,
    const VECTOR& capsuleP0, const VECTOR& capsuleP1, float capsuleRadius,
    VECTOR* outHitNormal,
    VECTOR* outHitPoint
) {
    const float combinedRadius = sphereRadius + capsuleRadius;
    float minTOI = 1.0f;

    const int samples = 10;
    for (int i = 0; i <= samples; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(samples);
        const VECTOR spherePos = VAdd(sphereStart, VScale(VSub(sphereEnd, sphereStart), t));
        const VECTOR closest = ClosestPointOnSegment(spherePos, capsuleP0, capsuleP1);
        const float dist = Len3(VSub(spherePos, closest));

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

        float toi = 1.0f;
        VECTOR hitNormal = VGet(0, 1, 0);
        VECTOR hitPoint = VGet(0, 0, 0);

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
    const float boundingSphereRadius = Len3(halfExtents);
    return ComputeTOI_Sphere(startPos, endPos, boundingSphereRadius, movingCollider, staticColliders, allowedPenetration);
}

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
    const float boundingSphereRadius = radius + halfHeight;
    return ComputeTOI_Sphere(startPos, endPos, boundingSphereRadius, movingCollider, staticColliders, allowedPenetration);
}

void PhysicsCcd::ProcessCCD(
    PhysicsBody* body,
    Collider* collider,
    float stepDt,
    const std::vector<Collider*>& allColliders
) {
    if (!body || !collider || !body->_owner) return;
    if (!body->IsDynamic()) return;

    const CcdQuality quality = body->_ccdQuality;

    if (quality == CcdQuality::Discrete) {
        return;
    }

    const VECTOR currentPos = body->_owner->transform.WorldPosition();
    const VECTOR predictedPos = VAdd(currentPos, VScale(body->_velocity, stepDt));
    const float displacement = Len3(VSub(predictedPos, currentPos));

    float minRadius = 0.5f;
    if (collider->GetKind() == Collider::Kind::Sphere) {
        minRadius = static_cast<SphereCollider*>(collider)->GetRadius();
    } else if (collider->GetKind() == Collider::Kind::Box) {
        const VECTOR he = static_cast<BoxCollider*>(collider)->GetHalfExtents();
        minRadius = (std::min)({he.x, he.y, he.z});
    } else if (collider->GetKind() == Collider::Kind::Capsule) {
        minRadius = static_cast<CapsuleCollider*>(collider)->GetRadius();
    }

    const float threshold = (quality == CcdQuality::Debris) ? (minRadius * 2.0f) :
                            (quality == CcdQuality::Default) ? (minRadius * 1.0f) :
                            (quality == CcdQuality::Bullet) ? (minRadius * 0.5f) :
                            (minRadius * 0.2f);

    if (displacement < threshold) {
        return;
    }

    TOIResult toiResult;
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
        const float halfHeight = Len3(VSub(capsule->GetTop(), capsule->GetBottom())) * 0.5f;
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
        const float speedLimit = minRadius / stepDt;
        const float currentSpeed = Len3(body->_velocity);
        if (currentSpeed > speedLimit) {
            body->_velocity = VScale(body->_velocity, speedLimit / currentSpeed);
        }
        return;
    }

    if (quality == CcdQuality::Bullet || quality == CcdQuality::Critical) {
        const bool hitStatic = !toiResult.hitCollider || !toiResult.hitCollider->owner || toiResult.hitCollider->owner->isStatic;

        if (hitStatic) {
            const float safeTOI = (std::max)(toiResult.toi - 0.01f, 0.0f);
            const VECTOR safePos = VAdd(currentPos, VScale(VSub(predictedPos, currentPos), safeTOI));
            body->_owner->transform.SetLocalPosition(safePos);

            // Against static geometry, remove inward normal speed and keep tangential component.
            // Add restitution-based bounce so CCD backstep does not kill rebound.
            const float vn = Dot3(body->_velocity, toiResult.hitNormal);
            if (vn < 0.0f) {
                const VECTOR vt = VSub(body->_velocity, VScale(toiResult.hitNormal, vn));
                const float e = std::clamp(body->_restitution, 0.0f, 1.0f);
                body->_velocity = VAdd(vt, VScale(toiResult.hitNormal, -vn * e));
            }
        } else {
            // Dynamic-vs-dynamic: transfer momentum immediately so first-hit reaction is visible
            // even when narrow-phase contact is not produced in the same frame.
            PhysicsBody* other = FindDynamicBodyByOwner(toiResult.hitCollider ? toiResult.hitCollider->owner : nullptr);
            if (other && other != body) {
                const VECTOR n = SafeNormalize(toiResult.hitNormal, VGet(0, 1, 0));
                const float relVn = Dot3(VSub(body->_velocity, other->_velocity), n);
                if (relVn < -1e-4f) {
                    const float invA = body->InverseMass();
                    const float invB = other->InverseMass();
                    const float invSum = invA + invB;
                    if (invSum > 1e-8f) {
                        const float e = std::clamp((std::max)(body->_restitution, other->_restitution), 0.0f, 1.0f);
                        const float j = (-(1.0f + e) * relVn) / invSum;
                        const VECTOR imp = VScale(n, j);
                        body->_velocity = VAdd(body->_velocity, VScale(imp, invA));
                        other->_velocity = VSub(other->_velocity, VScale(imp, invB));
                        body->WakeUp();
                        other->WakeUp();
                    }
                }
            }
        }

        if (quality == CcdQuality::Critical) {
            body->_velocity = VScale(body->_velocity, 0.8f);
        }
    }
}

void PhysicsManager::GenerateSpeculativeContacts(float stepDt) {
    if (stepDt <= 1e-6f) return;
    if (!_havokCcdEnabled) return;

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
