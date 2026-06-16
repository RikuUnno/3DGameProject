#include "Raycast.h"
#include "ColliderManager.h"
#include "SphereCollider.h"
#include "BoxCollider.h"
#include "CapsuleCollider.h"
#include "GameObject.h"
#include "BitOperation.h"
#include <algorithm>
#include <cmath>
#include <cfloat>

namespace {
    inline float Dot3(const VECTOR& a, const VECTOR& b) noexcept {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
    inline float LenSq(const VECTOR& v) noexcept { return Dot3(v, v); }
    inline float Len3(const VECTOR& v) noexcept { return std::sqrt((std::max)(LenSq(v), 0.0f)); }
    inline VECTOR SafeNorm(const VECTOR& v, const VECTOR& fb = VGet(0, 1, 0)) noexcept {
        const float l = Len3(v);
        return (l > 1e-6f) ? VScale(v, 1.0f / l) : fb;
    }

    // Ray vs AABB (slab method). Returns tMin if hit, or negative if miss.
    inline bool SlabAABB(const Ray& ray, const VECTOR& aabbMin, const VECTOR& aabbMax,
        float maxDist, float* outT) noexcept
    {
        float tMin = 0.0f;
        float tMax = maxDist;

        const float o[3] = { ray.origin.x, ray.origin.y, ray.origin.z };
        const float d[3] = { ray.direction.x, ray.direction.y, ray.direction.z };
        const float mn[3] = { aabbMin.x, aabbMin.y, aabbMin.z };
        const float mx[3] = { aabbMax.x, aabbMax.y, aabbMax.z };

        for (int i = 0; i < 3; ++i) {
            if (std::fabs(d[i]) < 1e-8f) {
                if (o[i] < mn[i] || o[i] > mx[i]) return false;
            }
            else {
                float invD = 1.0f / d[i];
                float t1 = (mn[i] - o[i]) * invD;
                float t2 = (mx[i] - o[i]) * invD;
                if (t1 > t2) std::swap(t1, t2);
                tMin = (std::max)(tMin, t1);
                tMax = (std::min)(tMax, t2);
                if (tMin > tMax) return false;
            }
        }
        if (outT) *outT = tMin;
        return true;
    }

    // Closest point on line segment [a,b] to point p, returns parameter t in [0,1].
    inline float ClosestPointOnSegment(const VECTOR& a, const VECTOR& b, const VECTOR& p) noexcept {
        const VECTOR ab = VSub(b, a);
        const float abLenSq = LenSq(ab);
        if (abLenSq < 1e-8f) return 0.0f;
        float t = Dot3(VSub(p, a), ab) / abLenSq;
        return std::clamp(t, 0.0f, 1.0f);
    }
}

// ============================================================
//  Ray vs Sphere
// ============================================================
bool Raycast::TestSphere(
    const Ray& ray, const VECTOR& center, float radius,
    float maxDistance, float* outDistance, VECTOR* outPoint, VECTOR* outNormal)
{
    const VECTOR oc = VSub(ray.origin, center);
    const float b = Dot3(oc, ray.direction);
    const float c = Dot3(oc, oc) - radius * radius;
    const float disc = b * b - c;
    if (disc < 0.0f) return false;

    float t = -b - std::sqrt(disc);
    if (t < 0.0f) {
        // origin is inside the sphere ? use the far intersection
        t = -b + std::sqrt(disc);
        if (t < 0.0f) return false;
    }
    if (t > maxDistance) return false;

    const VECTOR hitPt = VAdd(ray.origin, VScale(ray.direction, t));
    if (outDistance) *outDistance = t;
    if (outPoint) *outPoint = hitPt;
    if (outNormal) *outNormal = SafeNorm(VSub(hitPt, center));
    return true;
}

// ============================================================
//  Ray vs AABB
// ============================================================
bool Raycast::TestAABB(
    const Ray& ray, const AABB& aabb, float maxDistance, float* outDistance)
{
    return SlabAABB(ray, aabb.min, aabb.max, maxDistance, outDistance);
}

// ============================================================
//  Ray vs OBB (Box)
// ============================================================
bool Raycast::TestBox(
    const Ray& ray, const BoxCollider* box, float maxDistance,
    float* outDistance, VECTOR* outPoint, VECTOR* outNormal)
{
    if (!box) return false;

    // Transform ray into OBB local space
    const VECTOR boxCenter = box->GetCenter();
    const VECTOR axes[3] = { box->GetAxisX(), box->GetAxisY(), box->GetAxisZ() };
    const VECTOR he = box->GetHalfExtents();
    const float halfExt[3] = { he.x, he.y, he.z };

    const VECTOR delta = VSub(ray.origin, boxCenter);
    // local origin & direction
    float localO[3], localD[3];
    for (int i = 0; i < 3; ++i) {
        localO[i] = Dot3(delta, axes[i]);
        localD[i] = Dot3(ray.direction, axes[i]);
    }

    float tMin = 0.0f;
    float tMax = maxDistance;
    int hitAxis = -1;
    float hitSign = 0.0f;

    for (int i = 0; i < 3; ++i) {
        if (std::fabs(localD[i]) < 1e-8f) {
            if (localO[i] < -halfExt[i] || localO[i] > halfExt[i]) return false;
        }
        else {
            float invD = 1.0f / localD[i];
            float t1 = (-halfExt[i] - localO[i]) * invD;
            float t2 = (halfExt[i] - localO[i]) * invD;
            float nearSign = -1.0f;
            int nearAxis = i;
            if (t1 > t2) {
                std::swap(t1, t2);
                nearSign = 1.0f;
            }
            if (t1 > tMin) {
                tMin = t1;
                hitAxis = nearAxis;
                hitSign = nearSign;
            }
            tMax = (std::min)(tMax, t2);
            if (tMin > tMax) return false;
        }
    }

    if (tMin < 0.0f) return false;

    const VECTOR hitPt = VAdd(ray.origin, VScale(ray.direction, tMin));
    if (outDistance) *outDistance = tMin;
    if (outPoint) *outPoint = hitPt;
    if (outNormal) {
        if (hitAxis >= 0) {
            *outNormal = VScale(axes[hitAxis], hitSign);
        }
        else {
            *outNormal = VGet(0, 1, 0);
        }
    }
    return true;
}

// ============================================================
//  Ray vs Capsule (bottom, top, radius)
// ============================================================
bool Raycast::TestCapsule(
    const Ray& ray, const VECTOR& bottom, const VECTOR& top, float radius,
    float maxDistance, float* outDistance, VECTOR* outPoint, VECTOR* outNormal)
{
    // Strategy: test ray vs infinite cylinder, clamp to segment,
    // then test ray vs hemisphere caps.

    const VECTOR seg = VSub(top, bottom);
    const float segLenSq = LenSq(seg);
    const VECTOR segDir = (segLenSq > 1e-8f) ? VScale(seg, 1.0f / std::sqrt(segLenSq)) : VGet(0, 1, 0);
    const float segLen = std::sqrt((std::max)(segLenSq, 0.0f));

    // Infinite cylinder along segment axis
    const VECTOR oc = VSub(ray.origin, bottom);
    const float dDotSeg = Dot3(ray.direction, segDir);
    const float ocDotSeg = Dot3(oc, segDir);

    // Quadratic for infinite cylinder: |P(t) - bottom - ((P(t)-bottom)?segDir)segDir|? = r?
    const VECTOR dPerp = VSub(ray.direction, VScale(segDir, dDotSeg));
    const VECTOR ocPerp = VSub(oc, VScale(segDir, ocDotSeg));

    const float a = LenSq(dPerp);
    const float b = 2.0f * Dot3(dPerp, ocPerp);
    const float c = LenSq(ocPerp) - radius * radius;

    float bestT = FLT_MAX;
    VECTOR bestNormal = VGet(0, 1, 0);

    if (a > 1e-8f) {
        const float disc = b * b - 4.0f * a * c;
        if (disc >= 0.0f) {
            const float sqrtDisc = std::sqrt(disc);
            const float inv2a = 1.0f / (2.0f * a);
            float t0 = (-b - sqrtDisc) * inv2a;
            float t1 = (-b + sqrtDisc) * inv2a;

            for (float t : { t0, t1 }) {
                if (t < 0.0f || t > maxDistance) continue;
                // Check if hit is within the finite segment
                const VECTOR hitPt = VAdd(ray.origin, VScale(ray.direction, t));
                const float proj = Dot3(VSub(hitPt, bottom), segDir);
                if (proj >= 0.0f && proj <= segLen) {
                    if (t < bestT) {
                        bestT = t;
                        // Normal: from cylinder axis to hit point
                        const VECTOR axisPoint = VAdd(bottom, VScale(segDir, proj));
                        bestNormal = SafeNorm(VSub(hitPt, axisPoint));
                    }
                    break; // t0 < t1, so first valid hit is closest
                }
            }
        }
    }

    // Test hemisphere caps (bottom and top)
    float capDist;
    VECTOR capPt, capN;
    if (TestSphere(ray, bottom, radius, maxDistance, &capDist, &capPt, &capN)) {
        // Only count if the hit is on the correct hemisphere (below the bottom plane)
        const float proj = Dot3(VSub(capPt, bottom), segDir);
        if (proj <= 0.0f && capDist < bestT) {
            bestT = capDist;
            bestNormal = capN;
        }
    }
    if (TestSphere(ray, top, radius, maxDistance, &capDist, &capPt, &capN)) {
        const float proj = Dot3(VSub(capPt, top), segDir);
        if (proj >= 0.0f && capDist < bestT) {
            bestT = capDist;
            bestNormal = capN;
        }
    }

    if (bestT > maxDistance || bestT == FLT_MAX) return false;

    if (outDistance) *outDistance = bestT;
    if (outPoint) *outPoint = VAdd(ray.origin, VScale(ray.direction, bestT));
    if (outNormal) *outNormal = bestNormal;
    return true;
}

// ============================================================
//  Scene Raycast ? single closest hit
// ============================================================
RaycastHit Raycast::Cast(const Ray& ray, float maxDistance, int layerMask) {
    RaycastHit result;
    result.hit = false;
    result.distance = FLT_MAX;

    auto hits = CastAll(ray, maxDistance, layerMask);
    if (!hits.empty()) {
        result = hits.front(); // CastAll returns sorted by distance
    }
    return result;
}

// ============================================================
//  Scene Raycast ? all hits sorted by distance
// ============================================================
std::vector<RaycastHit> Raycast::CastAll(const Ray& ray, float maxDistance, int layerMask) {
    std::vector<RaycastHit> results;

    // Access registered colliders via ColliderManager.
    // We iterate all colliders, perform AABB broadphase, then narrow-phase per shape.
    const auto& colliders = ColliderManager::Instance().GetColliders();

    for (auto* col : colliders) {
        if (!col) continue;
        if (!col->IsEnabled()) continue;
        if (col->owner && !col->owner->IsActive()) continue;

        // Layer filter
        if (!BitOperation::HasAny(col->layer, layerMask)) continue;

        // Broad-phase: Ray vs AABB
        float aabbT = 0.0f;
        if (!TestAABB(ray, col->GetAABB(), maxDistance, &aabbT)) continue;

        // Narrow-phase per shape
        float hitDist = 0.0f;
        VECTOR hitPt = VGet(0, 0, 0);
        VECTOR hitNormal = VGet(0, 0, 0);
        bool hit = false;

        switch (col->GetKind()) {
        case Collider::Kind::Sphere: {
            auto* sc = static_cast<SphereCollider*>(col);
            hit = TestSphere(ray, sc->GetCenter(), sc->GetRadius(), maxDistance,
                &hitDist, &hitPt, &hitNormal);
            break;
        }
        case Collider::Kind::Box: {
            auto* bc = static_cast<BoxCollider*>(col);
            hit = TestBox(ray, bc, maxDistance, &hitDist, &hitPt, &hitNormal);
            break;
        }
        case Collider::Kind::Capsule: {
            auto* cc = static_cast<CapsuleCollider*>(col);
            hit = TestCapsule(ray, cc->GetBottom(), cc->GetTop(), cc->GetRadius(),
                maxDistance, &hitDist, &hitPt, &hitNormal);
            break;
        }
        default:
            break;
        }

        if (hit) {
            RaycastHit rh;
            rh.hit = true;
            rh.distance = hitDist;
            rh.point = hitPt;
            rh.normal = hitNormal;
            rh.collider = col;
            rh.gameObject = col->owner;
            results.push_back(rh);
        }
    }

    // Sort by distance (nearest first)
    std::sort(results.begin(), results.end(),
        [](const RaycastHit& a, const RaycastHit& b) { return a.distance < b.distance; });

    return results;
}
