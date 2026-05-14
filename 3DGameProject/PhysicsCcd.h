#pragma once

#include "DxLib.h"
#include "PhysicsBody.h"
#include "Collider.h"
#include <vector>

struct TOIResult {
    bool hit = false;
    float toi = 1.0f;
    VECTOR hitNormal = VGet(0, 1, 0);
    VECTOR hitPoint = VGet(0, 0, 0);
    Collider* hitCollider = nullptr;
};

class PhysicsCcd {
public:
    static TOIResult ComputeTOI_Sphere(
        const VECTOR& startPos,
        const VECTOR& endPos,
        float radius,
        Collider* movingCollider,
        const std::vector<Collider*>& staticColliders,
        float allowedPenetration = 0.0f
    );

    static TOIResult ComputeTOI_Box(
        const VECTOR& startPos,
        const VECTOR& endPos,
        const VECTOR& halfExtents,
        const VECTOR& axisX,
        const VECTOR& axisY,
        const VECTOR& axisZ,
        Collider* movingCollider,
        const std::vector<Collider*>& staticColliders,
        float allowedPenetration = 0.0f
    );

    static TOIResult ComputeTOI_Capsule(
        const VECTOR& startPos,
        const VECTOR& endPos,
        float radius,
        float halfHeight,
        const VECTOR& axis,
        Collider* movingCollider,
        const std::vector<Collider*>& staticColliders,
        float allowedPenetration = 0.0f
    );

    static void ProcessCCD(
        PhysicsBody* body,
        Collider* collider,
        float stepDt,
        const std::vector<Collider*>& allColliders
    );

private:
    static float ComputeTOI_SphereSphere(
        const VECTOR& centerA0, const VECTOR& centerA1, float radiusA,
        const VECTOR& centerB0, const VECTOR& centerB1, float radiusB,
        VECTOR* outHitNormal = nullptr,
        VECTOR* outHitPoint = nullptr
    );

    static float ComputeTOI_SphereBox(
        const VECTOR& sphereStart, const VECTOR& sphereEnd, float radius,
        const VECTOR& boxCenter, const VECTOR& boxHalfExtents,
        const VECTOR& boxAxisX, const VECTOR& boxAxisY, const VECTOR& boxAxisZ,
        VECTOR* outHitNormal = nullptr,
        VECTOR* outHitPoint = nullptr
    );

    static float ComputeTOI_SphereCapsule(
        const VECTOR& sphereStart, const VECTOR& sphereEnd, float sphereRadius,
        const VECTOR& capsuleP0, const VECTOR& capsuleP1, float capsuleRadius,
        VECTOR* outHitNormal = nullptr,
        VECTOR* outHitPoint = nullptr
    );

    static void ClosestPointSegmentSegment(
        const VECTOR& p1, const VECTOR& q1,
        const VECTOR& p2, const VECTOR& q2,
        float& s, float& t,
        VECTOR& c1, VECTOR& c2
    );

    static VECTOR ClosestPointOnSegment(const VECTOR& p, const VECTOR& a, const VECTOR& b);
};
