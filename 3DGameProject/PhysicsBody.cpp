#include "PhysicsBody.h"
#include "Collider.h"
#include "SphereCollider.h"
#include "BoxCollider.h"
#include "CapsuleCollider.h"
#include "GameObject.h"
#include <cmath>
#include <algorithm>

// Apply inverse inertia tensor (full 3x3) to a world-space vector:
// I_world^{-1} * v = R * I_local^{-1} * (R^T * v)
// where R is the body's current rotation matrix.
VECTOR PhysicsBody::ApplyInverseInertia(const VECTOR& v) const noexcept {
    const float* I = InverseInertiaLocal3x3();
    if (!_owner) {
        // No owner ? apply local tensor directly
        return VGet(
            I[0]*v.x + I[1]*v.y + I[2]*v.z,
            I[3]*v.x + I[4]*v.y + I[5]*v.z,
            I[6]*v.x + I[7]*v.y + I[8]*v.z
        );
    }
    const VECTOR r0 = _owner->transform.Right();
    const VECTOR r1 = _owner->transform.Up();
    const VECTOR r2 = _owner->transform.Forward();
    // R^T * v  (rotate world vector into local frame)
    const float lx = r0.x*v.x + r0.y*v.y + r0.z*v.z;
    const float ly = r1.x*v.x + r1.y*v.y + r1.z*v.z;
    const float lz = r2.x*v.x + r2.y*v.y + r2.z*v.z;
    // I_local^{-1} * local  (full 3x3 multiply)
    const float sx = I[0]*lx + I[1]*ly + I[2]*lz;
    const float sy = I[3]*lx + I[4]*ly + I[5]*lz;
    const float sz = I[6]*lx + I[7]*ly + I[8]*lz;
    // R * scaled  (rotate back to world)
    return VGet(
        r0.x*sx + r1.x*sy + r2.x*sz,
        r0.y*sx + r1.y*sy + r2.y*sz,
        r0.z*sx + r1.z*sy + r2.z*sz
    );
}

// Compute inverse inertia tensor (full 3x3) from collider shape and current mass.
// For primitive shapes the result is diagonal; for compound shapes it may have
// off-diagonal terms (via parallel-axis theorem).
void PhysicsBody::ComputeInertia(Collider* collider) noexcept {
    // Zero out
    for (int i = 0; i < 9; ++i) _inverseInertiaLocal[i] = 0.0f;

    if (_mass <= 1e-6f || _isKinematic) return;

    const float m = _mass;
    float Ix = 1.0f, Iy = 1.0f, Iz = 1.0f;
    // Off-diagonal products of inertia (Ixy, Ixz, Iyz). Zero for aligned primitives.
    float Ixy = 0.0f, Ixz = 0.0f, Iyz = 0.0f;

    if (!collider) {
        const float I = 0.4f * m * 0.25f;
        Ix = Iy = Iz = I;
    }
    else if (collider->GetKind() == Collider::Kind::Sphere) {
        auto* sc = dynamic_cast<SphereCollider*>(collider);
        const float r = sc ? sc->GetRadius() : 0.5f;
        const float I = 0.4f * m * r * r;
        Ix = Iy = Iz = I;
    }
    else if (collider->GetKind() == Collider::Kind::Box) {
        auto* bc = dynamic_cast<BoxCollider*>(collider);
        if (bc) {
            const VECTOR he = bc->GetHalfExtents();
            const float w = 2.0f * he.x, h = 2.0f * he.y, d = 2.0f * he.z;
            const float k = m / 12.0f;
            Ix = k * (h*h + d*d);
            Iy = k * (w*w + d*d);
            Iz = k * (w*w + h*h);
        } else {
            const float I = 0.4f * m * 0.25f;
            Ix = Iy = Iz = I;
        }
    }
    else if (collider->GetKind() == Collider::Kind::Capsule) {
        auto* cc = dynamic_cast<CapsuleCollider*>(collider);
        if (cc) {
            const float r = cc->GetRadius();
            const VECTOR seg = VSub(cc->GetTop(), cc->GetBottom());
            const float hSeg = std::sqrt(seg.x*seg.x + seg.y*seg.y + seg.z*seg.z) * 0.5f;
            const float cylH = 2.0f * hSeg;
            const float Ia = 0.5f * m * r * r;
            const float Ip = m * (3.0f*r*r + cylH*cylH) / 12.0f;
            Ix = Ip; Iy = Ia; Iz = Ip;
        } else {
            const float I = 0.4f * m * 0.25f;
            Ix = Iy = Iz = I;
        }
    }
    else {
        const float I = 0.4f * m * 0.25f;
        Ix = Iy = Iz = I;
    }

    const float minI = 1e-6f;
    Ix = (std::max)(Ix, minI);
    Iy = (std::max)(Iy, minI);
    Iz = (std::max)(Iz, minI);

    // Build full inertia tensor (symmetric)
    //   [ Ix  -Ixy  -Ixz ]
    //   [-Ixy  Iy   -Iyz ]
    //   [-Ixz -Iyz   Iz  ]
    // For standard primitives off-diagonals are zero.
    // Invert: for diagonal matrix, inverse is just reciprocal of each diagonal.
    // For non-diagonal (compound shapes), use analytic 3x3 inverse.
    const float det =
        Ix * (Iy*Iz - Iyz*Iyz)
      + Ixy * (Iyz*Ixz - Ixy*Iz)
      + Ixz * (Ixy*Iyz - Iy*Ixz);

    if (std::fabs(det) < 1e-12f) {
        // Degenerate ? fallback to diagonal inverse
        _inverseInertiaLocal[0] = 1.0f / Ix;
        _inverseInertiaLocal[4] = 1.0f / Iy;
        _inverseInertiaLocal[8] = 1.0f / Iz;
        return;
    }

    const float invDet = 1.0f / det;
    _inverseInertiaLocal[0] =  (Iy*Iz  - Iyz*Iyz) * invDet;
    _inverseInertiaLocal[1] = -((-Ixy)*Iz - (-Iyz)*(-Ixz)) * invDet; // cofactor(0,1)
    _inverseInertiaLocal[2] =  ((-Ixy)*(-Iyz) - Iy*(-Ixz)) * invDet;
    _inverseInertiaLocal[3] = _inverseInertiaLocal[1]; // symmetric
    _inverseInertiaLocal[4] =  (Ix*Iz  - Ixz*Ixz) * invDet;
    _inverseInertiaLocal[5] = -(Ix*(-Iyz) - (-Ixy)*(-Ixz)) * invDet;
    _inverseInertiaLocal[6] = _inverseInertiaLocal[2];
    _inverseInertiaLocal[7] = _inverseInertiaLocal[5];
    _inverseInertiaLocal[8] =  (Ix*Iy  - Ixy*Ixy) * invDet;
}

// Apply physics material: set friction, restitution, damping from material.
// If density > 0 and a collider is provided, compute mass from volume * density.
void PhysicsBody::ApplyMaterial(const PhysicsMaterial& mat, Collider* collider) noexcept {
    _material = mat;
    _friction = mat.friction;
    _restitution = mat.restitution;
    _linearDamping = mat.linearDamping;
    _angularDamping = mat.angularDamping;

    // Auto-compute mass from density if density > 0 and collider is available
    if (mat.density > 0.0f && collider && !_isKinematic) {
        float volume = 0.0f;
        switch (collider->GetKind()) {
        case Collider::Kind::Sphere: {
            auto* sc = dynamic_cast<SphereCollider*>(collider);
            if (sc) {
                const float r = sc->GetRadius();
                volume = (4.0f / 3.0f) * 3.14159265f * r * r * r;
            }
            break;
        }
        case Collider::Kind::Box: {
            auto* bc = dynamic_cast<BoxCollider*>(collider);
            if (bc) {
                const VECTOR he = bc->GetHalfExtents();
                volume = 8.0f * he.x * he.y * he.z;
            }
            break;
        }
        case Collider::Kind::Capsule: {
            auto* cc = dynamic_cast<CapsuleCollider*>(collider);
            if (cc) {
                const float r = cc->GetRadius();
                const VECTOR seg = VSub(cc->GetTop(), cc->GetBottom());
                const float h = std::sqrt(seg.x * seg.x + seg.y * seg.y + seg.z * seg.z);
                // cylinder + two hemispheres = cylinder + sphere
                volume = 3.14159265f * r * r * h + (4.0f / 3.0f) * 3.14159265f * r * r * r;
            }
            break;
        }
        default:
            break;
        }
        if (volume > 1e-6f) {
            SetMass(mat.density * volume);
        }
    }

    // Recompute inertia if collider is provided
    if (collider) {
        ComputeInertia(collider);
    }
}
