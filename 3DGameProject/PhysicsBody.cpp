#include "PhysicsBody.h"
#include "PhysicsBody.h"
#include "Collider.h"
#include "SphereCollider.h"
#include "BoxCollider.h"
#include "CapsuleCollider.h"
#include <cmath>
#include <algorithm>

// Compute inverse inertia tensor diagonal from collider shape and current mass.
// Formulas assume uniform density solid shapes.
//   Sphere:  I = (2/5) * m * r^2  (all axes equal)
//   Box:     Ix = (1/12) * m * (h^2 + d^2), etc.  (full-size w,h,d = 2*halfExtent)
//   Capsule: approximate as cylinder + hemisphere caps
//   Default: sphere of radius 0.5
void PhysicsBody::ComputeInertia(Collider* collider) noexcept {
    if (_mass <= 1e-6f || _isKinematic) {
        _inverseInertiaDiag = VGet(0, 0, 0);
        return;
    }

    const float m = _mass;
    float Ix = 1.0f, Iy = 1.0f, Iz = 1.0f;

    if (!collider) {
        // No collider: treat as unit sphere
        const float I = 0.4f * m * 0.25f; // (2/5)*m*0.5^2
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
            const float w = 2.0f * he.x;
            const float h = 2.0f * he.y;
            const float d = 2.0f * he.z;
            const float k = m / 12.0f;
            Ix = k * (h * h + d * d);
            Iy = k * (w * w + d * d);
            Iz = k * (w * w + h * h);
        }
        else {
            const float I = 0.4f * m * 0.25f;
            Ix = Iy = Iz = I;
        }
    }
    else if (collider->GetKind() == Collider::Kind::Capsule) {
        auto* cc = dynamic_cast<CapsuleCollider*>(collider);
        if (cc) {
            const float r = cc->GetRadius();
            const VECTOR bot = cc->GetBottom();
            const VECTOR top = cc->GetTop();
            const VECTOR seg = VSub(top, bot);
            const float hSeg = std::sqrt(seg.x * seg.x + seg.y * seg.y + seg.z * seg.z) * 0.5f;
            // Approximate as cylinder (height=2*hSeg, radius=r) for simplicity
            const float cylH = 2.0f * hSeg;
            // Along capsule axis (Y-like): I_axis = (1/2)*m*r^2
            const float Ia = 0.5f * m * r * r;
            // Perpendicular axes: I_perp = (1/12)*m*(3*r^2 + h^2)
            const float Ip = m * (3.0f * r * r + cylH * cylH) / 12.0f;
            Ix = Ip;
            Iy = Ia;
            Iz = Ip;
        }
        else {
            const float I = 0.4f * m * 0.25f;
            Ix = Iy = Iz = I;
        }
    }
    else {
        // Fallback: unit sphere
        const float I = 0.4f * m * 0.25f;
        Ix = Iy = Iz = I;
    }

    // Clamp to avoid division by zero
    const float minI = 1e-6f;
    Ix = (std::max)(Ix, minI);
    Iy = (std::max)(Iy, minI);
    Iz = (std::max)(Iz, minI);

    _inverseInertiaDiag = VGet(1.0f / Ix, 1.0f / Iy, 1.0f / Iz);
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
