#pragma once
#include "Collider.h"
#include "ColliderType.h"

// HalfPlaneCollider
// - Infinite half-space collider defined by a plane normal and offset.
// - The "solid" side is the negative half-space (n . x < d).
// - Typically used as an infinite ground plane replacement.
// - Always static (infinite AABB for broad-phase, but cheap narrow-phase).
class HalfPlaneCollider : public Collider {
public:
    HalfPlaneCollider() {
        _plane.normal = VGet(0, 1, 0);
        _plane.d = 0.0f;
        _aabb.min = VGet(-1e6f, -1e6f, -1e6f);
        _aabb.max = VGet(1e6f, 1e6f, 1e6f);
        _aabb.center = VGet(0, 0, 0);
    }

    void SetPlane(const VECTOR& normal, float d) noexcept {
        _plane.normal = normal;
        _plane.d = d;
    }

    const HalfPlane& GetPlane() const noexcept { return _plane; }

    // --- Collider overrides ---
    Kind GetKind() const override { return Kind::HalfPlane; }
    const AABB& GetAABB() const override { return _aabb; }
    VECTOR GetCenter() const override { return VScale(_plane.normal, _plane.d); }
    void UpdateShape() override {} // infinite ? nothing to update
    void SetAABB() override {}

    void DrawDebug() override;
    void DrawDebugAABB() override {}

private:
    HalfPlane _plane{};
    AABB _aabb{};
};
