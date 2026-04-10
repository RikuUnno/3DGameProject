#include "HalfPlaneCollider.h"
#include "DxLib.h"
#include <cmath>

void HalfPlaneCollider::DrawDebug() {
    // Draw a large quad to visualize the plane
    const VECTOR center = VScale(_plane.normal, _plane.d);
    // Build tangent basis
    VECTOR t1, t2;
    if (std::fabs(_plane.normal.x) < 0.9f) {
        t1 = VCross(_plane.normal, VGet(1, 0, 0));
    } else {
        t1 = VCross(_plane.normal, VGet(0, 1, 0));
    }
    const float len = VSize(t1);
    if (len > 1e-6f) t1 = VScale(t1, 1.0f / len);
    t2 = VCross(_plane.normal, t1);

    const float size = 20.0f;
    const unsigned int color = DebugColor() ? DebugColor() : GetColor(200, 200, 200);
    const VECTOR p0 = VAdd(center, VAdd(VScale(t1, -size), VScale(t2, -size)));
    const VECTOR p1 = VAdd(center, VAdd(VScale(t1,  size), VScale(t2, -size)));
    const VECTOR p2 = VAdd(center, VAdd(VScale(t1,  size), VScale(t2,  size)));
    const VECTOR p3 = VAdd(center, VAdd(VScale(t1, -size), VScale(t2,  size)));
    DrawLine3D(p0, p1, color);
    DrawLine3D(p1, p2, color);
    DrawLine3D(p2, p3, color);
    DrawLine3D(p3, p0, color);
    DrawLine3D(p0, p2, color);
    DrawLine3D(p1, p3, color);
}
