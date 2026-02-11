#include "SphereCollider.h"
#include "GameObject.h"

SphereCollider::SphereCollider() {
	// デフォルト値
	sphere_.center = VGet(0,0,0);
	sphere_.radius =0.5f;
	UpdateShape();
}

SphereCollider::~SphereCollider() = default;

void SphereCollider::UpdateShape() {
	// owner Transformから中心を算出（Transformが無い場合は保持値を使用）
	if (owner) {
		sphere_.center = owner->transform.WorldPosition();
	}

	aabb_.center = sphere_.center;
	aabb_.min = VGet(sphere_.center.x - sphere_.radius, sphere_.center.y - sphere_.radius, sphere_.center.z - sphere_.radius);
	aabb_.max = VGet(sphere_.center.x + sphere_.radius, sphere_.center.y + sphere_.radius, sphere_.center.z + sphere_.radius);
}

void SphereCollider::DrawDebug() {
	const unsigned int col = isTrigger ? GetColor(255,220,80) : GetColor(80,200,200);
	DrawSphere3D(sphere_.center, sphere_.radius,20, col, col, FALSE);
	DrawSphere3D(sphere_.center,0.05f,8, col, col, TRUE);
}

void SphereCollider::DrawDebugAABB() {
	const unsigned int col = GetColor(120,120,120);
	const VECTOR mn = aabb_.min;
	const VECTOR mx = aabb_.max;

	const VECTOR p000 = VGet(mn.x,mn.y,mn.z);
	const VECTOR p001 = VGet(mn.x,mn.y,mx.z);
	const VECTOR p010 = VGet(mn.x,mx.y,mn.z);
	const VECTOR p011 = VGet(mn.x,mx.y,mx.z);
	const VECTOR p100 = VGet(mx.x,mn.y,mn.z);
	const VECTOR p101 = VGet(mx.x,mn.y,mx.z);
	const VECTOR p110 = VGet(mx.x,mx.y,mn.z);
	const VECTOR p111 = VGet(mx.x,mx.y,mx.z);

	DrawLine3D(p000, p001, col);
	DrawLine3D(p001, p011, col);
	DrawLine3D(p011, p010, col);
	DrawLine3D(p010, p000, col);

	DrawLine3D(p100, p101, col);
	DrawLine3D(p101, p111, col);
	DrawLine3D(p111, p110, col);
	DrawLine3D(p110, p100, col);

	DrawLine3D(p000, p100, col);
	DrawLine3D(p001, p101, col);
	DrawLine3D(p010, p110, col);
	DrawLine3D(p011, p111, col);
}

