#include "SphereCollider.h"
#include "GameObject.h"
#include "DxLib.h"
#include <algorithm>

SphereCollider::SphereCollider() {
	// デフォルト値
	_sphere.center = VGet(0,0,0);
	_sphere.radius =0.5f;
	UpdateShape();
}

SphereCollider::~SphereCollider() = default;

void SphereCollider::UpdateShape() {
	// owner がある場合、_sphere.center は「ワールド中心」ではなく
	// 「owner基準のローカルオフセット」として扱う。
	// これにより親回転・親スケール・親移動をすべて反映できる。
	if (owner) {
		const VECTOR worldScale = owner->transform.WorldScale();
		// Sphere は非一様スケールを厳密には表現できないため、
		// 最大スケールを採用して抜けにくさを優先する。
		const float maxScale = (std::max)((std::max)(std::fabs(worldScale.x), std::fabs(worldScale.y)), std::fabs(worldScale.z));
		_center = owner->transform.TransformPoint(_sphere.center);
		_radius = _sphere.radius * maxScale;
	}
	else {
		_center = _sphere.center;
		_radius = _sphere.radius;
	}

	_aabb.center = _center;
	_aabb.min = VGet(_center.x - _radius, _center.y - _radius, _center.z - _radius);
	_aabb.max = VGet(_center.x + _radius, _center.y + _radius, _center.z + _radius);
}

void SphereCollider::DrawDebug() {
	const unsigned int defaultCol = isTrigger ? GetColor(255,220,80) : GetColor(80,200,200);
	const unsigned int col = DebugColor() != 0 ? DebugColor() : defaultCol;
	DrawSphere3D(_center, _radius,20, col, col, FALSE);
	DrawSphere3D(_center,0.05f,8, col, col, TRUE);
}

void SphereCollider::DrawDebugAABB() {
	const unsigned int col = GetColor(120,120,120);
	const VECTOR mn = _aabb.min;
	const VECTOR mx = _aabb.max;

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

