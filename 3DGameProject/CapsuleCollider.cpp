#include "CapsuleCollider.h"

#include <algorithm>

#include "GameObject.h"
#include "DxLib.h"

CapsuleCollider::CapsuleCollider() {
	_cap.radius =0.3f;
	_cap.bottom = VGet(0,-0.5f,0);
	_cap.top = VGet(0,0.5f,0);
	_cap.center = VGet(0,0,0);
	UpdateShape();
}

CapsuleCollider::~CapsuleCollider() = default;

void CapsuleCollider::UpdateShape() {
	VECTOR center = _cap.center;
	VECTOR up = VGet(0,1,0);
	if (owner) {
		center = owner->transform.WorldPosition();
		up = owner->transform.Up();
	}

	const float halfLen = (_cap.top.y - _cap.bottom.y) *0.5f;
	const VECTOR upN = VNorm(up);
	_cap.center = center;
	_cap.bottom = VSub(center, VScale(upN, halfLen));
	_cap.top = VAdd(center, VScale(upN, halfLen));

	const float r = _cap.radius;
	const float minX = (std::min)(_cap.bottom.x, _cap.top.x) - r;
	const float minY = (std::min)(_cap.bottom.y, _cap.top.y) - r;
	const float minZ = (std::min)(_cap.bottom.z, _cap.top.z) - r;
	const float maxX = (std::max)(_cap.bottom.x, _cap.top.x) + r;
	const float maxY = (std::max)(_cap.bottom.y, _cap.top.y) + r;
	const float maxZ = (std::max)(_cap.bottom.z, _cap.top.z) + r;

	_aabb.min = VGet(minX,minY,minZ);
	_aabb.max = VGet(maxX,maxY,maxZ);
	_aabb.center = VScale(VAdd(_aabb.min, _aabb.max),0.5f);
}

void CapsuleCollider::DrawDebug() {
	unsigned int col = isTrigger ? GetColor(255,220,80) : GetColor(80,200,200);
	if (DebugColor() !=0) {
		col = DebugColor();
	}

	DrawLine3D(_cap.bottom, _cap.top, col);
	DrawSphere3D(_cap.bottom, _cap.radius,16, col, col, FALSE);
	DrawSphere3D(_cap.top, _cap.radius,16, col, col, FALSE);

	VECTOR axis = VSub(_cap.top, _cap.bottom);
	axis = VNorm(axis);

	VECTOR ref = (std::fabs(axis.y) <0.99f) ? VGet(0,1,0) : VGet(1,0,0);
	VECTOR right = VCross(axis, ref);
	right = VNorm(right);
	VECTOR forward = VCross(right, axis);
	forward = VNorm(forward);

	const float r = _cap.radius;
	const VECTOR p = _cap.bottom;
	const VECTOR q = _cap.top;

	constexpr int kSideLines =16;
	const float pi = DX_PI_F;
	for (int i =0; i < kSideLines; ++i) {
		const float theta = (2.0f*pi) * (float)i / (float)kSideLines;
		const VECTOR dir = VAdd(VScale(right, std::cos(theta)), VScale(forward, std::sin(theta)));
		const VECTOR off = VScale(dir, r);
		DrawLine3D(VAdd(p, off), VAdd(q, off), col);
	}

	DrawSphere3D(_cap.center,0.05f,8, col, col, TRUE);
}

void CapsuleCollider::DrawDebugAABB() {
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