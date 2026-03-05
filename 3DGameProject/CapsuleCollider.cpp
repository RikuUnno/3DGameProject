#include "CapsuleCollider.h"

#include <algorithm>

#include "GameObject.h"
#include "DxLib.h"

CapsuleCollider::CapsuleCollider() {
	cap_.radius =0.3f;
	// デフォルトは上下1.0のカプセル（ローカル）
	cap_.bottom = VGet(0,-0.5f,0);
	cap_.top = VGet(0,0.5f,0);
	cap_.center = VGet(0,0,0);
	UpdateShape();
}

CapsuleCollider::~CapsuleCollider() = default;

void CapsuleCollider::UpdateShape() {
	// owner Transformから center を算出
	VECTOR center = cap_.center;
	VECTOR up = VGet(0,1,0);
	if (owner) {
		center = owner->transform.WorldPosition();
		up = owner->transform.Up();
	}

	// bottom/top は「中心 + up * offset」としてワールドへ
	// cap_.bottom/top はローカルの上下オフセット量（y差分）を参照
	const float halfLen = (cap_.top.y - cap_.bottom.y) *0.5f;
	const VECTOR upN = VNorm(up);
	cap_.center = center;
	cap_.bottom = VSub(center, VScale(upN, halfLen));
	cap_.top = VAdd(center, VScale(upN, halfLen));

	const float r = cap_.radius;
	const float minX = (std::min)(cap_.bottom.x, cap_.top.x) - r;
	const float minY = (std::min)(cap_.bottom.y, cap_.top.y) - r;
	const float minZ = (std::min)(cap_.bottom.z, cap_.top.z) - r;
	const float maxX = (std::max)(cap_.bottom.x, cap_.top.x) + r;
	const float maxY = (std::max)(cap_.bottom.y, cap_.top.y) + r;
	const float maxZ = (std::max)(cap_.bottom.z, cap_.top.z) + r;

	_aabb.min = VGet(minX,minY,minZ);
	_aabb.max = VGet(maxX,maxY,maxZ);
	_aabb.center = VScale(VAdd(_aabb.min, _aabb.max),0.5f);
}

void CapsuleCollider::DrawDebug() {
	unsigned int col = isTrigger ? GetColor(255,220,80) : GetColor(80,200,200);
	if (DebugColor() !=0) {
		col = DebugColor();
	}

	// 軸（中心線）
	DrawLine3D(cap_.bottom, cap_.top, col);

	//端の球
	DrawSphere3D(cap_.bottom, cap_.radius,16, col, col, FALSE);
	DrawSphere3D(cap_.top, cap_.radius,16, col, col, FALSE);

	// --- 外枠（球と球をつなぐ線：円筒部の母線） ---
	VECTOR axis = VSub(cap_.top, cap_.bottom);
	axis = VNorm(axis);

	VECTOR ref = (std::fabs(axis.y) <0.99f) ? VGet(0,1,0) : VGet(1,0,0);
	VECTOR right = VCross(axis, ref);
	right = VNorm(right);
	VECTOR forward = VCross(right, axis);
	forward = VNorm(forward);

	const float r = cap_.radius;
	const VECTOR p = cap_.bottom;
	const VECTOR q = cap_.top;

	constexpr int kSideLines =16;
	const float pi = DX_PI_F;
	for (int i =0; i < kSideLines; ++i) {
		const float theta = (2.0f*pi) * (float)i / (float)kSideLines;
		const VECTOR dir = VAdd(VScale(right, std::cos(theta)), VScale(forward, std::sin(theta)));
		const VECTOR off = VScale(dir, r);
		DrawLine3D(VAdd(p, off), VAdd(q, off), col);
	}

	DrawSphere3D(cap_.center,0.05f,8, col, col, TRUE);
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