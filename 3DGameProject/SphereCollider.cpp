#include "SphereCollider.h"
#include "GameObject.h"
#include "DxLib.h"
#include <algorithm>
#include <cmath>

namespace {
	// 大円の分割数（線分の本数）
	constexpr int kCircleSegments = 32;
	// 2π
	constexpr float k2Pi = DX_PI_F * 2.0f;

	// 球面上に回転済みの大円（Great Circle）をワイヤーフレームで描画する。
	// center : 球の中心（ワールド）
	// radius : 球の半径（ワールド）
	// axisU  : 大円が広がる平面の第1ベクトル（正規化済み）
	// axisV  : 大円が広がる平面の第2ベクトル（正規化済み）
	// col    : 描画色
	// 数学: 点 P(θ) = center + radius * (cos(θ)*axisU + sin(θ)*axisV)
	inline void DrawGreatCircle(
		const VECTOR& center, float radius,
		const VECTOR& axisU, const VECTOR& axisV,
		unsigned int col) noexcept
	{
		const float step = k2Pi / static_cast<float>(kCircleSegments);
		VECTOR prev = VAdd(center, VScale(axisU, radius));
		for (int i = 1; i <= kCircleSegments; ++i) {
			const float theta = step * static_cast<float>(i);
			const float c = std::cos(theta);
			const float s = std::sin(theta);
			const VECTOR cur = VAdd(center,
				VAdd(VScale(axisU, radius * c), VScale(axisV, radius * s)));
			DrawLine3D(prev, cur, col);
			prev = cur;
		}
	}
}

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

// デバッグ描画。
// ワイヤーフレーム球に加え、ownerの回転を視認できるよう
// 3軸方向のリング（大円）と軸線を描画する。
//  赤 = Right(X)方向, 緑 = Up(Y)方向, 青 = Forward(Z)方向
void SphereCollider::DrawDebug() {
	const unsigned int defaultCol = isTrigger ? GetColor(255,220,80) : GetColor(80,200,200);
	const unsigned int col = DebugColor() != 0 ? DebugColor() : defaultCol;

	// ワイヤーフレーム球本体
	DrawSphere3D(_center, _radius,20, col, col, FALSE);
	// 中心点
	DrawSphere3D(_center,0.05f,8, col, col, TRUE);

	// --- 回転視覚化 ---
	// ownerがなければ既定の軸を使用
	VECTOR axisRight   = VGet(1,0,0); // X軸方向
	VECTOR axisUp      = VGet(0,1,0); // Y軸方向
	VECTOR axisForward = VGet(0,0,1); // Z軸方向

	if (owner) {
		// 親回転込みのワールド方向ベクトルを取得
		axisRight   = owner->transform.Right();
		axisUp      = owner->transform.Up();
		axisForward = owner->transform.Forward();
	}

	// 軸カラー（RGB = XYZ 対応、一般的な3Dエディタの慣例に合わせる）
	const unsigned int colR = GetColor(220,40,40);  // 赤: Right(X)
	const unsigned int colG = GetColor(40,220,40);   // 緑: Up(Y)
	const unsigned int colB = GetColor(40,80,220);   // 青: Forward(Z)

	// --- 大円（Great Circle）を3つ描画 ---
	// 各大円は「軸に直交する平面」上の円。
	// YZ平面のリング → Right軸(X)に直交 → axisUp, axisForward で張る
	DrawGreatCircle(_center, _radius, axisUp, axisForward, colR);
	// XZ平面のリング → Up軸(Y)に直交 → axisRight, axisForward で張る
	DrawGreatCircle(_center, _radius, axisRight, axisForward, colG);
	// XY平面のリング → Forward軸(Z)に直交 → axisRight, axisUp で張る
	DrawGreatCircle(_center, _radius, axisRight, axisUp, colB);

	// --- 軸線を描画（中心から球面より少し飛び出す方向線）---
	// 各軸の正方向のみ描画し、矢印のように向きを示す
	const float axisLen = _radius * 1.15f; // 球面より少し飛び出す長さ
	DrawLine3D(_center, VAdd(_center, VScale(axisRight,   axisLen)), colR);
	DrawLine3D(_center, VAdd(_center, VScale(axisUp,      axisLen)), colG);
	DrawLine3D(_center, VAdd(_center, VScale(axisForward, axisLen)), colB);
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

