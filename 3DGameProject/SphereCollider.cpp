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
	// Transform がある場合はワールド変換を考慮してワールド中心と半径を更新
	if (owner) {	// Transform がある場合はワールド変換を考慮してワールド中心と半径を更新
		const VECTOR worldScale = owner->transform.WorldScale();																	// 親スケールを含んだワールドスケール
		const float maxScale = (std::max)((std::max)(std::fabs(worldScale.x), std::fabs(worldScale.y)), std::fabs(worldScale.z));	// 最大スケールを取得
		_center = owner->transform.TransformPoint(_sphere.center);																	// ローカル中心点をワールドに変換
		_radius = _sphere.radius * maxScale;																						// ワールド半径を計算
	}
	else {			// Transform がない場合はローカル設定をそのままワールドにコピー
		_center = _sphere.center;	// ワールド中心をコピー
		_radius = _sphere.radius;	// ワールド半径をコピー
	}

	// AABB を更新
	_aabb.center = _center;																// AABB の中心点を設定
	_aabb.min = VGet(_center.x - _radius, _center.y - _radius, _center.z - _radius);	// AABB の最小点を設定
	_aabb.max = VGet(_center.x + _radius, _center.y + _radius, _center.z + _radius);	// AABB の最大点を設定
}

// デバッグ描画（線状の形状）
void SphereCollider::DrawDebug() {
	_debugColor = (_debugColor != 0) ? _debugColor : GetColor(255, 255, 255);	// デフォルト色は白色

	// ワイヤーフレーム球本体
	// 粗い分割数だとポリゴン弦が内側に収縮して実半径より小さく見え、
	// 床に沈んでいるように見えるため 32 分割にする (最大誤差 ~0.2%)。
	DrawSphere3D(_center, _radius, 32, _debugColor, _debugColor, FALSE);
	// 中心点
	DrawSphere3D(_center,0.05f,8, _debugColor, _debugColor, TRUE);

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

// デバッグ描画（AABBのみ）
void SphereCollider::DrawDebugAABB() {
	_debugColor = (_debugColor != 0) ? _debugColor : GetColor(128, 128, 128);	// デフォルト色はグレー	
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

	DrawLine3D(p000, p001, _debugColor);
	DrawLine3D(p001, p011, _debugColor);
	DrawLine3D(p011, p010, _debugColor);
	DrawLine3D(p010, p000, _debugColor);

	DrawLine3D(p100, p101, _debugColor);
	DrawLine3D(p101, p111, _debugColor);
	DrawLine3D(p111, p110, _debugColor);
	DrawLine3D(p110, p100, _debugColor);

	DrawLine3D(p000, p100, _debugColor);
	DrawLine3D(p001, p101, _debugColor);
	DrawLine3D(p010, p110, _debugColor);
	DrawLine3D(p011, p111, _debugColor);
}

// デバッグ描画（DXLibのプリミティブ描画）
void SphereCollider::DrawPrimitive() {
	_debugColor = (_debugColor != 0) ? _debugColor : GetColor(255, 255, 255);	// デフォルト色は白色

	// 球本体
	DrawSphere3D(_center, _radius, 20, _debugColor, _debugColor, TRUE);

	// --- 回転視覚化 ---
	// ownerがなければ既定の軸を使用
	VECTOR axisRight = VGet(1, 0, 0); // X軸方向
	VECTOR axisUp = VGet(0, 1, 0); // Y軸方向
	VECTOR axisForward = VGet(0, 0, 1); // Z軸方向

	if (owner) {
		// 親回転込みのワールド方向ベクトルを取得
		axisRight = owner->transform.Right();
		axisUp = owner->transform.Up();
		axisForward = owner->transform.Forward();
	}

	// 軸カラー（RGB = XYZ 対応、一般的な3Dエディタの慣例に合わせる）
	const unsigned int colR = GetColor(220, 40, 40);  // 赤: Right(X)
	const unsigned int colG = GetColor(40, 220, 40);   // 緑: Up(Y)
	const unsigned int colB = GetColor(40, 80, 220);   // 青: Forward(Z)

	// --- 大円（Great Circle）を3つ描画 ---
	// 各大円は「軸に直交する平面」上の円。
	// YZ平面のリング → Right軸(X)に直交 → axisUp, axisForward で張る
	DrawGreatCircle(_center, _radius, axisUp, axisForward, colR);
	// XZ平面のリング → Up軸(Y)に直交 → axisRight, axisForward で張る
	DrawGreatCircle(_center, _radius, axisRight, axisForward, colG);
	// XY平面のリング → Forward軸(Z)に直交 → axisRight, axisUp で張る
	DrawGreatCircle(_center, _radius, axisRight, axisUp, colB);

}
