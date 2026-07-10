#include "CapsuleCollider.h"

#include <algorithm>
#include <cmath>

#include "GameObject.h"
#include "DxLib.h"

// コンストラクタ
CapsuleCollider::CapsuleCollider() {
	_cap.radius = 0.3f;					// デフォルト半径
	_cap.bottom = VGet(0, -0.5f, 0);	// デフォルトは原点中心で上下に1の長さ
	_cap.top = VGet(0, 0.5f, 0);		// デフォルトは原点中心で上下に1の長さ
	_cap.center = VGet(0, 0, 0);		// デフォルトは原点中心
	UpdateShape();						// 初期化時に AABB を計算
}

// デストラクタ
CapsuleCollider::~CapsuleCollider() = default;

// Shape 更新（Transform変更時に呼び出す）
void CapsuleCollider::UpdateShape() {
	if (owner) {	// Transform がある場合はワールド変換を考慮してワールド端点・ワールド半径を更新
		const VECTOR worldScale = owner->transform.WorldScale();																	// 親スケールを含んだワールドスケール
		const float maxScale = (std::max)((std::max)(std::fabs(worldScale.x), std::fabs(worldScale.y)), std::fabs(worldScale.z));	// 最大スケールを取得
		_bottom = owner->transform.TransformPoint(_cap.bottom);																		// ローカル端点Aをワールドに変換
		_top = owner->transform.TransformPoint(_cap.top);																			// ローカル端点Bをワールドに変換
		_center = VScale(VAdd(_bottom, _top), 0.5f);																				// ワールド中心を計算
		_radius = _cap.radius * maxScale;																							// ワールド半径を計算
	}
	else {			// Transform がない場合はローカル設定をそのままワールドにコピー
		_center = _cap.center;		// ワールド中心をコピー
		_bottom = _cap.bottom;		// ワールド端点Aをコピー
		_top = _cap.top;			// ワールド端点Bをコピー
		_radius = _cap.radius;		// ワールド半径をコピー
	}

	// AABB を更新
	const float minX = (std::min)(_bottom.x, _top.x) - _radius;
	const float minY = (std::min)(_bottom.y, _top.y) - _radius;
	const float minZ = (std::min)(_bottom.z, _top.z) - _radius;
	const float maxX = (std::max)(_bottom.x, _top.x) + _radius;
	const float maxY = (std::max)(_bottom.y, _top.y) + _radius;
	const float maxZ = (std::max)(_bottom.z, _top.z) + _radius;

	// AABB の min/max/center を設定
	_aabb.min = VGet(minX, minY, minZ);						// AABB の最小点を設定
	_aabb.max = VGet(maxX, maxY, maxZ);						// AABB の最大点を設定
	_aabb.center = VScale(VAdd(_aabb.min, _aabb.max),0.5f);	// AABB の中心点を設定
}

// デバッグ描画（線状の形状）
void CapsuleCollider::DrawDebug() {
	_debugColor = (_debugColor != 0) ? _debugColor : GetColor(255, 255, 255);	// デフォルト色は白色

	// カプセルの端点と半径を使ってデバッグ描画
	DrawLine3D(_bottom, _top, _debugColor);									// カプセルの軸線を描画
	DrawSphere3D(_bottom, _radius, 16, _debugColor, _debugColor, FALSE);	// カプセルの底面球を描画
	DrawSphere3D(_top, _radius, 16, _debugColor, _debugColor, FALSE);		// カプセルの上面球を描画

	// カプセルの側面を描画するために、軸方向のベクトルと垂直な基準ベクトルを計算
	VECTOR axis = VSub(_top, _bottom);	// カプセルの軸方向ベクトルを計算
	axis = VNorm(axis);					// 軸方向ベクトルを正規化

	// 軸方向ベクトルと垂直な基準ベクトルを選択（軸がほぼY軸に近い場合はX軸を使用）
	VECTOR ref = (std::fabs(axis.y) < 0.99f) ? VGet(0, 1, 0) : VGet(1, 0, 0);	// 軸方向ベクトルと垂直な基準ベクトルを選択
	VECTOR right = VCross(axis, ref);											// 軸方向ベクトルと基準ベクトルの外積を計算して垂直なベクトルを求める
	right = VNorm(right);														// 垂直なベクトルを正規化
	VECTOR forward = VCross(right, axis);										// もう一つの垂直なベクトルを計算
	forward = VNorm(forward);													// もう一つの垂直なベクトルを正規化

	// 側面の円周を描画するために、円周上の点を計算して線分で描画
	const float r = _radius;	// カプセルの半径
	const VECTOR p = _bottom;	// カプセルの底面の中心点
	const VECTOR q = _top;		// カプセルの上面の中心点

	// 側面の円周を描画するために、円周上の点を計算して線分で描画
	constexpr int kSideLines = 16;		// 側面の円周を描画するための分割数
	const float pi = DX_PI_F;			// 円周率
	for (int i = 0; i < kSideLines; ++i) {	// 側面の円周を描画するためのループ
		const float theta = (2.0f*pi) * (float)i / (float)kSideLines;
		const VECTOR dir = VAdd(VScale(right, std::cos(theta)), VScale(forward, std::sin(theta)));
		const VECTOR off = VScale(dir, r);
		DrawLine3D(VAdd(p, off), VAdd(q, off), _debugColor);
	}

	// 中心点を描画
	DrawSphere3D(_center,0.05f,8, _debugColor, _debugColor, TRUE);
}

// デバッグ描画（AABBのみ）
void CapsuleCollider::DrawDebugAABB() {
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
void CapsuleCollider::DrawPrimitive() {
	_debugColor = (_debugColor != 0) ? _debugColor : GetColor(255, 255, 255);	// デフォルト色は白色

	// カプセルの端点と半径を使ってデバッグ描画
	DrawCapsule3D(_bottom, _top, _radius, 16, _debugColor, _debugColor, TRUE);	// カプセルを描画
}