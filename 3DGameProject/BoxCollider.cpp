#include "BoxCollider.h"

#include <algorithm>
#include <cmath>

#include "GameObject.h"
#include "DxLib.h"

namespace { // ユーティリティ関数群
	// ベクトルの各要素絶対値
	inline VECTOR AbsVec(const VECTOR& v) {
		return VGet(std::fabs(v.x), std::fabs(v.y), std::fabs(v.z));
	}
	// 3つのベクトル和
	inline VECTOR Add3(const VECTOR& a, const VECTOR& b, const VECTOR& c) {
		return VAdd(VAdd(a, b), c);
	}
	// 3次元ドット積
	inline float Dot3Local(const VECTOR& a, const VECTOR& b) {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}
	// ベクトル長
	inline float LenLocal(const VECTOR& v) {
		return std::sqrt(Dot3Local(v, v));
	}
	// ベクトル正規化（ゼロベクトル対策付き）
	inline VECTOR SafeNormalize(const VECTOR& v, const VECTOR& fallback) {
		const float len = LenLocal(v);
		if (len >1e-6f) return VScale(v,1.0f / len);
		return fallback;
	}
}

// コンストラクタ
BoxCollider::BoxCollider() : Collider() {
	_box.center = VGet(0, 0, 0);				// デフォルトは原点中心
	_box.halfExtents = VGet(0.5f, 0.5f, 0.5f);	// デフォルトは 1x1x1 の立方体
	_box.axisX = VGet(1, 0, 0);					// デフォルトはワールド軸に沿った向き
	_box.axisY = VGet(0,1,0);					// デフォルトはワールド軸に沿った向き
	_box.axisZ = VGet(0,0,1);					// デフォルトはワールド軸に沿った向き
	UpdateShape();								// 初期化時に AABB を計算
}

// デストラクタ
BoxCollider::~BoxCollider() = default;

// Shape 更新（Transform変更時に呼び出す）
void BoxCollider::UpdateShape() {
	if (owner) {	// Transform がある場合はワールド変換を考慮して OBB を更新
		const VECTOR worldScale = owner->transform.WorldScale();
		_center = owner->transform.TransformPoint(_box.center);		// ローカル中心点をワールドに変換
		_axisX = owner->transform.Right();							// ワールド軸ベクトルを取得
		_axisY = owner->transform.Up();								// ワールド軸ベクトルを取得
		_axisZ = owner->transform.Forward();						// ワールド軸ベクトルを取得
		// ワールドスケールを考慮して半サイズを計算（絶対値を取ることで反転スケールにも対応）
		_halfExtents = VGet(
			std::fabs(_box.halfExtents.x * worldScale.x),
			std::fabs(_box.halfExtents.y * worldScale.y),
			std::fabs(_box.halfExtents.z * worldScale.z)
		);
	}
	else {		// Transform がない場合は OBB をそのまま使用
		_center = _box.center;				// ローカル中心点をそのまま使用
		_axisX = _box.axisX;				//	ローカル軸ベクトルをそのまま使用
		_axisY = _box.axisY;				// 	ローカル軸ベクトルをそのまま使用
		_axisZ = _box.axisZ;				// 	ローカル軸ベクトルをそのまま使用
		_halfExtents = _box.halfExtents;	// 	ローカル半サイズをそのまま使用
	}

	// 軸ベクトルを正規化（Transform のスケールが反映されている場合は長さが変わるため）
	_axisX = SafeNormalize(_axisX, VGet(1, 0, 0));
	_axisY = SafeNormalize(_axisY, VGet(0, 1, 0));
	_axisZ = SafeNormalize(_axisZ, VGet(0, 0, 1));

	// OBB を包むAABBは、各ワールド軸ベクトルの絶対値成分を使って求める。
	const VECTOR ex = VScale(AbsVec(_axisX), _halfExtents.x);
	const VECTOR ey = VScale(AbsVec(_axisY), _halfExtents.y);
	const VECTOR ez = VScale(AbsVec(_axisZ), _halfExtents.z);
	const VECTOR ext = Add3(ex, ey, ez);

	// AABBを更新
	_aabb.center = _center;				// AABBの中心はOBBの中心と同じ
	_aabb.min = VSub(_center, ext);		// AABBの最小点は中心からextを引いた点
	_aabb.max = VAdd(_center, ext);		// AABBの最大点は中心からextを足した点
}

// デバッグ描画
void BoxCollider::DrawDebug() {
	_debugColor = (_debugColor != 0) ? _debugColor : GetColor(255, 255, 255);	// デフォルト色は白色	

	// OBBの8頂点を計算するためのラムダ関数
	auto Corner = [&](float sx, float sy, float sz) {
		VECTOR p = _center;
		p = VAdd(p, VScale(_axisX, _halfExtents.x * sx));
		p = VAdd(p, VScale(_axisY, _halfExtents.y * sy));
		p = VAdd(p, VScale(_axisZ, _halfExtents.z * sz));
		return p;
	};

	// OBBの8頂点を計算
	const VECTOR p000 = Corner(-1,-1,-1);
	const VECTOR p001 = Corner(-1,-1,1);
	const VECTOR p010 = Corner(-1,1,-1);
	const VECTOR p011 = Corner(-1,1,1);
	const VECTOR p100 = Corner(1,-1,-1);
	const VECTOR p101 = Corner(1,-1,1);
	const VECTOR p110 = Corner(1,1,-1);
	const VECTOR p111 = Corner(1,1,1);

	// OBBの12辺を描画
	// 下面の辺
	DrawLine3D(p000, p001, _debugColor);
	DrawLine3D(p001, p011, _debugColor);
	DrawLine3D(p011, p010, _debugColor);
	DrawLine3D(p010, p000, _debugColor);
	// 上面の辺
	DrawLine3D(p100, p101, _debugColor);
	DrawLine3D(p101, p111, _debugColor);
	DrawLine3D(p111, p110, _debugColor);
	DrawLine3D(p110, p100, _debugColor);
	// 側面の辺
	DrawLine3D(p000, p100, _debugColor);
	DrawLine3D(p001, p101, _debugColor);
	DrawLine3D(p010, p110, _debugColor);
	DrawLine3D(p011, p111, _debugColor);

	// 中心点を描画
	DrawSphere3D(_center,0.05f,8, _debugColor, _debugColor, TRUE);
}

// デバッグ描画（AABBのみ）
void BoxCollider::DrawDebugAABB() {
	_debugColor = (_debugColor != 0) ? _debugColor : GetColor(128, 128, 128);	// デフォルト色はグレー
	// AABBの8頂点を計算
	const VECTOR mn = _aabb.min;
	const VECTOR mx = _aabb.max;

	// AABBの8頂点を計算
	const VECTOR p000 = VGet(mn.x,mn.y,mn.z);
	const VECTOR p001 = VGet(mn.x,mn.y,mx.z);
	const VECTOR p010 = VGet(mn.x,mx.y,mn.z);
	const VECTOR p011 = VGet(mn.x,mx.y,mx.z);
	const VECTOR p100 = VGet(mx.x,mn.y,mn.z);
	const VECTOR p101 = VGet(mx.x,mn.y,mx.z);
	const VECTOR p110 = VGet(mx.x,mx.y,mn.z);
	const VECTOR p111 = VGet(mx.x,mx.y,mx.z);

	// AABBの12辺を描画
	// 下面の辺
	DrawLine3D(p000, p001, _debugColor);
	DrawLine3D(p001, p011, _debugColor);
	DrawLine3D(p011, p010, _debugColor);
	DrawLine3D(p010, p000, _debugColor);
	// 上面の辺
	DrawLine3D(p100, p101, _debugColor);
	DrawLine3D(p101, p111, _debugColor);
	DrawLine3D(p111, p110, _debugColor);
	DrawLine3D(p110, p100, _debugColor);
	// 側面の辺
	DrawLine3D(p000, p100, _debugColor);
	DrawLine3D(p001, p101, _debugColor);
	DrawLine3D(p010, p110, _debugColor);
	DrawLine3D(p011, p111, _debugColor);
}

// デバッグ描画（DXLibのプリミティブ描画）
void BoxCollider::DrawPrimitive() 
{
	_debugColor = (_debugColor != 0) ? _debugColor : GetColor(255, 255, 255);	// デフォルト色は白色
	const unsigned int obbColor = GetColor(0, 0, 0);							// デフォルト色は黒

	// OBBの8頂点を計算するためのラムダ関数
	auto Corner = [&](float sx, float sy, float sz) {
		VECTOR p = _center;
		p = VAdd(p, VScale(_axisX, _halfExtents.x * sx));
		p = VAdd(p, VScale(_axisY, _halfExtents.y * sy));
		p = VAdd(p, VScale(_axisZ, _halfExtents.z * sz));
		return p;
		};

	// OBBの8頂点を計算
	const VECTOR p000 = Corner(-1, -1, -1);
	const VECTOR p001 = Corner(-1, -1, 1);
	const VECTOR p010 = Corner(-1, 1, -1);
	const VECTOR p011 = Corner(-1, 1, 1);
	const VECTOR p100 = Corner(1, -1, -1);
	const VECTOR p101 = Corner(1, -1, 1);
	const VECTOR p110 = Corner(1, 1, -1);
	const VECTOR p111 = Corner(1, 1, 1);

	// OBBの12辺を描画（三角形で描画することで面としても見えるようにする）
	// 下面の辺
	DrawTriangle3D(p000, p001, p011, _debugColor, TRUE);
	DrawTriangle3D(p000, p011, p010, _debugColor, TRUE);
	// 上面の辺
	DrawTriangle3D(p100, p101, p111, _debugColor, TRUE);
	DrawTriangle3D(p100, p111, p110, _debugColor, TRUE);
	// 側面の辺(左右)
	DrawTriangle3D(p000, p100, p101, _debugColor, TRUE);
	DrawTriangle3D(p000, p101, p001, _debugColor, TRUE);
	DrawTriangle3D(p010, p110, p111, _debugColor, TRUE);
	DrawTriangle3D(p010, p111, p011, _debugColor, TRUE);
	// 側面の辺(前後)
	DrawTriangle3D(p001, p101, p111, _debugColor, TRUE);
	DrawTriangle3D(p001, p111, p011, _debugColor, TRUE);
	DrawTriangle3D(p000, p100, p110, _debugColor, TRUE);
	DrawTriangle3D(p000, p110, p010, _debugColor, TRUE);

	// OBBの12辺を描画（線で輪郭を描く）
	// 下面の辺
	DrawLine3D(p000, p001, obbColor);
	DrawLine3D(p001, p011, obbColor);
	DrawLine3D(p011, p010, obbColor);
	DrawLine3D(p010, p000, obbColor);
	// 上面の辺
	DrawLine3D(p100, p101, obbColor);
	DrawLine3D(p101, p111, obbColor);
	DrawLine3D(p111, p110, obbColor);
	DrawLine3D(p110, p100, obbColor);
	// 側面の辺
	DrawLine3D(p000, p100, obbColor);
	DrawLine3D(p001, p101, obbColor);
	DrawLine3D(p010, p110, obbColor);
	DrawLine3D(p011, p111, obbColor);

	// 中心点は描画しない（面として描画されるため）
}