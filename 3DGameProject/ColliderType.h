// ColliderType.h
// オブジェクトの構造を定義するコライダータイプ
#pragma once
#include "DXLib.h"

// 回転軸アリのBoxCollider
struct Box {
    VECTOR center;       // 中心点(ワールド)
    VECTOR halfExtents;  // 各軸の半径(ローカルサイズの半分)
    VECTOR axisX;        // 向きX
	VECTOR axisY;		 // 向きY
	VECTOR axisZ;		 // 向きZ
};

// 回転軸ありのSphereCollider
struct Sphere
{
	VECTOR center; // 中心点(ワールド)
	float radius;  // 半径
};

// 回転軸アリのCapsuleCollider
struct Capsule
{
	VECTOR center; // 中心点(ワールド)
	VECTOR bottom; // カプセルの端点A
	VECTOR top; // カプセルの端点B
	float radius;  // 半径
};

// 回転軸ナシのAABBCollider
struct AABB
{
	VECTOR center;
	VECTOR min;
	VECTOR max;
};

// Half-plane (half-space collider): n . x <= d
struct HalfPlane
{
	VECTOR normal; // unit normal (world)
	float  d;      // signed distance from origin (n . p = d)
};

// 三角形（MeshCollider 構成要素）
// ワールド空間の頂点座標と、事前計算した法線/AABBを保持する。
// normal は (v1-v0) x (v2-v0) を正規化した値（CCW想定）。
struct Triangle
{
	VECTOR v0;
	VECTOR v1;
	VECTOR v2;
	VECTOR normal; // 事前計算した正規化法線（ワールド空間）
	AABB   aabb;   // BVH/ブロードフェーズ用の三角形AABB
};

// 三角形メッシュ（MeshCollider のジオメトリ）
// 別途BVHは MeshCollider 側で管理する（コライダ実装ファイルに閉じ込めるため）。
struct TriangleMesh
{
	// ワールド空間の三角形列。Static の場合は初回構築時のみ更新される。
	// Dynamic の場合は UpdateShape() で毎フレーム再計算する。
	// （実体は MeshCollider が保持する。本構造体は将来の API 用に共通化）
	int dummy = 0; // 現状は前方互換用のプレースホルダ
};
