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