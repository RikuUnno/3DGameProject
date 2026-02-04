// ColliderType.h
// オブジェクトの構造を定義するコライダータイプ
#pragma once
#include "DXLib.h"

// 回転軸アリのBoxCollider
struct BoxCollider {
    VECTOR center;       // 中心点(ワールド)
    VECTOR halfExtents;  // 各軸の半径(ローカルサイズの半分)
    VECTOR axisX;        // 向きX
	VECTOR axisY;		 // 向きY
	VECTOR axisZ;		 // 向きZ
};

// 回転軸ありのSphereCollider
struct SphereCollider
{
	VECTOR center; // 中心点(ワールド)
	float radius;  // 半径
};

// 回転軸アリのCapsuleCollider
struct CapsuleCollider
{
	VECTOR center; // 中心点(ワールド)
	VECTOR bottom; // カプセルの端点A
	VECTOR top; // カプセルの端点B
	float radius;  // 半径
};

// 回転軸ナシのAABBCollider
struct AABB
{
	VECTOR center; // 中心点(ワールド)
	VECTOR min; // 最小点
	VECTOR max; // 最大点
};