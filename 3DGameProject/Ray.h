#pragma once
#include "DxLib.h"

class Collider;
class GameObject;

// Ray: 原点と方向で定義される半直線
struct Ray {
    VECTOR origin    = VGet(0, 0, 0); // レイの始点（ワールド空間）
    VECTOR direction = VGet(0, 0, 1); // レイの方向（正規化済み）
};

// RaycastHit: レイキャストの衝突結果
struct RaycastHit {
    bool     hit       = false;           // ヒットしたか
    float    distance  = 0.0f;            // origin からヒット点までの距離
    VECTOR   point     = VGet(0, 0, 0);   // ヒット点（ワールド空間）
    VECTOR   normal    = VGet(0, 0, 0);   // ヒット面の法線（ワールド空間）
    Collider*   collider = nullptr;       // ヒットした Collider
    GameObject* gameObject = nullptr;     // ヒットした GameObject（collider->owner）
};
