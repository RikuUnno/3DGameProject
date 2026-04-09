#pragma once
#include "Ray.h"
#include "ColliderType.h"
#include "LayerMask.h"
#include <vector>
#include <cfloat>

class Collider;
class SphereCollider;
class BoxCollider;
class CapsuleCollider;

// Raycast: シーン内のコライダーに対するレイキャスト機能を提供する静的クラス。
// ColliderManager に登録済みのコライダー群を対象に、
// AABB ブロードフェーズ → 形状別ナローフェーズ の順で判定する。
class Raycast {
public:
    Raycast() = delete;

    // --- シーンレイキャスト ---

    // 最も近いヒットを1つ返す。ヒットしなければ hit==false。
    // layerMask: 対象レイヤー（ビットマスク）。デフォルトは全レイヤー。
    // maxDistance: 判定する最大距離。
    static RaycastHit Cast(
        const Ray& ray,
        float maxDistance = 1000.0f,
        int layerMask = mask::ALL
    );

    // ヒットした全コライダーを距離順で返す。
    static std::vector<RaycastHit> CastAll(
        const Ray& ray,
        float maxDistance = 1000.0f,
        int layerMask = mask::ALL
    );

    // --- 個別形状テスト（コライダー不要・純粋なジオメトリ判定） ---

    // Ray vs Sphere
    static bool TestSphere(
        const Ray& ray,
        const VECTOR& center,
        float radius,
        float maxDistance,
        float* outDistance,
        VECTOR* outPoint,
        VECTOR* outNormal
    );

    // Ray vs AABB（軸平行バウンディングボックス）
    static bool TestAABB(
        const Ray& ray,
        const AABB& aabb,
        float maxDistance,
        float* outDistance
    );

    // Ray vs OBB（BoxCollider）
    static bool TestBox(
        const Ray& ray,
        const BoxCollider* box,
        float maxDistance,
        float* outDistance,
        VECTOR* outPoint,
        VECTOR* outNormal
    );

    // Ray vs Capsule
    static bool TestCapsule(
        const Ray& ray,
        const VECTOR& bottom,
        const VECTOR& top,
        float radius,
        float maxDistance,
        float* outDistance,
        VECTOR* outPoint,
        VECTOR* outNormal
    );
};
