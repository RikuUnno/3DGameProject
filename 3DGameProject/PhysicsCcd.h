#pragma once

#include "DxLib.h"
#include "PhysicsBody.h"
#include "Collider.h"
#include <vector>

// TOI (Time Of Impact) 計算結果
// 連続衝突検出で、移動物体が最初に衝突する時刻と位置を表す
struct TOIResult {
    bool hit = false;                           // 衝突したかどうか
    float toi = 1.0f;                           // 衝突時刻 (0.0 = 開始時点、1.0 = 終了時点)
    VECTOR hitNormal = VGet(0, 1, 0);           // 衝突面の法線ベクトル (ワールド座標)
    VECTOR hitPoint = VGet(0, 0, 0);            // 衝突点 (ワールド座標)
    Collider* hitCollider = nullptr;            // 衝突したコライダー
};

// PhysicsCcd - 連続衝突検出 (CCD: Continuous Collision Detection)
// 高速移動するオブジェクトがすり抜けないよう、TOI (衝突予定時刻) を計算する
class PhysicsCcd {
public:
	// 球形コライダーの TOI 計算
    static TOIResult ComputeTOI_Sphere(
		const VECTOR& startPos,                             // 移動物体の中心の開始位置
		const VECTOR& endPos,                               // 移動物体の中心の終了位置
		float radius,                                       // 球の半径
		Collider* movingCollider,                           // 移動する球のコライダー
		const std::vector<Collider*>& staticColliders,      // 衝突判定対象の静的コライダー群
		float allowedPenetration = 0.0f                     // 許容されるペネトレーション深度 (0 以上の値で、ペネトレーションがこの値を超えると CCD が自動トリガーされる)
    );

	// 箱形コライダーの TOI 計算
    static TOIResult ComputeTOI_Box(
		const VECTOR& startPos,                          // 移動物体の中心の開始位置
		const VECTOR& endPos,                            // 移動物体の中心の終了位置 
		const VECTOR& halfExtents,                       // 箱の半サイズ (ローカル座標系での半幅、半高さ、半奥行き)
		const VECTOR& axisX,                             // 箱の向き軸 X (ローカル座標系での右方向ベクトル、ワールド座標系で正規化されていること)
		const VECTOR& axisY,                             // 箱の向き軸 Y (ローカル座標系での上方向ベクトル、ワールド座標系で正規化されていること)
		const VECTOR& axisZ,                             // 箱の向き軸 Z (ローカル座標系での前方向ベクトル、ワールド座標系で正規化されていること)
		Collider* movingCollider,                        // 移動する箱のコライダー
		const std::vector<Collider*>& staticColliders,   // 衝突判定対象の静的コライダー群
		float allowedPenetration = 0.0f                  // 許容されるペネトレーション深度 (0 以上の値で、ペネトレーションがこの値を超えると CCD が自動トリガーされる
    );

	// カプセル形コライダーの TOI 計算
    static TOIResult ComputeTOI_Capsule(
		const VECTOR& startPos,                             // 移動物体の中心の開始位置
		const VECTOR& endPos,                               // 移動物体の中心の終了位置
		float radius,                                       // カプセルの半径
		float halfHeight,                                   // カプセルの半高さ (中心から線分の端までの距離)
		const VECTOR& axis,                                 // カプセルの向き軸 (ローカル座標系での線分の方向ベクトル、ワールド座標系で正規化されていること)
		Collider* movingCollider,                           // 移動するカプセルのコライダー
		const std::vector<Collider*>& staticColliders,      // 衝突判定対象の静的コライダー群
		float allowedPenetration = 0.0f                     // 許容されるペネトレーション深度 (0 以上の値で、ペネトレーションがこの値を超えると CCD が自動トリガーされる
    );

	// 連続衝突検出の処理 (PhysicsManager::Update 内で呼び出し)
    static void ProcessCCD(
		PhysicsBody* body,                              // 衝突検出対象の物理ボディ
		Collider* collider,                             // 衝突検出対象のコライダー
		float stepDt,                                   // 現在の物理ステップの時間 (秒)
		const std::vector<Collider*>& allColliders      // 衝突判定対象の全コライダー群 (静的・動的を含む)
    );

private:
	// 移動する球と静止する球の衝突判定 (TOI 計算)
    static float ComputeTOI_SphereSphere(
		const VECTOR& centerA0, const VECTOR& centerA1, float radiusA,  // centerB0, centerB1: 球の中心の開始・終了位置
		const VECTOR& centerB0, const VECTOR& centerB1, float radiusB,  // outHitNormal: 衝突法線の出力先
		VECTOR* outHitNormal = nullptr,                                 // outHitPoint: 衝突点の出力先
		VECTOR* outHitPoint = nullptr                                   // 戻り値: 衝突時刻 (0.0 ～ 1.0)
    );

	// 移動する球と静止する箱の衝突判定 (TOI 計算)
    static float ComputeTOI_SphereBox(                              
		const VECTOR& sphereStart, const VECTOR& sphereEnd, float radius,           // boxCenter, boxHalfExtents: 箱の中心と半サイズ
		const VECTOR& boxCenter, const VECTOR& boxHalfExtents,                      // boxAxisX, boxAxisY, boxAxisZ: 箱の向き軸 (ローカル座標系での右/上/前方向ベクトル、ワールド座標系で正規化されていること)
		const VECTOR& boxAxisX, const VECTOR& boxAxisY, const VECTOR& boxAxisZ,     // outHitNormal: 衝突法線の出力先
		VECTOR* outHitNormal = nullptr,                                             // outHitPoint: 衝突点の出力先
		VECTOR* outHitPoint = nullptr                                               // 戻り値: 衝突時刻 (0.0 ～ 1.0)
    );

	// 移動する球と静止するカプセルの衝突判定 (TOI 計算)
    static float ComputeTOI_SphereCapsule(	
		const VECTOR& sphereStart, const VECTOR& sphereEnd, float sphereRadius,		// capsuleP0, capsuleP1: カプセルの線分の両端点
		const VECTOR& capsuleP0, const VECTOR& capsuleP1, float capsuleRadius,		// capsuleAxis: カプセルの向き軸 (ローカル座標系での線分の方向ベクトル、ワールド座標系で正規化されていること)
		VECTOR* outHitNormal = nullptr,												// outHitPoint: 衝突点の出力先
		VECTOR* outHitPoint = nullptr												// 戻り値: 衝突時刻 (0.0 ～ 1.0)
    );

	// 線分と線分の最近点を求める
    static void ClosestPointSegmentSegment(
		const VECTOR& p1, const VECTOR& q1,	// p1, q1: 線分1の両端点
		const VECTOR& p2, const VECTOR& q2,	// p2, q2: 線分2の両端点
		float& s, float& t,					// s, t: 線分1、線分2上の最近点の位置を表すパラメータ (0.0 ～ 1.0)
		VECTOR& c1, VECTOR& c2				// c1, c2: 線分1、線分2上の最近点 (ワールド座標)
    );

	// 点と線分の最近点を求める
    static VECTOR ClosestPointOnSegment(const VECTOR& p, const VECTOR& a, const VECTOR& b);
};
