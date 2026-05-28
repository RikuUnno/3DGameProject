// MeshCollider.h
// 三角形メッシュコライダー（ステージ等の任意形状向け）
// - 既存の BoxCollider / CapsuleCollider / SphereCollider と同じインターフェース
//   （Collider 派生）で動作するため、ColliderManager の登録/更新フローはそのまま利用可能。
// - 内部に三角形配列とBVH（Step2以降で実装）を保持する。
// - 静的ステージ向け（isStatic=true）と、動くプラットフォーム向け（isStatic=false）の
//   両方を考慮した設計。動的時は UpdateShape() で BVH を更新する。
//
// 実装状況（段階導入）:
//   Step 1: 骨組みのみ（ビルドが通る最小実装。判定は未実装）
//   Step 2: BVH 構築 + デバッグ描画
//   Step 3: Sphere   vs Mesh 判定 + 押し出し
//   Step 4: Capsule  vs Mesh 判定 + 押し出し
//   Step 5: Box(OBB) vs Mesh 判定 + 押し出し
//   Step 6: DxLib MV1 モデルからの構築 API
#pragma once
#include "Collider.h"
#include "ColliderType.h"
#include <vector>

class MeshCollider : public Collider {
public:
    MeshCollider();
    virtual ~MeshCollider();

public:
    // --- Collider overrides ---
    Kind GetKind() const override { return Kind::Mesh; }
    const AABB& GetAABB() const override { return _aabb; }
    VECTOR GetCenter() const override { return _aabb.center; }
    void UpdateShape() override;
    void SetAABB() override;

    void DrawDebug() override;
    void DrawDebugAABB() override;

public:
    // --- 構築 API（Step 2 以降で本実装。現状は受け取って保持するのみ） ---

    // 汎用：頂点配列＋インデックス配列からメッシュを構築する。
    // indices は 3 個ずつ三角形を成す（CCW 推奨）。
    // owner の Transform を考慮してワールド空間に変換し、BVH を再構築する。
    void BuildFromVertices(const std::vector<VECTOR>& verts,
        const std::vector<int>& indices);

    // 将来追加予定: DxLib MV1 モデルからの直接構築（Step 6）
    // void BuildFromMV1(int mv1Handle);

public:
    // --- 設定 ---
    // 静的フラグ：true の場合、UpdateShape() で再変換しない（初回のみ）。
    // ステージなど不動オブジェクトでは true 推奨。
    // 動くプラットフォーム等では false にして毎フレーム再構築する。
    bool isStatic = true;

    // 三角形数の取得（デバッグ/プロファイル用）
    size_t TriangleCount() const noexcept { return _trianglesWorld.size(); }

private:
    // ローカル空間の三角形（owner の Transform を適用する前のデータ）
    // 動的更新（isStatic=false）時に毎フレームこちらからワールド空間へ変換する。
    std::vector<Triangle> _trianglesLocal;

    // ワールド空間の三角形（実際の判定で使う側）
    std::vector<Triangle> _trianglesWorld;

    // ブロードフェーズ用の全体AABB
    AABB _aabb{};

    // 静的構築済みフラグ（isStatic=true の場合、初回のみ構築する判定に使用）
    bool _builtOnce = false;

    // BVH 等の加速構造は Step 2 以降で追加予定（実装ファイル側に閉じる）

private:
    // 内部ヘルパ：_trianglesLocal から _trianglesWorld と _aabb を再計算する。
    void RecomputeWorld();
};
