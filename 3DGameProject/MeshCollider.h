// MeshCollider.h
// MeshCollider.h
// 三角形メッシュコライダー（ステージ等の任意形状向け）。
// - Collider 派生として BoxCollider / CapsuleCollider / SphereCollider と
//   同じインターフェースを持ち、ColliderManager の登録・更新
//   ・ディスパッチスキームにそのまま乗る。
// - 三角形配列と BVH（フラット二分木）を保持し、broad-phase での
//   候補絞り込みに QueryOverlapping() を提供する。
// - 静的ステージ向け（isStatic=true / 初回のみ構築）と
//   動くプラットフォーム向け（isStatic=false / 毎フレーム再構築）を
//   両方サポートする。
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
    // --- 構築 API ---

    // 頂点配列 + インデックス配列からメッシュを構築する。
    // indices は 3 個ずつ三角形を成す（CCW 推奨）。
    // owner の Transform でワールド変換し、BVH を再構築する。
    void BuildFromVertices(const std::vector<VECTOR>& verts,
        const std::vector<int>& indices);

    // DxLib MV1 モデルから直接構築する。
    // - mv1Handle : MV1LoadModel で取得したハンドル
    // - frameIndex: -1 でモデル全体、0 以上で特定フレーム（メッシュ）のみ
    // 三角形はローカル座標で取り込み、owner の Transform でワールド化する。
    // 取り込み後は MV1 ハンドルを破棄しても問題ない（コライダ側でコピー保持）。
    // 戻り値: 三角形が 1 つ以上生成できれば true。
    bool BuildFromMV1(int mv1Handle, int frameIndex = -1);

public:
    // --- 設定 ---

    // 静的フラグ。true なら UpdateShape() で再変換しない（初回構築時のみ）。
    // ステージ等の不動オブジェクトでは true 推奨。動くプラットフォーム等では false。
    bool isStatic = true;

    // 三角形数（デバッグ / プロファイル用）
    size_t TriangleCount() const noexcept { return _trianglesWorld.size(); }

    // ワールド空間の三角形配列への参照（narrow-phase で使用）
    const std::vector<Triangle>& Triangles() const noexcept { return _trianglesWorld; }

public:
    // --- BVH 問い合わせ API ---

    // query AABB と重なる三角形の index を callback(size_t triIndex) で通知する。
    // narrow-phase（Sphere / Capsule / OBB vs Mesh）の候補絞り込みに使用する。
    // BVH 未構築（三角形 <= 1）の場合は全走査にフォールバックする。
    template<typename Func>
    void QueryOverlapping(const AABB& query, Func&& callback) const {
        if (_bvhNodes.empty()) {
            for (size_t i = 0; i < _trianglesWorld.size(); ++i) {
                if (AABBOverlap(_trianglesWorld[i].aabb, query)) {
                    callback(i);
                }
            }
            return;
        }
        QueryBVH(0, query, std::forward<Func>(callback));
    }

    // query AABB に重なる三角形のみワイヤ描画する（巨大ステージ向け）。
    void DrawDebugInAABB(const AABB& query);

private:
    // BVH ノード（フラット二分木）
    struct BVHNode {
        AABB aabb{};
        int leftOrChild = -1; // leaf: 三角形 index / branch: 左ノード index
        int right = -1;       // branch: 右ノード index / leaf: -1
        bool isLeaf = false;
    };

    // ローカル空間の三角形（owner の Transform 適用前のソース）。
    // isStatic=false 時は毎フレームこちらからワールド空間へ再変換する。
    std::vector<Triangle> _trianglesLocal;

    // ワールド空間の三角形（判定で使う側）
    std::vector<Triangle> _trianglesWorld;

    // BVH（_trianglesWorld のインデックスを参照するフラットノード列）
    std::vector<BVHNode> _bvhNodes;

    // broad-phase 用の全体 AABB
    AABB _aabb{};

    // 初回構築済みフラグ（isStatic=true の二度目以降の更新をスキップするための判定）
    bool _builtOnce = false;

private:
    // _trianglesLocal -> _trianglesWorld / _aabb / BVH を再計算する。
    void RecomputeWorld();

    // _trianglesWorld を入力に BVH を構築する。
    void RebuildBVH();
    int  BuildBVHNode(std::vector<int>& indices, int begin, int end);

    static bool AABBOverlap(const AABB& a, const AABB& b) noexcept {
        return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
            (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
            (a.min.z <= b.max.z && a.max.z >= b.min.z);
    }

    // BVH 探索本体
    template<typename Func>
    void QueryBVH(int nodeIdx, const AABB& query, Func&& callback) const {
        if (nodeIdx < 0 || nodeIdx >= static_cast<int>(_bvhNodes.size())) return;
        const BVHNode& node = _bvhNodes[nodeIdx];
        if (!AABBOverlap(node.aabb, query)) return;
        if (node.isLeaf) {
            callback(static_cast<size_t>(node.leftOrChild));
            return;
        }
        QueryBVH(node.leftOrChild, query, callback);
        if (node.right >= 0) QueryBVH(node.right, query, callback);
    }
};
