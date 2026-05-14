#pragma once

#include "DxLib.h"
#include "Collider.h"
#include <vector>
#include <unordered_map>
#include <cstdint>

// Dynamic AABB Tree (Aランク: Spatial Hashより高速なBroadPhase)
// - 動的なオブジェクトが多い場合に有効
// - 挿入/削除/更新がO(log n)
// - クエリ（範囲検索）もO(log n + k) (k=結果数)
// - Bullet Physics / Box2D で使用されている手法

class DynamicAABBTree {
public:
    DynamicAABBTree() = default;
    ~DynamicAABBTree() = default;

    // ノードID (外部から参照するハンドル)
    using NodeId = int32_t;
    static constexpr NodeId NULL_NODE = -1;

    // AABBマージン (移動予測のためのパディング)
    static constexpr float kAABBExtension = 0.1f;
    static constexpr float kAABBMultiplier = 2.0f;

    // Colliderの登録/削除/更新
    NodeId Insert(Collider* collider, const AABB& aabb);
    void Remove(NodeId nodeId);
    bool Update(NodeId nodeId, const AABB& newAABB, const VECTOR& displacement);

    // クエリ: AABBと交差する全てのColliderを取得
    void Query(const AABB& queryAABB, std::vector<Collider*>& out) const;
    void QueryPairs(std::vector<std::pair<Collider*, Collider*>>& outPairs);

    // ツリーのリバランス (定期的に呼ぶと木の品質が向上)
    void Rebalance();

    // デバッグ情報
    int GetNodeCount() const { return static_cast<int>(_nodes.size()); }
    int GetHeight() const;
    float GetSurfaceArea() const;

    // 全クリア
    void Clear();

private:
    struct Node {
        AABB aabb;                  // このノードのAABB
        Collider* collider = nullptr; // リーフノードの場合のみ有効
        NodeId parent = NULL_NODE;
        NodeId left = NULL_NODE;
        NodeId right = NULL_NODE;
        int height = 0;             // 木の高さ (リーフ=0)

        bool IsLeaf() const { return left == NULL_NODE; }
    };

    std::vector<Node> _nodes;
    std::unordered_map<Collider*, NodeId> _colliderToNode;
    NodeId _root = NULL_NODE;
    NodeId _freeList = NULL_NODE;

    // 内部ヘルパー
    NodeId AllocateNode();
    void FreeNode(NodeId nodeId);
    void InsertLeaf(NodeId leaf);
    void RemoveLeaf(NodeId leaf);
    int ComputeHeight(NodeId nodeId) const;
    float ComputeSurfaceArea(NodeId nodeId) const;
    NodeId Balance(NodeId nodeId);
    NodeId RotateLeft(NodeId nodeId);
    NodeId RotateRight(NodeId nodeId);

    // AABBマージ/拡張
    AABB Union(const AABB& a, const AABB& b) const;
    AABB FattenAABB(const AABB& aabb, const VECTOR& displacement) const;
    float SurfaceArea(const AABB& aabb) const;
};
