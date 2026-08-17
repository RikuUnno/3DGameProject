#pragma once
#include "Collider.h"
#include "ColliderType.h"
#include <vector>
#include <memory>
#include <algorithm>

// CompoundCollider
// 複合コライダー（複数の子Colliderをまとめて1つのColliderとして扱う）
// - 子Colliderのイベントは owner に送るか、CompoundCollider自身で受け取るかを選択可能
// - 子Colliderのイベントを owner の親GameObject にも伝えるかを選択可能
class CompoundCollider : public Collider {
public:
    CompoundCollider() = default;

	// 子Colliderを追加する（所有者はCompoundColliderのownerに設定される）
    void AddChild(std::unique_ptr<Collider> child) {
        if (child) {
            child->owner = this->owner;
            _children.push_back(std::move(child));
        }
        RebuildAABB();
        RebuildBVH();
    }

	// 子Colliderを削除する（所有者はnullptrに設定される）
    size_t ChildCount() const noexcept { return _children.size(); }
	// 子Colliderを取得する（インデックスが範囲外の場合はnullptrを返す）
    Collider* GetChild(size_t index) const noexcept {
        return (index < _children.size()) ? _children[index].get() : nullptr;
    }

	// 子Colliderの一覧を取得する（const参照）
    const std::vector<std::unique_ptr<Collider>>& Children() const noexcept { return _children; }

	// QueryBVH: 再帰的にBVHを探索して、queryと重なる子Colliderのインデックスをコールバックする
    template<typename Func>
    void QueryOverlapping(const AABB& query, Func&& callback) const {
        if (_bvhNodes.empty()) {
			// BVHが構築されていない場合は全ての子Colliderをチェック
            for (size_t i = 0; i < _children.size(); ++i) {
                callback(i);
            }
            return;
        }
        QueryBVH(0, query, callback);
    }

	// QueryBVH: 再帰的にBVHを探索して、queryと重なる子Colliderのインデックスをコールバックする
    Kind GetKind() const override { return Kind::Compound; }
    const AABB& GetAABB() const override { return _aabb; }
    VECTOR GetCenter() const override { return _aabb.center; }

	// UpdateShape: 子ColliderのAABBを集約してCompoundColliderのAABBを更新
    void UpdateShape() override {
        for (auto& child : _children) {
            if (child) {
                child->owner = this->owner;
                child->UpdateShape();
            }
        }
        RebuildAABB();
		// RebuildAABB: 子ColliderのAABBを集約してCompoundColliderのAABBを更新
		RebuildBVH();
    }

	// SetAABB: CompoundColliderのAABBを再構築
    void SetAABB() override { RebuildAABB(); }

	// デバッグ描画
    void DrawDebug() override {
        for (auto& child : _children) {
            if (child) child->DrawDebug();
        }
    }
	// デバッグ描画（AABBのみ）
    void DrawDebugAABB() override {
        for (auto& child : _children) {
            if (child) child->DrawDebugAABB();
        }
    }

private:
	// 子Colliderの所有権を保持する
    struct BVHNode {
		AABB aabb{};            // このノードのAABB
		int leftOrChild = -1;   // 左の子ノードのインデックス（葉ノードの場合は子Colliderのインデックス）
		int right = -1;         // 右の子ノードのインデックス（葉ノードの場合は-1）
		bool isLeaf = false;    // 葉ノードかどうか
    };

	// 子Colliderの所有権を保持する
    void RebuildAABB() {
        if (_children.empty()) {
            _aabb = {};
            return;
        }
        VECTOR mn = VGet(1e6f, 1e6f, 1e6f);
        VECTOR mx = VGet(-1e6f, -1e6f, -1e6f);
        for (auto& child : _children) {
            if (!child) continue;
            const AABB& ca = child->GetAABB();
            if (ca.min.x < mn.x) mn.x = ca.min.x;
            if (ca.min.y < mn.y) mn.y = ca.min.y;
            if (ca.min.z < mn.z) mn.z = ca.min.z;
            if (ca.max.x > mx.x) mx.x = ca.max.x;
            if (ca.max.y > mx.y) mx.y = ca.max.y;
            if (ca.max.z > mx.z) mx.z = ca.max.z;
        }
        _aabb.min = mn;
        _aabb.max = mx;
        _aabb.center = VScale(VAdd(mn, mx), 0.5f);
    }

	// BVH構築（子ColliderのAABBを使ってBVHを構築）
    void RebuildBVH() {
        _bvhNodes.clear();
        if (_children.size() <= 2) return; // Not worth building BVH for <=2 children

        std::vector<int> indices(_children.size());
        for (int i = 0; i < static_cast<int>(_children.size()); ++i) indices[i] = i;
        BuildBVHNode(indices, 0, static_cast<int>(_children.size()));
    }

	// BuildBVHNode: 再帰的にBVHノードを構築する
    int BuildBVHNode(std::vector<int>& indices, int begin, int end) {
        BVHNode node{};
        VECTOR mn = VGet(1e6f, 1e6f, 1e6f);
        VECTOR mx = VGet(-1e6f, -1e6f, -1e6f);
        for (int i = begin; i < end; ++i) {
            const AABB& ca = _children[indices[i]]->GetAABB();
            if (ca.min.x < mn.x) mn.x = ca.min.x;
            if (ca.min.y < mn.y) mn.y = ca.min.y;
            if (ca.min.z < mn.z) mn.z = ca.min.z;
            if (ca.max.x > mx.x) mx.x = ca.max.x;
            if (ca.max.y > mx.y) mx.y = ca.max.y;
            if (ca.max.z > mx.z) mx.z = ca.max.z;
        }
        node.aabb.min = mn;
        node.aabb.max = mx;
        node.aabb.center = VScale(VAdd(mn, mx), 0.5f);

        if (end - begin == 1) {
            node.isLeaf = true;
            node.leftOrChild = indices[begin];
            node.right = -1;
            int idx = static_cast<int>(_bvhNodes.size());
            _bvhNodes.push_back(node);
            return idx;
        }

        const float dx = mx.x - mn.x;
        const float dy = mx.y - mn.y;
        const float dz = mx.z - mn.z;
        int axis = 0;
        if (dy > dx && dy > dz) axis = 1;
        else if (dz > dx) axis = 2;

        auto GetAxisCenter = [&](int childIdx) -> float {
            const AABB& ca = _children[childIdx]->GetAABB();
            if (axis == 0) return ca.center.x;
            if (axis == 1) return ca.center.y;
            return ca.center.z;
        };

        int mid = begin + (end - begin) / 2;
        std::nth_element(indices.begin() + begin, indices.begin() + mid, indices.begin() + end,
            [&](int a, int b) { return GetAxisCenter(a) < GetAxisCenter(b); });

        int thisIdx = static_cast<int>(_bvhNodes.size());
        _bvhNodes.push_back(node); //

        int leftIdx = BuildBVHNode(indices, begin, mid);
        int rightIdx = BuildBVHNode(indices, mid, end);

        _bvhNodes[thisIdx].leftOrChild = leftIdx;
        _bvhNodes[thisIdx].right = rightIdx;
        _bvhNodes[thisIdx].isLeaf = false;
        return thisIdx;
    }

	// AABB同士の重なり判定
    static bool AABBOverlap(const AABB& a, const AABB& b) noexcept {
        return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
               (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
               (a.min.z <= b.max.z && a.max.z >= b.min.z);
    }

	// QueryBVH: 再帰的にBVHを探索して、queryと重なる子Colliderのインデックスをコールバックする
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

	// 子Colliderの所有権を保持する
	std::vector<std::unique_ptr<Collider>> _children;   // 子Colliderの所有権を保持する
	std::vector<BVHNode> _bvhNodes;                     // BVHノードの配列
	AABB _aabb{};                                       // CompoundColliderのAABB
};

// ※BVHとは、Bounding Volume Hierarchyの略で、複数のオブジェクトを階層的にまとめたバウンディングボリュームの構造です。
// これにより、衝突判定の際に効率的に候補を絞り込むことができます。
// ※BVH構築は、子Colliderが少ない場合（2個以下）は行わず、全ての子Colliderを線形探索するようにしています。