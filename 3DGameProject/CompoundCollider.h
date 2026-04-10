#pragma once
#include "Collider.h"
#include "ColliderType.h"
#include <vector>
#include <memory>
#include <algorithm>

// CompoundCollider
// - Composite collider that holds multiple child colliders.
// - Maintains a simple binary BVH for O(log n) broad-phase culling of children.
// - Narrow-phase dispatches to each overlapping child individually.
// - Children share the compound's owner and transform.
class CompoundCollider : public Collider {
public:
    CompoundCollider() = default;

    // Add a child collider. Ownership is transferred.
    void AddChild(std::unique_ptr<Collider> child) {
        if (child) {
            child->owner = this->owner;
            _children.push_back(std::move(child));
        }
        RebuildAABB();
        RebuildBVH();
    }

    size_t ChildCount() const noexcept { return _children.size(); }
    Collider* GetChild(size_t index) const noexcept {
        return (index < _children.size()) ? _children[index].get() : nullptr;
    }

    const std::vector<std::unique_ptr<Collider>>& Children() const noexcept { return _children; }

    // Query children overlapping a given AABB (for broad-phase culling).
    // Calls callback(childIndex) for each overlapping child.
    template<typename Func>
    void QueryOverlapping(const AABB& query, Func&& callback) const {
        if (_bvhNodes.empty()) {
            // Fallback: test all children
            for (size_t i = 0; i < _children.size(); ++i) {
                callback(i);
            }
            return;
        }
        QueryBVH(0, query, callback);
    }

    // --- Collider overrides ---
    Kind GetKind() const override { return Kind::Compound; }
    const AABB& GetAABB() const override { return _aabb; }
    VECTOR GetCenter() const override { return _aabb.center; }

    void UpdateShape() override {
        for (auto& child : _children) {
            if (child) {
                child->owner = this->owner;
                child->UpdateShape();
            }
        }
        RebuildAABB();
        // BVH is rebuilt only on AddChild; UpdateShape just updates the AABB
    }

    void SetAABB() override { RebuildAABB(); }

    void DrawDebug() override {
        for (auto& child : _children) {
            if (child) child->DrawDebug();
        }
    }

    void DrawDebugAABB() override {
        for (auto& child : _children) {
            if (child) child->DrawDebugAABB();
        }
    }

private:
    // BVH node (binary tree stored in a flat array)
    struct BVHNode {
        AABB aabb{};
        int leftOrChild = -1;   // If leaf: index into _children. If branch: left child node index.
        int right = -1;         // Branch only: right child node index. -1 for leaf.
        bool isLeaf = false;
    };

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

    void RebuildBVH() {
        _bvhNodes.clear();
        if (_children.size() <= 2) return; // Not worth building BVH for <=2 children

        std::vector<int> indices(_children.size());
        for (int i = 0; i < static_cast<int>(_children.size()); ++i) indices[i] = i;
        BuildBVHNode(indices, 0, static_cast<int>(_children.size()));
    }

    int BuildBVHNode(std::vector<int>& indices, int begin, int end) {
        BVHNode node{};
        // Compute AABB for this range
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

        // Split along longest axis at midpoint
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

        // Reserve space for this node
        int thisIdx = static_cast<int>(_bvhNodes.size());
        _bvhNodes.push_back(node); // placeholder

        int leftIdx = BuildBVHNode(indices, begin, mid);
        int rightIdx = BuildBVHNode(indices, mid, end);

        _bvhNodes[thisIdx].leftOrChild = leftIdx;
        _bvhNodes[thisIdx].right = rightIdx;
        _bvhNodes[thisIdx].isLeaf = false;
        return thisIdx;
    }

    static bool AABBOverlap(const AABB& a, const AABB& b) noexcept {
        return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
               (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
               (a.min.z <= b.max.z && a.max.z >= b.min.z);
    }

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

    std::vector<std::unique_ptr<Collider>> _children;
    std::vector<BVHNode> _bvhNodes;
    AABB _aabb{};
};
