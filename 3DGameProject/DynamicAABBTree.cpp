#include "DynamicAABBTree.h"
#include <algorithm>
#include <cmath>
#include <cfloat>

namespace {
    inline float Dot3(const VECTOR& a, const VECTOR& b) noexcept {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline bool IntersectAABB(const AABB& a, const AABB& b) noexcept {
        return (a.min.x <= b.max.x && a.max.x >= b.min.x) &&
               (a.min.y <= b.max.y && a.max.y >= b.min.y) &&
               (a.min.z <= b.max.z && a.max.z >= b.min.z);
    }
}

DynamicAABBTree::NodeId DynamicAABBTree::AllocateNode() {
    if (_freeList != NULL_NODE) {
        NodeId nodeId = _freeList;
        _freeList = _nodes[nodeId].parent;
        _nodes[nodeId] = Node{};
        _nodes[nodeId].height = 0;
        return nodeId;
    }

    NodeId nodeId = static_cast<NodeId>(_nodes.size());
    _nodes.emplace_back();
    return nodeId;
}

void DynamicAABBTree::FreeNode(NodeId nodeId) {
    if (nodeId < 0 || nodeId >= static_cast<NodeId>(_nodes.size())) return;
    _nodes[nodeId].parent = _freeList;
    _nodes[nodeId].height = -1;
    _freeList = nodeId;
}

AABB DynamicAABBTree::Union(const AABB& a, const AABB& b) const {
    AABB result;
    result.min.x = (std::min)(a.min.x, b.min.x);
    result.min.y = (std::min)(a.min.y, b.min.y);
    result.min.z = (std::min)(a.min.z, b.min.z);
    result.max.x = (std::max)(a.max.x, b.max.x);
    result.max.y = (std::max)(a.max.y, b.max.y);
    result.max.z = (std::max)(a.max.z, b.max.z);
    return result;
}

AABB DynamicAABBTree::FattenAABB(const AABB& aabb, const VECTOR& displacement) const {
    AABB result;
    const VECTOR extension = VGet(kAABBExtension, kAABBExtension, kAABBExtension);
    result.min = VSub(aabb.min, extension);
    result.max = VAdd(aabb.max, extension);

    if (displacement.x < 0.0f) result.min.x += kAABBMultiplier * displacement.x;
    else result.max.x += kAABBMultiplier * displacement.x;

    if (displacement.y < 0.0f) result.min.y += kAABBMultiplier * displacement.y;
    else result.max.y += kAABBMultiplier * displacement.y;

    if (displacement.z < 0.0f) result.min.z += kAABBMultiplier * displacement.z;
    else result.max.z += kAABBMultiplier * displacement.z;

    return result;
}

float DynamicAABBTree::SurfaceArea(const AABB& aabb) const {
    const VECTOR d = VSub(aabb.max, aabb.min);
    return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
}

DynamicAABBTree::NodeId DynamicAABBTree::Insert(Collider* collider, const AABB& aabb) {
    if (!collider) return NULL_NODE;

    NodeId nodeId = AllocateNode();
    _nodes[nodeId].aabb = FattenAABB(aabb, VGet(0, 0, 0));
    _nodes[nodeId].collider = collider;
    _nodes[nodeId].height = 0;

    InsertLeaf(nodeId);
    _colliderToNode[collider] = nodeId;
    return nodeId;
}

void DynamicAABBTree::Remove(NodeId nodeId) {
    if (nodeId == NULL_NODE || nodeId < 0 || nodeId >= static_cast<NodeId>(_nodes.size())) return;
    if (!_nodes[nodeId].IsLeaf()) return;

    Collider* collider = _nodes[nodeId].collider;
    if (collider) _colliderToNode.erase(collider);

    RemoveLeaf(nodeId);
    FreeNode(nodeId);
}

bool DynamicAABBTree::Update(NodeId nodeId, const AABB& newAABB, const VECTOR& displacement) {
    if (nodeId == NULL_NODE || nodeId < 0 || nodeId >= static_cast<NodeId>(_nodes.size())) return false;
    if (!_nodes[nodeId].IsLeaf()) return false;

    if (IntersectAABB(_nodes[nodeId].aabb, newAABB)) {
        return false;
    }

    RemoveLeaf(nodeId);
    _nodes[nodeId].aabb = FattenAABB(newAABB, displacement);
    InsertLeaf(nodeId);
    return true;
}

void DynamicAABBTree::InsertLeaf(NodeId leaf) {
    if (_root == NULL_NODE) {
        _root = leaf;
        _nodes[leaf].parent = NULL_NODE;
        return;
    }

    AABB leafAABB = _nodes[leaf].aabb;
    NodeId index = _root;
    while (!_nodes[index].IsLeaf()) {
        NodeId left = _nodes[index].left;
        NodeId right = _nodes[index].right;

        float area = SurfaceArea(_nodes[index].aabb);
        AABB combinedAABB = Union(_nodes[index].aabb, leafAABB);
        float combinedArea = SurfaceArea(combinedAABB);

        float cost = 2.0f * combinedArea;
        float inheritanceCost = 2.0f * (combinedArea - area);

        float costLeft;
        if (_nodes[left].IsLeaf()) {
            AABB aabb = Union(leafAABB, _nodes[left].aabb);
            costLeft = SurfaceArea(aabb) + inheritanceCost;
        } else {
            AABB aabb = Union(leafAABB, _nodes[left].aabb);
            float oldArea = SurfaceArea(_nodes[left].aabb);
            float newArea = SurfaceArea(aabb);
            costLeft = (newArea - oldArea) + inheritanceCost;
        }

        float costRight;
        if (_nodes[right].IsLeaf()) {
            AABB aabb = Union(leafAABB, _nodes[right].aabb);
            costRight = SurfaceArea(aabb) + inheritanceCost;
        } else {
            AABB aabb = Union(leafAABB, _nodes[right].aabb);
            float oldArea = SurfaceArea(_nodes[right].aabb);
            float newArea = SurfaceArea(aabb);
            costRight = (newArea - oldArea) + inheritanceCost;
        }

        if (cost < costLeft && cost < costRight) {
            break;
        }

        if (costLeft < costRight) {
            index = left;
        } else {
            index = right;
        }
    }

    NodeId sibling = index;
    NodeId oldParent = _nodes[sibling].parent;
    NodeId newParent = AllocateNode();
    _nodes[newParent].parent = oldParent;
    _nodes[newParent].collider = nullptr;
    _nodes[newParent].aabb = Union(leafAABB, _nodes[sibling].aabb);
    _nodes[newParent].height = _nodes[sibling].height + 1;

    if (oldParent != NULL_NODE) {
        if (_nodes[oldParent].left == sibling) {
            _nodes[oldParent].left = newParent;
        } else {
            _nodes[oldParent].right = newParent;
        }

        _nodes[newParent].left = sibling;
        _nodes[newParent].right = leaf;
        _nodes[sibling].parent = newParent;
        _nodes[leaf].parent = newParent;
    } else {
        _nodes[newParent].left = sibling;
        _nodes[newParent].right = leaf;
        _nodes[sibling].parent = newParent;
        _nodes[leaf].parent = newParent;
        _root = newParent;
    }

    index = _nodes[leaf].parent;
    while (index != NULL_NODE) {
        index = Balance(index);

        NodeId left = _nodes[index].left;
        NodeId right = _nodes[index].right;

        _nodes[index].height = 1 + (std::max)(_nodes[left].height, _nodes[right].height);
        _nodes[index].aabb = Union(_nodes[left].aabb, _nodes[right].aabb);

        index = _nodes[index].parent;
    }
}

void DynamicAABBTree::RemoveLeaf(NodeId leaf) {
    if (leaf == _root) {
        _root = NULL_NODE;
        return;
    }

    NodeId parent = _nodes[leaf].parent;
    NodeId grandParent = _nodes[parent].parent;
    NodeId sibling = (_nodes[parent].left == leaf) ? _nodes[parent].right : _nodes[parent].left;

    if (grandParent != NULL_NODE) {
        if (_nodes[grandParent].left == parent) {
            _nodes[grandParent].left = sibling;
        } else {
            _nodes[grandParent].right = sibling;
        }
        _nodes[sibling].parent = grandParent;
        FreeNode(parent);

        NodeId index = grandParent;
        while (index != NULL_NODE) {
            index = Balance(index);

            NodeId left = _nodes[index].left;
            NodeId right = _nodes[index].right;

            _nodes[index].aabb = Union(_nodes[left].aabb, _nodes[right].aabb);
            _nodes[index].height = 1 + (std::max)(_nodes[left].height, _nodes[right].height);

            index = _nodes[index].parent;
        }
    } else {
        _root = sibling;
        _nodes[sibling].parent = NULL_NODE;
        FreeNode(parent);
    }
}

DynamicAABBTree::NodeId DynamicAABBTree::Balance(NodeId nodeId) {
    if (nodeId == NULL_NODE || _nodes[nodeId].IsLeaf() || _nodes[nodeId].height < 2) {
        return nodeId;
    }

    NodeId left = _nodes[nodeId].left;
    NodeId right = _nodes[nodeId].right;

    int balance = _nodes[right].height - _nodes[left].height;

    if (balance > 1) {
        return RotateLeft(nodeId);
    }

    if (balance < -1) {
        return RotateRight(nodeId);
    }

    return nodeId;
}

DynamicAABBTree::NodeId DynamicAABBTree::RotateLeft(NodeId nodeId) {
    NodeId right = _nodes[nodeId].right;
    NodeId rightLeft = _nodes[right].left;

    _nodes[right].left = nodeId;
    _nodes[right].parent = _nodes[nodeId].parent;
    _nodes[nodeId].parent = right;

    if (_nodes[right].parent != NULL_NODE) {
        if (_nodes[_nodes[right].parent].left == nodeId) {
            _nodes[_nodes[right].parent].left = right;
        } else {
            _nodes[_nodes[right].parent].right = right;
        }
    } else {
        _root = right;
    }

    if (rightLeft != NULL_NODE) {
        _nodes[rightLeft].parent = nodeId;
    }
    _nodes[nodeId].right = rightLeft;

    _nodes[nodeId].aabb = Union(_nodes[_nodes[nodeId].left].aabb, _nodes[_nodes[nodeId].right].aabb);
    _nodes[right].aabb = Union(_nodes[nodeId].aabb, _nodes[_nodes[right].right].aabb);

    _nodes[nodeId].height = 1 + (std::max)(_nodes[_nodes[nodeId].left].height, _nodes[_nodes[nodeId].right].height);
    _nodes[right].height = 1 + (std::max)(_nodes[nodeId].height, _nodes[_nodes[right].right].height);

    return right;
}

DynamicAABBTree::NodeId DynamicAABBTree::RotateRight(NodeId nodeId) {
    NodeId left = _nodes[nodeId].left;
    NodeId leftRight = _nodes[left].right;

    _nodes[left].right = nodeId;
    _nodes[left].parent = _nodes[nodeId].parent;
    _nodes[nodeId].parent = left;

    if (_nodes[left].parent != NULL_NODE) {
        if (_nodes[_nodes[left].parent].left == nodeId) {
            _nodes[_nodes[left].parent].left = left;
        } else {
            _nodes[_nodes[left].parent].right = left;
        }
    } else {
        _root = left;
    }

    if (leftRight != NULL_NODE) {
        _nodes[leftRight].parent = nodeId;
    }
    _nodes[nodeId].left = leftRight;

    _nodes[nodeId].aabb = Union(_nodes[_nodes[nodeId].left].aabb, _nodes[_nodes[nodeId].right].aabb);
    _nodes[left].aabb = Union(_nodes[_nodes[left].left].aabb, _nodes[nodeId].aabb);

    _nodes[nodeId].height = 1 + (std::max)(_nodes[_nodes[nodeId].left].height, _nodes[_nodes[nodeId].right].height);
    _nodes[left].height = 1 + (std::max)(_nodes[_nodes[left].left].height, _nodes[nodeId].height);

    return left;
}

void DynamicAABBTree::Query(const AABB& queryAABB, std::vector<Collider*>& out) const {
    if (_root == NULL_NODE) return;

    std::vector<NodeId> stack;
    stack.reserve(256);
    stack.push_back(_root);

    while (!stack.empty()) {
        NodeId nodeId = stack.back();
        stack.pop_back();

        if (nodeId == NULL_NODE) continue;

        const Node& node = _nodes[nodeId];

        if (IntersectAABB(node.aabb, queryAABB)) {
            if (node.IsLeaf()) {
                if (node.collider) {
                    out.push_back(node.collider);
                }
            } else {
                stack.push_back(node.left);
                stack.push_back(node.right);
            }
        }
    }
}

void DynamicAABBTree::QueryPairs(std::vector<std::pair<Collider*, Collider*>>& outPairs) {
    if (_root == NULL_NODE) return;

    std::vector<NodeId> stack;
    stack.reserve(256);

    auto querySelf = [&](NodeId nodeA, NodeId nodeB, auto& querySelfRef) -> void {
        if (nodeA == NULL_NODE || nodeB == NULL_NODE) return;

        if (nodeA == nodeB) {
            if (!_nodes[nodeA].IsLeaf()) {
                querySelfRef(_nodes[nodeA].left, _nodes[nodeA].right, querySelfRef);
                querySelfRef(_nodes[nodeA].left, _nodes[nodeA].left, querySelfRef);
                querySelfRef(_nodes[nodeA].right, _nodes[nodeA].right, querySelfRef);
            }
            return;
        }

        if (!IntersectAABB(_nodes[nodeA].aabb, _nodes[nodeB].aabb)) return;

        if (_nodes[nodeA].IsLeaf() && _nodes[nodeB].IsLeaf()) {
            if (_nodes[nodeA].collider && _nodes[nodeB].collider) {
                outPairs.emplace_back(_nodes[nodeA].collider, _nodes[nodeB].collider);
            }
        } else if (_nodes[nodeA].IsLeaf()) {
            querySelfRef(nodeA, _nodes[nodeB].left, querySelfRef);
            querySelfRef(nodeA, _nodes[nodeB].right, querySelfRef);
        } else if (_nodes[nodeB].IsLeaf()) {
            querySelfRef(_nodes[nodeA].left, nodeB, querySelfRef);
            querySelfRef(_nodes[nodeA].right, nodeB, querySelfRef);
        } else {
            querySelfRef(_nodes[nodeA].left, _nodes[nodeB].left, querySelfRef);
            querySelfRef(_nodes[nodeA].left, _nodes[nodeB].right, querySelfRef);
            querySelfRef(_nodes[nodeA].right, _nodes[nodeB].left, querySelfRef);
            querySelfRef(_nodes[nodeA].right, _nodes[nodeB].right, querySelfRef);
        }
    };

    querySelf(_root, _root, querySelf);
}

int DynamicAABBTree::GetHeight() const {
    if (_root == NULL_NODE) return 0;
    return _nodes[_root].height;
}

int DynamicAABBTree::ComputeHeight(NodeId nodeId) const {
    if (nodeId == NULL_NODE || _nodes[nodeId].IsLeaf()) return 0;
    int leftHeight = ComputeHeight(_nodes[nodeId].left);
    int rightHeight = ComputeHeight(_nodes[nodeId].right);
    return 1 + (std::max)(leftHeight, rightHeight);
}

float DynamicAABBTree::GetSurfaceArea() const {
    if (_root == NULL_NODE) return 0.0f;
    return SurfaceArea(_nodes[_root].aabb);
}

void DynamicAABBTree::Clear() {
    _nodes.clear();
    _colliderToNode.clear();
    _root = NULL_NODE;
    _freeList = NULL_NODE;
}

void DynamicAABBTree::Rebalance() {
    if (_root == NULL_NODE) return;

    std::vector<NodeId> leaves;
    std::vector<NodeId> stack;
    stack.push_back(_root);

    while (!stack.empty()) {
        NodeId nodeId = stack.back();
        stack.pop_back();

        if (nodeId == NULL_NODE) continue;

        if (_nodes[nodeId].IsLeaf()) {
            leaves.push_back(nodeId);
        } else {
            stack.push_back(_nodes[nodeId].left);
            stack.push_back(_nodes[nodeId].right);
        }
    }

    std::vector<Node> savedLeaves;
    std::vector<Collider*> savedColliders;
    for (NodeId leafId : leaves) {
        savedLeaves.push_back(_nodes[leafId]);
        savedColliders.push_back(_nodes[leafId].collider);
    }

    Clear();

    for (size_t i = 0; i < savedLeaves.size(); ++i) {
        Insert(savedColliders[i], savedLeaves[i].aabb);
    }
}
