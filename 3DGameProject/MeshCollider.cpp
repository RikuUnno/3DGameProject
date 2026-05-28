// MeshCollider.cpp
// 三角形メッシュコライダーの骨組み実装（Step 1）
// - 既存コライダ群と整合するインターフェースのみ提供する。
// - 衝突判定の本体は Step 3 以降で ColliderManager 側に追加する。
// - 本ファイルはビルドが通る最小実装で、登録/更新/AABB計算/簡易デバッグ描画のみ動作する。
#include "MeshCollider.h"

#include <algorithm>
#include <cmath>

#include "GameObject.h"
#include "DxLib.h"

namespace {
    // 安全な正規化（ゼロ長対策）
    inline VECTOR SafeNormalize(const VECTOR& v, const VECTOR& fallback) {
        const float l2 = v.x * v.x + v.y * v.y + v.z * v.z;
        if (l2 > 1e-12f) {
            const float inv = 1.0f / std::sqrt(l2);
            return VGet(v.x * inv, v.y * inv, v.z * inv);
        }
        return fallback;
    }

    // 三角形の法線とAABBを計算して埋める
    inline void FillTriangleDerived(Triangle& tri) {
        const VECTOR e1 = VSub(tri.v1, tri.v0);
        const VECTOR e2 = VSub(tri.v2, tri.v0);
        tri.normal = SafeNormalize(VCross(e1, e2), VGet(0, 1, 0));

        tri.aabb.min = VGet(
            (std::min)(tri.v0.x, (std::min)(tri.v1.x, tri.v2.x)),
            (std::min)(tri.v0.y, (std::min)(tri.v1.y, tri.v2.y)),
            (std::min)(tri.v0.z, (std::min)(tri.v1.z, tri.v2.z))
        );
        tri.aabb.max = VGet(
            (std::max)(tri.v0.x, (std::max)(tri.v1.x, tri.v2.x)),
            (std::max)(tri.v0.y, (std::max)(tri.v1.y, tri.v2.y)),
            (std::max)(tri.v0.z, (std::max)(tri.v1.z, tri.v2.z))
        );
        tri.aabb.center = VScale(VAdd(tri.aabb.min, tri.aabb.max), 0.5f);
    }
}

MeshCollider::MeshCollider() : Collider() {
    // 初期 AABB は空（未構築状態）
    _aabb.min = VGet(0, 0, 0);
    _aabb.max = VGet(0, 0, 0);
    _aabb.center = VGet(0, 0, 0);
}

MeshCollider::~MeshCollider() = default;

void MeshCollider::BuildFromVertices(const std::vector<VECTOR>& verts,
    const std::vector<int>& indices) {
    _trianglesLocal.clear();
    if (indices.size() < 3) {
        _trianglesWorld.clear();
        _builtOnce = false;
        SetAABB();
        return;
    }

    const size_t triCount = indices.size() / 3;
    _trianglesLocal.reserve(triCount);

    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        const int i0 = indices[i + 0];
        const int i1 = indices[i + 1];
        const int i2 = indices[i + 2];
        if (i0 < 0 || i1 < 0 || i2 < 0 ||
            static_cast<size_t>(i0) >= verts.size() ||
            static_cast<size_t>(i1) >= verts.size() ||
            static_cast<size_t>(i2) >= verts.size()) {
            continue; // 不正なインデックスはスキップ
        }
        Triangle tri{};
        tri.v0 = verts[i0];
        tri.v1 = verts[i1];
        tri.v2 = verts[i2];
        FillTriangleDerived(tri);
        _trianglesLocal.push_back(tri);
    }

    _builtOnce = false; // owner の Transform で再変換する
    RecomputeWorld();
    _builtOnce = true;
}

bool MeshCollider::BuildFromMV1(int mv1Handle, int frameIndex) {
    _trianglesLocal.clear();

    if (mv1Handle < 0) {
        _trianglesWorld.clear();
        _builtOnce = false;
        SetAABB();
        return false;
    }

    // 参照メッシュを「ローカル座標(IsTransform=FALSE)」でセットアップする。
    // owner の Transform を後段でかけるため、二重変換にならないようにする。
    // frameIndex < 0 のときはモデル全体を対象（DxLib では -1 指定でモデル全体）。
    const int frame = (frameIndex < 0) ? -1 : frameIndex;
    if (MV1SetupReferenceMesh(mv1Handle, frame, FALSE) < 0) {
        _trianglesWorld.clear();
        _builtOnce = false;
        SetAABB();
        return false;
    }

    const MV1_REF_POLYGONLIST refMesh = MV1GetReferenceMesh(mv1Handle, frame, FALSE);
    if (refMesh.PolygonNum <= 0 || refMesh.Polygons == nullptr || refMesh.Vertexs == nullptr) {
        // 取得失敗時は安全に空メッシュ化して終了
        _trianglesWorld.clear();
        _builtOnce = false;
        SetAABB();
        return false;
    }

    _trianglesLocal.reserve(static_cast<size_t>(refMesh.PolygonNum));
    for (int i = 0; i < refMesh.PolygonNum; ++i) {
        const auto& poly = refMesh.Polygons[i];
        const int i0 = poly.VIndex[0];
        const int i1 = poly.VIndex[1];
        const int i2 = poly.VIndex[2];
        if (i0 < 0 || i1 < 0 || i2 < 0 ||
            i0 >= refMesh.VertexNum ||
            i1 >= refMesh.VertexNum ||
            i2 >= refMesh.VertexNum) {
            continue;
        }
        Triangle tri{};
        tri.v0 = refMesh.Vertexs[i0].Position;
        tri.v1 = refMesh.Vertexs[i1].Position;
        tri.v2 = refMesh.Vertexs[i2].Position;
        FillTriangleDerived(tri);
        _trianglesLocal.push_back(tri);
    }

    _builtOnce = false;
    RecomputeWorld();
    _builtOnce = !_trianglesLocal.empty();
    return _builtOnce;
}

void MeshCollider::UpdateShape() {
    // 静的メッシュ：初回構築済みなら何もしない（パフォーマンス重視）
    if (isStatic && _builtOnce) {
        return;
    }
    // 動的メッシュ or 未構築：ワールド変換を更新
    RecomputeWorld();
    if (!_trianglesLocal.empty()) {
        _builtOnce = true;
    }
}

void MeshCollider::SetAABB() {
    if (_trianglesWorld.empty()) {
        _aabb.min = VGet(0, 0, 0);
        _aabb.max = VGet(0, 0, 0);
        _aabb.center = VGet(0, 0, 0);
        return;
    }

    VECTOR mn = VGet(1e30f, 1e30f, 1e30f);
    VECTOR mx = VGet(-1e30f, -1e30f, -1e30f);

    for (const auto& tri : _trianglesWorld) {
        if (tri.aabb.min.x < mn.x) mn.x = tri.aabb.min.x;
        if (tri.aabb.min.y < mn.y) mn.y = tri.aabb.min.y;
        if (tri.aabb.min.z < mn.z) mn.z = tri.aabb.min.z;
        if (tri.aabb.max.x > mx.x) mx.x = tri.aabb.max.x;
        if (tri.aabb.max.y > mx.y) mx.y = tri.aabb.max.y;
        if (tri.aabb.max.z > mx.z) mx.z = tri.aabb.max.z;
    }
    _aabb.min = mn;
    _aabb.max = mx;
    _aabb.center = VScale(VAdd(mn, mx), 0.5f);
}

void MeshCollider::RecomputeWorld() {
    _trianglesWorld.clear();
    _trianglesWorld.reserve(_trianglesLocal.size());

    if (!owner) {
        // owner が無い場合はローカル＝ワールドとみなす
        _trianglesWorld = _trianglesLocal;
    }
    else {
        // owner の Transform に従って各頂点をワールド変換
        for (const auto& src : _trianglesLocal) {
            Triangle tri{};
            tri.v0 = owner->transform.TransformPoint(src.v0);
            tri.v1 = owner->transform.TransformPoint(src.v1);
            tri.v2 = owner->transform.TransformPoint(src.v2);
            FillTriangleDerived(tri);
            _trianglesWorld.push_back(tri);
        }
    }
    SetAABB();
    RebuildBVH();
}

// --- BVH 構築 ---------------------------------------------------------------
// _trianglesWorld をリーフとするフラット二分木を構築する。
// 構造は CompoundCollider のそれと同型（ヘッダ側の QueryBVH と整合）。
void MeshCollider::RebuildBVH() {
    _bvhNodes.clear();
    if (_trianglesWorld.size() <= 1) return; // 1件以下は走査の方が安い

    std::vector<int> indices(_trianglesWorld.size());
    for (int i = 0; i < static_cast<int>(_trianglesWorld.size()); ++i) indices[i] = i;
    // ノード数の上限は概ね 2N。事前確保で再確保を避ける。
    _bvhNodes.reserve(_trianglesWorld.size() * 2);
    BuildBVHNode(indices, 0, static_cast<int>(_trianglesWorld.size()));
}

int MeshCollider::BuildBVHNode(std::vector<int>& indices, int begin, int end) {
    BVHNode node{};

    // この範囲の AABB を結合計算
    VECTOR mn = VGet(1e30f, 1e30f, 1e30f);
    VECTOR mx = VGet(-1e30f, -1e30f, -1e30f);
    for (int i = begin; i < end; ++i) {
        const AABB& ca = _trianglesWorld[indices[i]].aabb;
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

    if (end - begin <= 1) {
        node.isLeaf = true;
        node.leftOrChild = indices[begin];
        node.right = -1;
        int idx = static_cast<int>(_bvhNodes.size());
        _bvhNodes.push_back(node);
        return idx;
    }

    // 最長軸で中央値分割
    const float dx = mx.x - mn.x;
    const float dy = mx.y - mn.y;
    const float dz = mx.z - mn.z;
    int axis = 0;
    if (dy > dx && dy > dz) axis = 1;
    else if (dz > dx) axis = 2;

    auto GetAxisCenter = [&](int triIdx) -> float {
        const AABB& ca = _trianglesWorld[triIdx].aabb;
        if (axis == 0) return ca.center.x;
        if (axis == 1) return ca.center.y;
        return ca.center.z;
    };

    int mid = begin + (end - begin) / 2;
    std::nth_element(indices.begin() + begin, indices.begin() + mid, indices.begin() + end,
        [&](int a, int b) { return GetAxisCenter(a) < GetAxisCenter(b); });

    // 自ノード位置を確保（左右子の前に push して index を確定させる）
    int thisIdx = static_cast<int>(_bvhNodes.size());
    _bvhNodes.push_back(node); // placeholder

    int leftIdx = BuildBVHNode(indices, begin, mid);
    int rightIdx = BuildBVHNode(indices, mid, end);

    _bvhNodes[thisIdx].leftOrChild = leftIdx;
    _bvhNodes[thisIdx].right = rightIdx;
    _bvhNodes[thisIdx].isLeaf = false;
    return thisIdx;
}

// --- デバッグ描画 -----------------------------------------------------------
void MeshCollider::DrawDebug() {
    // 全三角形ワイヤ描画（巨大ステージでは DrawDebugInAABB を推奨）
    if (_trianglesWorld.empty()) return;

    const unsigned int col = DebugColor() ? DebugColor() : GetColor(0, 255, 0);
    for (const auto& tri : _trianglesWorld) {
        DrawTriangle3D(tri.v0, tri.v1, tri.v2, col, FALSE);
    }
}

void MeshCollider::DrawDebugInAABB(const AABB& query) {
    // BVH で範囲内の三角形のみ抽出して描画する。
    // narrow-phase に使うのと同じ QueryOverlapping 経路で動作確認も兼ねる。
    if (_trianglesWorld.empty()) return;
    const unsigned int col = DebugColor() ? DebugColor() : GetColor(0, 255, 0);
    QueryOverlapping(query, [&](size_t triIdx) {
        const Triangle& tri = _trianglesWorld[triIdx];
        DrawTriangle3D(tri.v0, tri.v1, tri.v2, col, FALSE);
    });
}

void MeshCollider::DrawDebugAABB() {
    if (_trianglesWorld.empty()) return;
    const unsigned int col = DebugColor() ? DebugColor() : GetColor(0, 128, 255);
    DrawCube3D(_aabb.min, _aabb.max, col, col, FALSE);
}

