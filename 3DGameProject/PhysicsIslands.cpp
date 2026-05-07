#include "PhysicsManager_Internal.h"

// ---- Union-Find（ランク付き）----------------------------------------

int PhysicsManager::UFFind(int x) noexcept {
    while (_ufParent[x] != x) {
        _ufParent[x] = _ufParent[_ufParent[x]]; // 経路半減
        x = _ufParent[x];
    }
    return x;
}

void PhysicsManager::UFUnite(int a, int b) noexcept {
    a = UFFind(a); b = UFFind(b);
    if (a == b) return;
    if (_ufRank[a] < _ufRank[b]) std::swap(a, b);
    _ufParent[b] = a;
    if (_ufRank[a] == _ufRank[b]) ++_ufRank[a];
}

// ---- BuildIslands ---------------------------------------------------

void PhysicsManager::BuildIslands() {
    _islands.clear();
    _bodyIslandMap.clear();
    if (_bodies.empty()) return;

    // 変化検出: body 数とコンタクトグラフのハッシュが同じなら再構築スキップ
    const size_t newBodyCount = _bodies.size();
    size_t contactHash = 0;
    for (const auto& sc : _solverContacts) {
        contactHash ^= reinterpret_cast<size_t>(sc.colA) * 2654435761ULL
                     + reinterpret_cast<size_t>(sc.colB) * 2246822519ULL;
    }
    const bool graphUnchanged = (newBodyCount == _prevIslandBodyCount)
                             && (contactHash  == _prevContactHash)
                             && !_islands.empty();
    _prevIslandBodyCount = newBodyCount;
    _prevContactHash     = contactHash;

    if (graphUnchanged) {
        // 高速パス: 接触インデックスだけ再割り当て
        for (auto& island : _islands) {
            island.contactIndices.clear();
            island.constraintBatches.clear();
        }
        for (int ci = 0; ci < static_cast<int>(_solverContacts.size()); ++ci) {
            auto& sc = _solverContacts[ci];
            PhysicsBody* rep = nullptr;
            if (sc.bodyA && sc.bodyA->IsDynamic() && !sc.bodyA->_isKinematic) rep = sc.bodyA;
            else if (sc.bodyB && sc.bodyB->IsDynamic() && !sc.bodyB->_isKinematic) rep = sc.bodyB;
            else rep = sc.bodyA ? sc.bodyA : sc.bodyB;
            if (!rep) continue;
            auto it = _bodyIslandMap.find(rep);
            if (it == _bodyIslandMap.end()) continue;
            sc.islandId = it->second;
            _islands[it->second].contactIndices.push_back(ci);
        }
        for (auto& island : _islands) {
            bool allSleep = true;
            for (auto* body : island.bodies) {
                if (body && body->IsDynamic() && !body->_isSleeping) { allSleep = false; break; }
            }
            island.allSleeping = allSleep;
            if (!island.allSleeping
                && static_cast<int>(island.contactIndices.size()) >= kBatchingThreshold) {
                BuildConstraintBatches(island);
            }
        }
        return;
    }

    // 通常パス: 完全再構築
    auto& bodyIndex = _bodyIndexBuf;
    bodyIndex.clear();
    if (bodyIndex.bucket_count() < _bodies.size()) bodyIndex.reserve(_bodies.size());
    for (int i = 0; i < static_cast<int>(_bodies.size()); ++i) {
        if (_bodies[i]) bodyIndex.emplace(_bodies[i], i);
    }

    const int n = static_cast<int>(_bodies.size());
    _ufParent.resize(n);
    _ufRank.assign(n, 0);
    for (int i = 0; i < n; ++i) _ufParent[i] = i;

    for (const auto& sc : _solverContacts) {
        int ia = -1, ib = -1;
        if (sc.bodyA) { auto it = bodyIndex.find(sc.bodyA); if (it != bodyIndex.end()) ia = it->second; }
        if (sc.bodyB) { auto it = bodyIndex.find(sc.bodyB); if (it != bodyIndex.end()) ib = it->second; }
        if (ia >= 0 && ib >= 0) {
            const bool aStatic = !_bodies[ia]->IsDynamic() || _bodies[ia]->_isKinematic;
            const bool bStatic = !_bodies[ib]->IsDynamic() || _bodies[ib]->_isKinematic;
            if (!aStatic && !bStatic) UFUnite(ia, ib);
        }
    }

    auto& rootToIsland = _rootToIslandBuf;
    rootToIsland.clear();
    rootToIsland.reserve(n);
    for (int i = 0; i < n; ++i) {
        if (!_bodies[i]) continue;
        const int root = UFFind(i);
        auto it = rootToIsland.find(root);
        int islandIdx;
        if (it == rootToIsland.end()) {
            islandIdx = static_cast<int>(_islands.size());
            rootToIsland[root] = islandIdx;
            _islands.emplace_back();
        } else {
            islandIdx = it->second;
        }
        _islands[islandIdx].bodies.push_back(_bodies[i]);
        _bodyIslandMap[_bodies[i]] = islandIdx;
    }

    for (int ci = 0; ci < static_cast<int>(_solverContacts.size()); ++ci) {
        auto& sc = _solverContacts[ci];
        PhysicsBody* rep = nullptr;
        if (sc.bodyA && sc.bodyA->IsDynamic() && !sc.bodyA->_isKinematic) rep = sc.bodyA;
        else if (sc.bodyB && sc.bodyB->IsDynamic() && !sc.bodyB->_isKinematic) rep = sc.bodyB;
        else rep = sc.bodyA ? sc.bodyA : sc.bodyB;
        if (!rep) continue;
        auto it = _bodyIslandMap.find(rep);
        if (it == _bodyIslandMap.end()) continue;
        sc.islandId = it->second;
        _islands[it->second].contactIndices.push_back(ci);
    }

    for (auto& island : _islands) {
        bool allSleep = true;
        for (auto* body : island.bodies) {
            if (body && body->IsDynamic() && !body->_isSleeping) { allSleep = false; break; }
        }
        island.allSleeping = allSleep;
    }

    for (auto& island : _islands) {
        if (!island.allSleeping && static_cast<int>(island.contactIndices.size()) >= kBatchingThreshold)
            BuildConstraintBatches(island);
    }

    // 大規模アイランドを分割
    {
        const int bodyTotal      = static_cast<int>(_bodies.size());
        const int estimatedMax   = static_cast<int>(_islands.size()) + bodyTotal / kIslandSplitThreshold + 2;
        _islands.reserve(static_cast<size_t>(estimatedMax));
        for (int ii = 0; ii < static_cast<int>(_islands.size()); ++ii) {
            if (!_islands[ii].allSleeping
                && static_cast<int>(_islands[ii].bodies.size()) > kIslandSplitThreshold) {
                SplitLargeIsland(ii, kIslandSplitThreshold);
            }
        }
    }
}

// ---- SplitLargeIsland -----------------------------------------------

void PhysicsManager::SplitLargeIsland(int islandIdx, int maxBodiesPerSplit) {
    if (islandIdx < 0 || static_cast<size_t>(islandIdx) >= _islands.size()) {
        ASSERT_MSG(false, "SplitLargeIsland: out of range. idx=%d size=%zu", islandIdx, _islands.size());
        return;
    }
    const int bodyCount = static_cast<int>(_islands[islandIdx].bodies.size());
    if (bodyCount <= maxBodiesPerSplit) return;

    auto& localIdx = _splitLocalIdxBuf;
    localIdx.clear();
    if (localIdx.bucket_count() < static_cast<size_t>(bodyCount)) localIdx.reserve(bodyCount);
    for (int i = 0; i < bodyCount; ++i)
        localIdx[_islands[islandIdx].bodies[i]] = i;

    auto& adj = _splitAdjBuf;
    adj.resize(bodyCount);
    for (auto& a : adj) a.clear();
    for (int ci : _islands[islandIdx].contactIndices) {
        const auto& sc = _solverContacts[ci];
        auto itA = localIdx.find(sc.bodyA);
        auto itB = localIdx.find(sc.bodyB);
        if (itA == localIdx.end() || itB == localIdx.end()) continue;
        adj[itA->second].push_back(itB->second);
        adj[itB->second].push_back(itA->second);
    }

    // BFS 2着色
    auto& color = _splitColorBuf;
    color.assign(bodyCount, -1);
    color[0] = 0;
    auto& queue = _splitQueueBuf;
    queue.clear();
    queue.push_back(0);
    for (size_t head = 0; head < queue.size(); ++head) {
        const int u = queue[head];
        for (int v : adj[u]) {
            if (color[v] < 0) { color[v] = 1 - color[u]; queue.push_back(v); }
        }
    }
    for (int i = 0; i < bodyCount; ++i) if (color[i] < 0) color[i] = 0;

    int count0 = 0, count1 = 0;
    for (int c : color) { if (c == 0) ++count0; else ++count1; }
    if (count1 == 0 || count0 == 0) return;

    // emplace_back 前に元データを退避（_islands 再確保で参照が無効になるのを防ぐ）
    auto& origBodies   = _splitOrigBodiesBuf;
    auto& origContacts = _splitOrigContactsBuf;
    origBodies   = std::move(_islands[islandIdx].bodies);
    origContacts = std::move(_islands[islandIdx].contactIndices);

    const int newIslandIdx = static_cast<int>(_islands.size());
    _islands.emplace_back();

    PhysicsIsland oldNew;
    oldNew.bodies.reserve(count0);
    _islands[newIslandIdx].bodies.reserve(count1);
    for (int i = 0; i < bodyCount; ++i) {
        if (color[i] == 0) oldNew.bodies.push_back(origBodies[i]);
        else { _islands[newIslandIdx].bodies.push_back(origBodies[i]); _bodyIslandMap[origBodies[i]] = newIslandIdx; }
    }

    oldNew.contactIndices.reserve(origContacts.size());
    _islands[newIslandIdx].contactIndices.reserve(origContacts.size() / 2);
    for (int ci : origContacts) {
        const auto& sc = _solverContacts[ci];
        auto itA = localIdx.find(sc.bodyA);
        auto itB = localIdx.find(sc.bodyB);
        const int cA = (itA != localIdx.end()) ? color[itA->second] : 0;
        const int cB = (itB != localIdx.end()) ? color[itB->second] : 0;
        if (cA == 1 && cB == 1) { _islands[newIslandIdx].contactIndices.push_back(ci); _solverContacts[ci].islandId = newIslandIdx; }
        else oldNew.contactIndices.push_back(ci);
    }

    _islands[islandIdx].bodies         = std::move(oldNew.bodies);
    _islands[islandIdx].contactIndices = std::move(oldNew.contactIndices);

    auto computeSleep = [](PhysicsIsland& isl) {
        bool allSleep = true;
        for (auto* body : isl.bodies)
            if (body && body->IsDynamic() && !body->_isSleeping) { allSleep = false; break; }
        isl.allSleeping = allSleep;
    };
    computeSleep(_islands[islandIdx]);
    computeSleep(_islands[newIslandIdx]);
}

// ---- BuildConstraintBatches -----------------------------------------

void PhysicsManager::BuildConstraintBatches(PhysicsIsland& island) {
    const int numContacts = static_cast<int>(island.contactIndices.size());
    if (numContacts <= 0) return;

    auto& bodyToContacts = _batchBodyToContactsBuf;
    bodyToContacts.clear();
    if (bodyToContacts.bucket_count() < island.bodies.size())
        bodyToContacts.reserve(island.bodies.size());
    for (int li = 0; li < numContacts; ++li) {
        const auto& sc = _solverContacts[island.contactIndices[li]];
        if (sc.bodyA) bodyToContacts[sc.bodyA].push_back(li);
        if (sc.bodyB) bodyToContacts[sc.bodyB].push_back(li);
    }

    // 貪欲着色（エポック配列で O(1) リセット）
    auto& contactColor   = _batchContactColorBuf;
    auto& usedColorEpoch = _batchUsedColorEpochBuf;
    contactColor.assign(numContacts, -1);
    usedColorEpoch.assign(numContacts + 2, -1);
    int maxColor = 0;
    for (int li = 0; li < numContacts; ++li) {
        const auto& sc = _solverContacts[island.contactIndices[li]];
        auto markUsed = [&](PhysicsBody* body) {
            if (!body) return;
            auto it = bodyToContacts.find(body);
            if (it == bodyToContacts.end()) return;
            for (int adj : it->second) {
                if (adj != li && contactColor[adj] >= 0) {
                    const int c = contactColor[adj];
                    if (c < static_cast<int>(usedColorEpoch.size())) usedColorEpoch[c] = li;
                }
            }
        };
        markUsed(sc.bodyA);
        markUsed(sc.bodyB);
        int color = 0;
        while (color < static_cast<int>(usedColorEpoch.size()) && usedColorEpoch[color] == li) ++color;
        contactColor[li] = color;
        if (color > maxColor) maxColor = color;
    }

    island.constraintBatches.resize(maxColor + 1);
    for (auto& batch : island.constraintBatches) batch.clear();
    for (int li = 0; li < numContacts; ++li)
        island.constraintBatches[contactColor[li]].push_back(island.contactIndices[li]);
}
