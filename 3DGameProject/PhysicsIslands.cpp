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

// ---- Island pool helpers --------------------------------------------

int PhysicsManager::AcquireIsland() {
    if (!_islandPool.empty()) {
        _islands.emplace_back(std::move(_islandPool.back()));
        _islandPool.pop_back();
        auto& isl = _islands.back();
        isl.bodies.clear();
        isl.contactIndices.clear();
        for (auto& batch : isl.constraintBatches) batch.clear();
        isl.constraintBatches.clear();
        isl.allSleeping = false;
    } else {
        _islands.emplace_back();
    }
    return static_cast<int>(_islands.size()) - 1;
}

void PhysicsManager::RecycleAllIslands() {
    // 中身を破棄せずプールへ退避（内部 vector の capacity を温存）
    const size_t total = _islandPool.size() + _islands.size();
    if (_islandPool.capacity() < total) _islandPool.reserve(total);
    for (auto& isl : _islands) {
        isl.bodies.clear();
        isl.contactIndices.clear();
        for (auto& batch : isl.constraintBatches) batch.clear();
        isl.constraintBatches.clear();
        isl.allSleeping = false;
        _islandPool.emplace_back(std::move(isl));
    }
    _islands.clear();
}

void PhysicsManager::BuildIslands() {
    if (_bodies.empty()) {
        RecycleAllIslands();
        _bodyIslandMap.clear();
        return;
    }

    // 変化検出: body 数とコンタクトグラフのハッシュが同じなら再構築スキップ
    // ペア (a,b) と (b,a) を同一視するため min/max を取ってからハッシュ。
    // 旧実装は冒頭で _islands.clear() を呼んでいたため graphUnchanged が
    // 常に false になりファストパスがデッドコードだった。
    const size_t newBodyCount = _bodies.size();
    size_t contactHash = _solverContacts.size() * 2654435761ULL;
    for (const auto& sc : _solverContacts) {
        const uintptr_t ua = reinterpret_cast<uintptr_t>(sc.colA);
        const uintptr_t ub = reinterpret_cast<uintptr_t>(sc.colB);
        const uintptr_t lo = (ua < ub) ? ua : ub;
        const uintptr_t hi = (ua < ub) ? ub : ua;
        contactHash ^= lo * 2654435761ULL + hi * 2246822519ULL
                     + (contactHash << 6) + (contactHash >> 2);
    }
    const bool graphUnchanged = (newBodyCount == _prevIslandBodyCount)
                             && (contactHash  == _prevContactHash)
                             && !_islands.empty();
    _prevIslandBodyCount = newBodyCount;
    _prevContactHash     = contactHash;

    if (graphUnchanged) {
        // 高速パス: アイランド構造は維持し、接触インデックスのみ再割り当て
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

    RecycleAllIslands();
    _bodyIslandMap.clear();

    // 通常パス: 完全再構築
    auto& bodyIndex = _bodyIndexBuf;
    bodyIndex.clear();
    const size_t bodySize = _bodies.size();
    if (bodySize > 0 && bodyIndex.bucket_count() < bodySize * 2)
        bodyIndex.reserve(bodySize);
    for (int i = 0; i < static_cast<int>(bodySize); ++i) {
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
            islandIdx = AcquireIsland();
            rootToIsland[root] = islandIdx;
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

    // AcquireIsland() による _islands 再確保で参照が無効になる前に元データを退避
    auto& origBodies   = _splitOrigBodiesBuf;
    auto& origContacts = _splitOrigContactsBuf;
    origBodies   = std::move(_islands[islandIdx].bodies);
    origContacts = std::move(_islands[islandIdx].contactIndices);
    // 既存スロットの bodies/contactIndices は move 済み（空・capacity 温存）
    // ここで新スロットを取得（_islands が再確保される可能性あり）
    const int newIslandIdx = AcquireIsland();

    auto& oldIsl = _islands[islandIdx];
    auto& newIsl = _islands[newIslandIdx];
    if (oldIsl.bodies.capacity() < static_cast<size_t>(count0)) oldIsl.bodies.reserve(count0);
    if (newIsl.bodies.capacity() < static_cast<size_t>(count1)) newIsl.bodies.reserve(count1);
    for (int i = 0; i < bodyCount; ++i) {
        if (color[i] == 0) oldIsl.bodies.push_back(origBodies[i]);
        else { newIsl.bodies.push_back(origBodies[i]); _bodyIslandMap[origBodies[i]] = newIslandIdx; }
    }

    if (oldIsl.contactIndices.capacity() < origContacts.size()) oldIsl.contactIndices.reserve(origContacts.size());
    if (newIsl.contactIndices.capacity() < origContacts.size() / 2) newIsl.contactIndices.reserve(origContacts.size() / 2);
    for (int ci : origContacts) {
        const auto& sc = _solverContacts[ci];
        auto itA = localIdx.find(sc.bodyA);
        auto itB = localIdx.find(sc.bodyB);
        const int cA = (itA != localIdx.end()) ? color[itA->second] : 0;
        const int cB = (itB != localIdx.end()) ? color[itB->second] : 0;
        if (cA == 1 && cB == 1) { newIsl.contactIndices.push_back(ci); _solverContacts[ci].islandId = newIslandIdx; }
        else oldIsl.contactIndices.push_back(ci);
    }

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
