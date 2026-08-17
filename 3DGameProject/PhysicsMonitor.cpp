#include "PhysicsMonitor.h"
#include "PhysicsManager.h"
#include "ColliderManager.h"
#include "PhysicsBody.h"
#include "Collider.h"
#include "SphereCollider.h"
#include "BoxCollider.h"
#include "CapsuleCollider.h"
#include "HalfPlaneCollider.h"
#include "CompoundCollider.h"
#include "GameObject.h"
#include "Transform.h"
#include "PerformanceMonitor.h"
#include <fstream>
#include <algorithm>
#undef min
#undef max

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

namespace {
    // GameObject **********とりあえず有効なポインタか判定する処理************
    //  - メモリ上でのアドレスが0x10000未満なら無効とみなす
    //  - IsBadReadPtr によるポインタ妥当性チェック（Windows専用）
    bool IsLikelyValidGameObject(const GameObject* obj) noexcept {
        if (!obj) return false;

        // メモリ上でのアドレスが0x10000未満なら無効とみなす
        const auto addr = reinterpret_cast<uintptr_t>(obj);
        if (addr < 0x10000ull) return false;

        // IsBadReadPtr によるポインタ妥当性チェック（Windows専用）
        if (IsBadReadPtr(obj, sizeof(GameObject))) return false;

        // vtable が null または無効なアドレスの場合は無効とみなす
        const void* const* vptr = *reinterpret_cast<const void* const* const*>(obj);
        if (vptr == nullptr) return false;
        if (reinterpret_cast<uintptr_t>(vptr) < 0x10000ull) return false;
        if (IsBadReadPtr(vptr, sizeof(void*))) return false;

        return true;
    }

    // Collider のポインタ妥当性チェック（vtable まで含めて検証）
    //  - ダングリングポインタによる仮想関数呼び出しクラッシュを防ぐ
    bool IsLikelyValidCollider(const Collider* col) noexcept {
        if (!col) return false;

        const auto addr = reinterpret_cast<uintptr_t>(col);
        if (addr < 0x10000ull) return false;

        if (IsBadReadPtr(col, sizeof(Collider))) return false;

        // vtable チェック
        const void* const* vptr = *reinterpret_cast<const void* const* const*>(col);
        if (vptr == nullptr) return false;
        if (reinterpret_cast<uintptr_t>(vptr) < 0x10000ull) return false;
        if (IsBadReadPtr(vptr, sizeof(void*))) return false;

        return true;
    }

    // std::string のポインタ妥当性チェック
    //  - サイズやアドレスのチェックを行い、怪しい場合は無効とみなす
    bool IsLikelyValidString(const std::string& s) noexcept {
        const size_t sz = s.size();
        const size_t cap = s.capacity();
        // 異常に大きなサイズ・キャパシティは無効とみなす
        constexpr size_t kSane = 1ull << 28; // 256MB上限
        if (sz > kSane || cap > kSane) return false;
        // サイズがキャパシティを超えているのも無効
        if (sz > cap) return false;
        return true;
    }
}

PhysicsMonitor& PhysicsMonitor::Instance() noexcept {
    static PhysicsMonitor inst;
    return inst;
}

void PhysicsMonitor::Update(float dt) {
    UpdateStatistics();
    UpdateObjectList();

    // 自動保存
    if (_autoSaveEnabled) {
        _autoSaveTimer += dt;
        if (_autoSaveTimer >= _autoSaveInterval) {
            _autoSaveTimer = 0.0f;
            SaveDetailedLog();
        }
    }
}

void PhysicsMonitor::UpdateStatistics() {
    _stats = Statistics{};

    // PhysicsManagerのシャットダウンチェック
    if (PhysicsManager::Instance().IsShuttingDown()) {
        return;
    }
    // ColliderManager のシャットダウンチェック
    if (ColliderManager::Instance().IsShuttingDown()) {
        return;
    }

    // PhysicsBody統計
    const auto& bodies = PhysicsManager::Instance().GetBodies();
    _stats.totalBodies = static_cast<int>(bodies.size());
    for (const auto* body : bodies) {
        if (!body) continue;
        if (body->_isKinematic || body->_mass <= 0.0f) {
            _stats.staticObjects++;
        } else {
            _stats.dynamicObjects++;
            if (body->_isSleeping) {
                _stats.sleepingBodies++;
                _stats.sleepingObjects++;
            } else {
                _stats.activeBodies++;
                _stats.activeObjects++;
            }
        }
        _stats.bodyMemoryBytes += sizeof(PhysicsBody);
    }
    _stats.totalObjects = _stats.dynamicObjects + _stats.staticObjects;

    // Collider統計
    const auto& colliders = ColliderManager::Instance().GetColliders();
    _stats.totalColliders = static_cast<int>(colliders.size());
    for (const auto* col : colliders) {
        // ダングリングポインタ対策: vtable まで含めて妥当性を確認
        if (!IsLikelyValidCollider(col)) continue;

        size_t colSize = sizeof(Collider);
        switch (col->GetKind()) {
            case Collider::Kind::Sphere:
                _stats.sphereColliders++;
                colSize = sizeof(SphereCollider);
                break;
            case Collider::Kind::Box:
                _stats.boxColliders++;
                colSize = sizeof(BoxCollider);
                break;
            case Collider::Kind::Capsule:
                _stats.capsuleColliders++;
                colSize = sizeof(CapsuleCollider);
                break;
            case Collider::Kind::HalfPlane:
                _stats.planeColliders++;
                colSize = sizeof(HalfPlaneCollider);
                break;
            case Collider::Kind::Compound:
                _stats.compoundColliders++;
                colSize = sizeof(CompoundCollider);
                break;
        }
        _stats.colliderMemoryBytes += colSize;
    }

    // 接触統計
    const auto& contacts = ColliderManager::Instance().GetContacts();
    _stats.totalContacts = static_cast<int>(contacts.size());
    _stats.activeContacts = _stats.totalContacts;
    _stats.contactMemoryBytes = contacts.size() * sizeof(ColliderManager::Contact);

    // めり込み診断: 接触の penetration の最大値/平均値を集計する。
    // 「見た目がめり込んでいる」とき、ここが大きければソルバ/位置補正の問題、
    // 接触数が 0 または pen が小さければ接触生成(ナロウフェーズ)の問題と切り分けられる。
    _stats.maxPenetration = 0.0f;
    _stats.avgPenetration = 0.0f;
    if (!contacts.empty()) {
        float sum = 0.0f;
        for (const auto& ct : contacts) {
            if (ct.penetration > _stats.maxPenetration) _stats.maxPenetration = ct.penetration;
            sum += ct.penetration;
        }
        _stats.avgPenetration = sum / static_cast<float>(contacts.size());
    }

    // アイランド統計（PhysicsManagerから取得）
    // Note: PhysicsManagerにアイランド情報取得APIが必要
    // 仮実装: アクティブボディからの推定
    _stats.totalIslands = (_stats.activeBodies > 0) ? 1 : 0;
    _stats.largestIslandSize = _stats.activeBodies;

    // 総メモリ
    _stats.totalMemoryBytes = _stats.bodyMemoryBytes + _stats.colliderMemoryBytes + _stats.contactMemoryBytes;

    // パフォーマンス統計（PerformanceMonitorから取得）
    _stats.physicsTimeMs = 0.0f;
    _stats.collisionTimeMs = 0.0f;
    _stats.solverTimeMs = 0.0f;

    const auto sections = PerformanceMonitor::Instance().GetTopSections(50);
    for (const auto& sec : sections) {
        if (sec.name == "Physics.Update" || sec.name == "Physics.StepSimulation") {
            _stats.physicsTimeMs = (std::max)(_stats.physicsTimeMs, static_cast<float>(sec.timeUs) / 1000.0f);
        } else if (sec.name == "Collider.CheckDetailedCollisions") {
            _stats.collisionTimeMs += static_cast<float>(sec.timeUs) / 1000.0f;
        } else if (sec.name == "Collider.SpatialPartitioning") {
            _stats.collisionTimeMs += static_cast<float>(sec.timeUs) / 1000.0f;
        } else if (sec.name == "Collider.Update") {
            _stats.collisionTimeMs += static_cast<float>(sec.timeUs) / 1000.0f;
        } else if (sec.name == "Physics.SolveAllIslands") {
            _stats.solverTimeMs = static_cast<float>(sec.timeUs) / 1000.0f;
        }
    }
}

void PhysicsMonitor::UpdateObjectList() {
    _objects.clear();

    // PhysicsManagerのシャットダウンチェック
    if (PhysicsManager::Instance().IsShuttingDown()) {
        return;
    }
    if (ColliderManager::Instance().IsShuttingDown()) {
        return;
    }

    const auto& colliders = ColliderManager::Instance().GetColliders();
    const auto& bodies = PhysicsManager::Instance().GetBodies();

    // ボディからオブジェクト情報を構築
    std::unordered_map<GameObject*, PhysicsBody*> bodyMap;
    for (auto* body : bodies) {
        if (!body || !body->_owner) continue;
        if (!IsLikelyValidGameObject(body->_owner)) continue;
        bodyMap[body->_owner] = body;
    }

    // コライダーを走査してオブジェクト情報を収集
    for (const auto* col : colliders) {
        // ダングリングポインタ対策
        if (!IsLikelyValidCollider(col)) continue;
        if (!col->owner) continue;

        // ********** GameObject の妥当性チェック **********
        // 有効でない場合、スキップ
        if (!IsLikelyValidGameObject(col->owner)) {
            continue;
        }
        // プールキーが無効な文字列の場合もスキップ
        if (!IsLikelyValidString(col->owner->_poolKey)) {
            continue;
        }

        ObjectInfo info;
        info.name = col->owner->_poolKey.empty() ? std::string("Unknown") : col->owner->_poolKey;
        info.worldPos = col->owner->transform.WorldPosition();
        info.isStatic = false;
        info.isSleeping = false;
        info.mass = 0.0f;
        info.velocity = VGet(0, 0, 0);
        info.contactCount = 0;

        // サイズ取得
        const AABB aabb = col->GetAABB();
        info.size = VGet(
            aabb.max.x - aabb.min.x,
            aabb.max.y - aabb.min.y,
            aabb.max.z - aabb.min.z
        );

        // メモリサイズ推定
        size_t colSize = sizeof(Collider);
        switch (col->GetKind()) {
            case Collider::Kind::Sphere:   colSize = sizeof(SphereCollider); break;
            case Collider::Kind::Box:      colSize = sizeof(BoxCollider); break;
            case Collider::Kind::Capsule:  colSize = sizeof(CapsuleCollider); break;
            case Collider::Kind::HalfPlane: colSize = sizeof(HalfPlaneCollider); break;
            case Collider::Kind::Compound: colSize = sizeof(CompoundCollider); break;
        }
        info.memoryBytes = colSize + sizeof(GameObject);

        // PhysicsBody情報
        auto it = bodyMap.find(col->owner);
        if (it != bodyMap.end() && it->second) {
            PhysicsBody* body = it->second;
            info.isStatic = body->_isKinematic || body->_mass <= 0.0f;
            info.isSleeping = body->_isSleeping;
            info.mass = body->_mass;
            info.velocity = body->_velocity;
            info.memoryBytes += sizeof(PhysicsBody);
        }

        // 接触数カウント
        const auto& contacts = ColliderManager::Instance().GetContacts();
        for (const auto& ct : contacts) {
            if (ct.a == col || ct.b == col) {
                info.contactCount++;
            }
        }

        _objects.push_back(std::move(info));
    }

    // 名前順にソート
    std::sort(_objects.begin(), _objects.end(), [](const ObjectInfo& a, const ObjectInfo& b) {
        return a.name < b.name;
    });
}

void PhysicsMonitor::Draw(int x, int y) const {
    if (!_visible) return;

    const unsigned int white   = GetColor(255, 255, 255);
    const unsigned int yellow  = GetColor(255, 255, 0);
    const unsigned int cyan    = GetColor(100, 255, 255);
    const unsigned int green   = GetColor(100, 255, 100);
    const unsigned int red     = GetColor(255, 100, 100);
    const unsigned int blue    = GetColor(150, 200, 255);

    int ly = y;
    const int lineHeight = 20;

    // タイトル
    DrawFormatString(x, ly, yellow, "=== Physics Monitor ==="); ly += lineHeight;

    // オブジェクト統計
    DrawFormatString(x, ly, white, "Objects: %d (Dynamic:%d Static:%d)", 
        _stats.totalObjects, _stats.dynamicObjects, _stats.staticObjects); ly += lineHeight;
    DrawFormatString(x, ly, green, "  Active:%d Sleeping:%d", 
        _stats.activeObjects, _stats.sleepingObjects); ly += lineHeight;

    // コライダー統計
    DrawFormatString(x, ly, cyan, "Colliders: %d", _stats.totalColliders); ly += lineHeight;
    DrawFormatString(x, ly, white, "  Sphere:%d Box:%d Capsule:%d Plane:%d Compound:%d",
        _stats.sphereColliders, _stats.boxColliders, _stats.capsuleColliders,
        _stats.planeColliders, _stats.compoundColliders); ly += lineHeight;

    // 物理ボディ統計
    DrawFormatString(x, ly, cyan, "Bodies: %d (Active:%d Sleep:%d)",
        _stats.totalBodies, _stats.activeBodies, _stats.sleepingBodies); ly += lineHeight;

    // 接触統計
    DrawFormatString(x, ly, _stats.totalContacts > 0 ? red : white, 
        "Contacts: %d", _stats.totalContacts); ly += lineHeight;

    // めり込み診断表示: MaxPen が球の半径級 (>0.1) のまま張り付くなら
    // ソルバ/位置補正が機能していない。Contacts=0 で見た目めり込みなら
    // 接触生成自体が失敗している。
    DrawFormatString(x, ly, _stats.maxPenetration > 0.05f ? red : white,
        "Penetration: max=%.4f avg=%.4f", _stats.maxPenetration, _stats.avgPenetration); ly += lineHeight;

    // アイランド統計
    DrawFormatString(x, ly, white, "Islands: %d (Largest:%d) ",
        _stats.totalIslands, _stats.largestIslandSize); ly += lineHeight;

    // メモリ統計
    const float totalMB = _stats.totalMemoryBytes / (1024.0f * 1024.0f);
    DrawFormatString(x, ly, yellow, "Memory: %.2f MB", totalMB); ly += lineHeight;
    DrawFormatString(x, ly, white, "  Body:%.2fKB Col:%.2fKB Contact:%.2fKB",
        _stats.bodyMemoryBytes / 1024.0f,
        _stats.colliderMemoryBytes / 1024.0f,
        _stats.contactMemoryBytes / 1024.0f); ly += lineHeight;

    // パフォーマンス統計
    DrawFormatString(x, ly, yellow, "Performance:"); ly += lineHeight;
    DrawFormatString(x, ly, white, "  Physics: %.2fms", _stats.physicsTimeMs); ly += lineHeight;
    DrawFormatString(x, ly, white, "  Collision: %.2fms", _stats.collisionTimeMs); ly += lineHeight;
    DrawFormatString(x, ly, white, "  Solver: %.2fms", _stats.solverTimeMs); ly += lineHeight;

    // トップオブジェクト表示（上位5個）
    ly += lineHeight;
    DrawFormatString(x, ly, yellow, "Top Objects (by contact):"); ly += lineHeight;

    std::vector<ObjectInfo> sortedObjs = _objects;
    std::sort(sortedObjs.begin(), sortedObjs.end(), [](const ObjectInfo& a, const ObjectInfo& b) {
        return a.contactCount > b.contactCount;
    });

    const int maxDisplay = 5;
    const int displayCount = (std::min)(maxDisplay, static_cast<int>(sortedObjs.size()));
    for (int i = 0; i < displayCount; ++i) {
        const auto& obj = sortedObjs[i];
        const unsigned int color = obj.isSleeping ? GetColor(150, 150, 150) : white;
        DrawFormatString(x, ly, color, "  %s: Contacts:%d Pos:(%.1f,%.1f,%.1f)  Vel:(%.2f,%.2f,%.2f)",
            obj.name.c_str(), obj.contactCount,
            obj.worldPos.x, obj.worldPos.y, obj.worldPos.z,
            obj.velocity.x, obj.velocity.y, obj.velocity.z);
        ly += lineHeight;
    }
}

void PhysicsMonitor::SaveDetailedLog(const char* filename) const {
    // PhysicsManagerのシャットダウンチェック
    if (PhysicsManager::Instance().IsShuttingDown()) {
        return;
    }

    std::string fname;
    if (filename) {
        fname = filename;
    } else {
        fname = "PhysicsLog.txt";
    }

    std::ofstream ofs(fname, std::ios::out | std::ios::trunc);
    if (!ofs) return;

    ofs << "\n========================================\n";
    ofs << "[PHYSICS DETAILED LOG]\n";
    ofs << "========================================\n";

    // タイムスタンプ
    {
        SYSTEMTIME st;
        GetLocalTime(&st);
        ofs << "Timestamp: " << st.wYear << "/" << st.wMonth << "/" << st.wDay
            << " " << st.wHour << ":" << st.wMinute << ":" << st.wSecond << "\n";
    }

    // 統計概要
    ofs << "\n--- Object Statistics ---\n";
    ofs << "Total Objects: " << _stats.totalObjects << "\n";
    ofs << "  Dynamic: " << _stats.dynamicObjects << "\n";
    ofs << "  Static: " << _stats.staticObjects << "\n";
    ofs << "  Active: " << _stats.activeObjects << "\n";
    ofs << "  Sleeping: " << _stats.sleepingObjects << "\n";

    ofs << "\n--- Collider Statistics ---\n";
    ofs << "Total Colliders: " << _stats.totalColliders << "\n";
    ofs << "  Sphere: " << _stats.sphereColliders << "\n";
    ofs << "  Box: " << _stats.boxColliders << "\n";
    ofs << "  Capsule: " << _stats.capsuleColliders << "\n";
    ofs << "  Plane: " << _stats.planeColliders << "\n";
    ofs << "  Compound: " << _stats.compoundColliders << "\n";

    ofs << "\n--- Physics Body Statistics ---\n";
    ofs << "Total Bodies: " << _stats.totalBodies << "\n";
    ofs << "  Active: " << _stats.activeBodies << "\n";
    ofs << "  Sleeping: " << _stats.sleepingBodies << "\n";

    ofs << "\n--- Contact Statistics ---\n";
    ofs << "Total Contacts: " << _stats.totalContacts << "\n";
    ofs << "Active Contacts: " << _stats.activeContacts << "\n";

    ofs << "\n--- Island Statistics ---\n";
    ofs << "Total Islands: " << _stats.totalIslands << "\n";
    ofs << "Largest Island Size: " << _stats.largestIslandSize << "\n";

    ofs << "\n--- Memory Usage ---\n";
    ofs << "Total: " << (_stats.totalMemoryBytes / 1024.0f) << " KB\n";
    ofs << "  Bodies: " << (_stats.bodyMemoryBytes / 1024.0f) << " KB\n";
    ofs << "  Colliders: " << (_stats.colliderMemoryBytes / 1024.0f) << " KB\n";
    ofs << "  Contacts: " << (_stats.contactMemoryBytes / 1024.0f) << " KB\n";

    ofs << "\n--- Performance ---\n";
    ofs << "Physics Time: " << _stats.physicsTimeMs << " ms\n";
    ofs << "Collision Time: " << _stats.collisionTimeMs << " ms\n";
    ofs << "Solver Time: " << _stats.solverTimeMs << " ms\n";

    // オブジェクト詳細
    ofs << "\n--- All Objects Detail ---\n";
    for (const auto& obj : _objects) {
        ofs << "\n[" << obj.name << "]\n";
        ofs << "  Position: (" << obj.worldPos.x << ", " << obj.worldPos.y << ", " << obj.worldPos.z << ")\n";
        ofs << "  Size: (" << obj.size.x << ", " << obj.size.y << ", " << obj.size.z << ")\n";
        ofs << "  Memory: " << (obj.memoryBytes / 1024.0f) << " KB\n";
        ofs << "  Static: " << (obj.isStatic ? "Yes" : "No") << "\n";
        ofs << "  Sleeping: " << (obj.isSleeping ? "Yes" : "No") << "\n";
        ofs << "  Mass: " << obj.mass << "\n";
        ofs << "  Velocity: (" << obj.velocity.x << ", " << obj.velocity.y << ", " << obj.velocity.z << ")\n";
        ofs << "  Contacts: " << obj.contactCount << "\n";
    }

    ofs << "\n========================================\n\n";
}
