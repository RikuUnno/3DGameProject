#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include "DxLib.h"

// 物理演算専用モニター
// - オブジェクト数、位置、サイズ、メモリ使用量
// - 物理ボディ、コライダー、接触点の詳細統計
// - リアルタイム可視化とTXTログ出力
class PhysicsMonitor {
public:
    static PhysicsMonitor& Instance() noexcept;

    // 表示ON/OFF
    void SetVisible(bool visible) noexcept { _visible = visible; }
    bool IsVisible() const noexcept { return _visible; }
    void ToggleVisible() noexcept { _visible = !_visible; }

    // 自動保存機能
    void EnableAutoSave(bool enable) noexcept { _autoSaveEnabled = enable; }
    bool IsAutoSaveEnabled() const noexcept { return _autoSaveEnabled; }
    void SetAutoSaveInterval(float seconds) noexcept { _autoSaveInterval = seconds; }
    float GetAutoSaveInterval() const noexcept { return _autoSaveInterval; }

    // 統計更新
    void Update(float dt = 1.0f / 60.0f);

    // 画面描画
    void Draw(int x = 10, int y = 130) const;

    // 詳細ログ保存
    void SaveDetailedLog(const char* filename = nullptr) const;

    // 個別オブジェクト情報
    struct ObjectInfo {
        std::string name;
        VECTOR worldPos;
        VECTOR size;           // コライダーサイズ
        size_t memoryBytes;    // 推定メモリ使用量
        bool isStatic;
        bool isSleeping;
        float mass;
        VECTOR velocity;
        int contactCount;      // 接触数
    };

    // 統計情報
    struct Statistics {
        // オブジェクト数
        int totalObjects = 0;
        int dynamicObjects = 0;
        int staticObjects = 0;
        int sleepingObjects = 0;
        int activeObjects = 0;

        // コライダー統計
        int totalColliders = 0;
        int sphereColliders = 0;
        int boxColliders = 0;
        int capsuleColliders = 0;
        int planeColliders = 0;
        int compoundColliders = 0;

        // 物理ボディ統計
        int totalBodies = 0;
        int activeBodies = 0;
        int sleepingBodies = 0;

        // 接触統計
        int totalContacts = 0;
        int activeContacts = 0;
        float maxPenetration = 0.0f;   // 最大めり込み量（診断用）
        float avgPenetration = 0.0f;   // 平均めり込み量（診断用）

        // アイランド統計
        int totalIslands = 0;
        int largestIslandSize = 0;

        // メモリ使用量（推定）
        size_t colliderMemoryBytes = 0;
        size_t bodyMemoryBytes = 0;
        size_t contactMemoryBytes = 0;
        size_t totalMemoryBytes = 0;

        // 空間分割統計
        int gridCells = 0;
        int candidatePairs = 0;
        int narrowPhasePairs = 0;

        // パフォーマンス
        float physicsTimeMs = 0.0f;
        float collisionTimeMs = 0.0f;
        float solverTimeMs = 0.0f;
    };

    const Statistics& GetStatistics() const noexcept { return _stats; }
    const std::vector<ObjectInfo>& GetObjectList() const noexcept { return _objects; }

    PhysicsMonitor(const PhysicsMonitor&) = delete;
    PhysicsMonitor& operator=(const PhysicsMonitor&) = delete;

private:
    PhysicsMonitor() = default;
    ~PhysicsMonitor() = default;

    void UpdateStatistics();
    void UpdateObjectList();

    bool _visible = true;
    Statistics _stats;
    std::vector<ObjectInfo> _objects;

    // 自動保存機能
    bool _autoSaveEnabled = false;
    float _autoSaveInterval = 5.0f; // 秒
    mutable float _autoSaveTimer = 0.0f;
};
