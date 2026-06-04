#pragma once

#include <vector>
#include <memory>
#include <unordered_map>
#include <atomic>
#include <mutex>
#include <string>
#include "ObjectPool.h"
#include "Manager.h"

class GameObject;
using VariantMap = std::unordered_map<std::string, std::string>;

class ObjectManager : public Manager {
public:
    static ObjectManager& Instance() noexcept;

    // オブジェクト取得（プールがあれば再利用、なければ Factory 生成）
    GameObject* Spawn(const std::string& key, const VariantMap& params = {});
    // プール登録
    void RegisterPool(const std::string& key, size_t maxSize = 64);
    // プール登録 + モデルテンプレート（1キー1モデル）も同時に登録するヘルパー。
    // modelPath は .mv1 / .x / .fbx (Unity FBX エクスポート形式含む) に対応。
    // モデル登録のみ失敗した場合は false を返すが、ObjectPool の登録は維持される。
    bool RegisterPool(const std::string& key, const std::string& modelPath,
                      size_t maxSize = 64, size_t maxModelPoolSize = 32);
    // オブジェクト返却（プールキーがあれば返却、なければ破棄）
    void Release(GameObject* obj);

    void UpdateAll(float dtSec);
    void DrawAll();

    GameObject* FindById(int id) const;
    bool        RemoveById(int id);

    // 現在アクティブなシーン ID（Spawn されたオブジェクトの ownerSceneId に付与）
    void SetCurrentSceneId(int sceneId);
    int  CurrentSceneId() const;
    // 指定シーン ID のオブジェクトを一括 Release（シーン終了時に使用）
    void ReleaseBySceneId(int sceneId);

    bool   ClearPool(const std::string& key);
    size_t TrimPoolUnused(const std::string& key, double maxIdleSeconds);
    size_t TrimAllPoolsUnused(double maxIdleSeconds);
    // プール登録解除（使用中オブジェクトが残っている場合は false を返す）
    bool   UnregisterPool(const std::string& key);

    void Update(float dt) override { UpdateAll(dt); }

#ifdef _DEBUG
    void DebugDraw(int x = 10, int y = 90) const;
#endif

    ObjectManager(const ObjectManager&) = delete;
    ObjectManager& operator=(const ObjectManager&) = delete;

private:
    ObjectManager() = default;
    virtual ~ObjectManager();

    // --- オブジェクト管理コンテナ ---
    // raw ポインタキーにすることで Release/FindByPtr が O(1) になる。
    std::unordered_map<GameObject*, ObjectPool::UniquePtr> _objects;

    // ID → ptr のセカンダリインデックス（FindById / RemoveById 用）
    std::unordered_map<int, GameObject*> _idIndex;

    // UpdateAll 用永続スナップショットバッファ（毎フレームの heap 確保を排除）
    std::vector<GameObject*> _snapshotBuf;

    // プール管理
    std::unordered_map<std::string, std::unique_ptr<ObjectPool>> _pools;

    mutable std::mutex _mtx;

    int             _currentSceneId = 0;
    std::atomic<int> _nextId{ 1 }; // Spawn 時に自動付与する一意 ID

#ifdef _DEBUG
    size_t _debugTotalAcquire = 0;
    size_t _debugTotalCreated = 0;
    size_t _debugTotalDeleted = 0;
#endif
};
