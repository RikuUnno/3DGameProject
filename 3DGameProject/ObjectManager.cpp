#include "ObjectManager.h"
#include "ObjectFactory.h"
#include "ObjectPool.h"
#include "GameObject.h"
#include "ModelManager.h"
#include "Time.h"
#include "ThreadPool.h"
#include <algorithm>

#ifdef _DEBUG
#include "DxLib.h"
#endif

ObjectManager& ObjectManager::Instance() noexcept {
    static ObjectManager inst;
    return inst;
}

ObjectManager::~ObjectManager() {
    _objects.clear();
    _pools.clear();
}

// ---- 内部ヘルパー: オブジェクトを _objects / _idIndex に登録 ----------------

static int AssignId(GameObject* obj, std::atomic<int>& nextId,
    std::unordered_map<int, GameObject*>& idIndex)
{
    int id = obj->GetId();
    if (id < 0) {
        id = nextId.fetch_add(1, std::memory_order_relaxed);
        obj->SetId(id);
    }
    idIndex.emplace(id, obj);
    return id;
}

// ---- プール登録 --------------------------------------------------------------

void ObjectManager::RegisterPool(const std::string& key, size_t maxSize) {
    std::lock_guard lk(_mtx);
    if (_pools.count(key)) return;
    auto poolCreator = [key]() -> std::unique_ptr<GameObject> {
        return ObjectFactory::Instance().Create(key);
    };
    _pools.emplace(key, std::make_unique<ObjectPool>(poolCreator, maxSize));
}

bool ObjectManager::RegisterPool(const std::string& key, const std::string& modelPath,
                                 size_t maxSize, size_t maxModelPoolSize) {
    RegisterPool(key, maxSize);
    // モデルは 1 キーにつき 1 種類だけ登録する想定（既に登録済みなら true 扱い）
    return ModelManager::Instance().Register(key, modelPath, maxModelPoolSize);
}

// ---- シーン ID ---------------------------------------------------------------

void ObjectManager::SetCurrentSceneId(int sceneId) {
    std::lock_guard lk(_mtx);
    _currentSceneId = sceneId;
}

int ObjectManager::CurrentSceneId() const {
    std::lock_guard lk(_mtx);
    return _currentSceneId;
}

// ---- Spawn ------------------------------------------------------------------
// 1) プール登録済みなら Acquire → OnAcquire
// 2) なければ Factory::Create → OnAcquire
// どちらも _objects / _idIndex に登録して raw ポインタを返す。

GameObject* ObjectManager::Spawn(const std::string& key, const VariantMap& params) {
    std::lock_guard lk(_mtx);

    ObjectPool::UniquePtr up;
    bool wasCreated = false;

    auto pit = _pools.find(key);
    if (pit != _pools.end()) {
        up = pit->second->Acquire(&wasCreated);
    }

    if (!up) {
        // プール未登録 or プールが空かつ creator なし → Factory から直接生成
        auto raw_up = ObjectFactory::Instance().Create(key, params);
        if (!raw_up) return nullptr;
        up = ObjectPool::UniquePtr(
            raw_up.release(),
            ObjectPool::Deleter([](GameObject* p) { delete p; })
        );
        wasCreated = true;
    }

    if (!up) return nullptr;

    GameObject* raw = up.get();
    raw->SetActive(true);
    raw->_poolKey      = (pit != _pools.end()) ? key : std::string{};
    raw->_ownerSceneId = _currentSceneId;
    // ModelManager に同名キーが登録されていればモデルも自動取得
    // （1 オブジェクト = 1 モデル前提）
    if (ModelManager::Instance().IsRegistered(key)) {
        raw->_model = ModelManager::Instance().Acquire(key);
    }
    raw->OnAcquire(params);

    AssignId(raw, _nextId, _idIndex);
    _objects.emplace(raw, std::move(up));

#ifdef _DEBUG
    ++_debugTotalAcquire;
    if (wasCreated) ++_debugTotalCreated;
#endif
    return raw;
}

// ---- Release ----------------------------------------------------------------
// O(1): raw ポインタキーで直接 erase。

void ObjectManager::Release(GameObject* obj) {
    if (!obj) return;
    std::lock_guard lk(_mtx);

    auto it = _objects.find(obj);
    if (it == _objects.end()) return;

    _idIndex.erase(obj->GetId());

    if (!obj->_poolKey.empty()) {
        obj->OnRelease();
        obj->SetActive(false);
    } else {
        obj->OnDestroy();
#ifdef _DEBUG
        ++_debugTotalDeleted;
#endif
        obj->SetActive(false);
    }
    // モデルもプールへ返却（unique_ptr の解放でデリータが呼ばれる）
    obj->_model.reset();
    _objects.erase(it);
}

// ---- シーン一括 Release -----------------------------------------------------

void ObjectManager::ReleaseBySceneId(int sceneId) {
    std::lock_guard lk(_mtx);
    for (auto it = _objects.begin(); it != _objects.end(); ) {
        GameObject* obj = it->first;
        if (!obj || obj->_ownerSceneId != sceneId) { ++it; continue; }

        obj->End();
        _idIndex.erase(obj->GetId());

        if (!obj->_poolKey.empty()) {
            obj->OnRelease();
        } else {
            obj->OnDestroy();
#ifdef _DEBUG
            ++_debugTotalDeleted;
#endif
        }
        obj->_model.reset();
        obj->SetActive(false);
        it = _objects.erase(it);
    }
}
// ---- FindById / RemoveById --------------------------------------------------
// O(1): _idIndex のセカンダリインデックスで検索。

GameObject* ObjectManager::FindById(int id) const {
    std::lock_guard lk(_mtx);
    auto it = _idIndex.find(id);
    return (it != _idIndex.end()) ? it->second : nullptr;
}

bool ObjectManager::RemoveById(int id) {
    std::lock_guard lk(_mtx);
    auto idIt = _idIndex.find(id);
    if (idIt == _idIndex.end()) return false;

    GameObject* obj = idIt->second;
    _idIndex.erase(idIt);

    auto objIt = _objects.find(obj);
    if (objIt != _objects.end()) {
        obj->OnDestroy();
#ifdef _DEBUG
        ++_debugTotalDeleted;
#endif
        obj->_model.reset();
        _objects.erase(objIt);
    }
    return true;
}

// ---- UpdateAll / DrawAll ----------------------------------------------------

void ObjectManager::UpdateAll(float dtSec) {
    // ローカル変数としてスナップショットを作成（thisへの依存を排除）
    std::vector<GameObject*> snapshot;
    {
        std::lock_guard lk(_mtx);
        snapshot.reserve(_objects.size());
        for (auto& [ptr, up] : _objects) {
            if (ptr && ptr->IsActive())
                snapshot.push_back(ptr);
        }
    }

    const size_t count = snapshot.size();
    if (count == 0) return;

    // ラムダでthisを参照せず、ローカル変数のみをキャプチャ
    ThreadPool::Instance().ParallelFor(0, count, [&snapshot, dtSec](size_t i) {
        snapshot[i]->Update(dtSec);
    }, 16);
}

void ObjectManager::DrawAll() {
    std::lock_guard lk(_mtx);
    for (auto& [ptr, up] : _objects) {
        if (ptr && ptr->IsActive()) ptr->Draw();
    }
}

// ---- プール管理・内容 -------------------------------------------------------

bool ObjectManager::ClearPool(const std::string& key) {
    std::lock_guard lk(_mtx);
    auto it = _pools.find(key);
    if (it == _pools.end() || !it->second) return false;
#ifdef _DEBUG
    _debugTotalDeleted += it->second->Size();
#endif
    it->second->Clear();
    return true;
}

size_t ObjectManager::TrimPoolUnused(const std::string& key, double maxIdleSeconds) {
    const double now = Time::Instance().GetTotalTime();
    std::lock_guard lk(_mtx);
    auto it = _pools.find(key);
    if (it == _pools.end() || !it->second) return 0;
    const size_t removed = it->second->TrimUnused(maxIdleSeconds, now);
#ifdef _DEBUG
    _debugTotalDeleted += removed;
#endif
    return removed;
}

size_t ObjectManager::TrimAllPoolsUnused(double maxIdleSeconds) {
    const double now = Time::Instance().GetTotalTime();
    std::lock_guard lk(_mtx);
    size_t total = 0;
    for (auto& [k, pool] : _pools) {
        if (!pool) continue;
        const size_t removed = pool->TrimUnused(maxIdleSeconds, now);
        total += removed;
#ifdef _DEBUG
        _debugTotalDeleted += removed;
#endif
    }
    return total;
}

bool ObjectManager::UnregisterPool(const std::string& key) {
    std::lock_guard lk(_mtx);
    // 使用中オブジェクトが残っている場合は登録解除しない
    for (const auto& [ptr, up] : _objects) {
        if (up && up->_poolKey == key) return false;
    }
    auto it = _pools.find(key);
    if (it == _pools.end()) return false;
#ifdef _DEBUG
    if (it->second) _debugTotalDeleted += it->second->Size();
#endif
    if (it->second) it->second->Clear();
    _pools.erase(it);
    // 紐づくモデルテンプレート/プールも破棄（登録されていなければ no-op）
    ModelManager::Instance().Unregister(key);
    return true;
}

// ---- デバッグ表示 -----------------------------------------------------------

#ifdef _DEBUG
void ObjectManager::DebugDraw(int x, int y) const {
    std::lock_guard lk(_mtx);

    const int lineH  = 16;
    const int rightX = x + 360;
    int leftY  = y;
    int rightY = y;

    DrawFormatString(x, leftY, GetColor(255,255,0),
        "[ObjectManager] objects:%d", (int)_objects.size()); leftY += lineH;
    DrawFormatString(x, leftY, GetColor(255,255,0),
        "[ObjectManager] acquire:%d", (int)_debugTotalAcquire); leftY += lineH;
    DrawFormatString(x, leftY, GetColor(255,255,0),
        "[ObjectManager] created:%d", (int)_debugTotalCreated); leftY += lineH;
    DrawFormatString(x, leftY, GetColor(255,255,0),
        "[ObjectManager] deleted:%d", (int)_debugTotalDeleted); leftY += lineH;
    DrawFormatString(x, leftY, GetColor(255,255,0),
        "[ObjectManager] sceneId:%d", _currentSceneId); leftY += lineH;
    DrawFormatString(x, leftY, GetColor(255,255,0),
        "[ObjectManager] pools:%d",   (int)_pools.size());

    DrawFormatString(rightX, rightY, GetColor(255,255,0), "[Pool] free stock"); rightY += lineH;
    for (const auto& [key, pool] : _pools) {
        DrawFormatString(rightX, rightY, GetColor(200,255,200),
            "%s : %d", key.c_str(), pool ? (int)pool->Size() : 0);
        rightY += lineH;
    }
}
#endif
