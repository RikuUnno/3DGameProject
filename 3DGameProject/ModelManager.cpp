#include "ModelManager.h"
#include "Mv1Model.h"
#include "Time.h"

ModelManager& ModelManager::Instance() noexcept {
    static ModelManager inst;
    return inst;
}

bool ModelManager::Register(const std::string& key, const std::string& filePath, size_t maxPoolSize) {
    return Register(key, filePath,
        [](const std::string& p) -> std::unique_ptr<IModel> {
            auto m = std::make_unique<Mv1Model>();
            if (!m->Load(p)) return nullptr;
            return m;
        }, maxPoolSize);
}

bool ModelManager::Register(const std::string& key, const std::string& filePath, ModelLoader loader, size_t maxPoolSize) {
    if (!loader) return false;

    // テンプレート読み込みは重いのでロック外で行う
    auto tmpl = loader(filePath);
    if (!tmpl) return false;

    std::lock_guard lk(_mtx);

    // 既存登録はそのまま (再登録したい場合は先に Unregister)
    auto it = _entries.find(key);
    if (it != _entries.end()) return true;

    IModel* rawTemplate = tmpl.get();

    auto pool = std::make_unique<ModelPool>(
        [rawTemplate]() -> std::unique_ptr<IModel> {
            // テンプレートから軽量複製
            return rawTemplate ? rawTemplate->Duplicate() : nullptr;
        },
        maxPoolSize);

    Entry e;
    e.templateModel = std::move(tmpl);
    e.pool = std::move(pool);
    _entries.emplace(key, std::move(e));
    return true;
}

bool ModelManager::Unregister(const std::string& key) {
    std::lock_guard lk(_mtx);
    auto it = _entries.find(key);
    if (it == _entries.end()) return false;
    if (it->second.pool) it->second.pool->Clear();
    _entries.erase(it);
    return true;
}

bool ModelManager::IsRegistered(const std::string& key) const {
    std::lock_guard lk(_mtx);
    return _entries.find(key) != _entries.end();
}

ModelManager::TypedUniquePtr ModelManager::Acquire(const std::string& key) {
    ModelPool* pool = nullptr;
    {
        std::lock_guard lk(_mtx);
        auto it = _entries.find(key);
        if (it == _entries.end() || !it->second.pool) {
            return TypedUniquePtr(nullptr, [](IModel*) {});
        }
        pool = it->second.pool.get();
    }
    return pool->AcquireModel();
}

IModel* ModelManager::GetTemplate(const std::string& key) const {
    std::lock_guard lk(_mtx);
    auto it = _entries.find(key);
    if (it == _entries.end()) return nullptr;
    return it->second.templateModel.get();
}

size_t ModelManager::TrimUnused(const std::string& key, double maxIdleSeconds) {
    const double now = Time::Instance().GetTotalTime();
    std::lock_guard lk(_mtx);
    auto it = _entries.find(key);
    if (it == _entries.end() || !it->second.pool) return 0;
    return it->second.pool->TrimUnused(maxIdleSeconds, now);
}

size_t ModelManager::TrimAllUnused(double maxIdleSeconds) {
    const double now = Time::Instance().GetTotalTime();
    std::lock_guard lk(_mtx);
    size_t total = 0;
    for (auto& [k, e] : _entries) {
        if (e.pool) total += e.pool->TrimUnused(maxIdleSeconds, now);
    }
    return total;
}

void ModelManager::ClearAll() {
    std::lock_guard lk(_mtx);
    for (auto& [k, e] : _entries) {
        if (e.pool) e.pool->Clear();
    }
    _entries.clear();
}
