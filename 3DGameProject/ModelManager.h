#pragma once

#include <string>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <functional>

#include "Manager.h"
#include "IModel.h"
#include "ModelPool.h"

// ModelManager
// - 「モデルキー」単位に 1 個のテンプレート IModel を読み込む (重い処理は 1 回だけ)
// - 同じキーで Acquire するとプールから複製 (Mv1Model なら MV1DuplicateModel) で
//   軽量な IModel インスタンスを取り出せる
// - ObjectManager と連携: GameObject の _poolKey と同じキーで Register しておけば、
//   GameObject::OnAcquire 内で AcquireModel(_poolKey) するだけで描画できる
//
// 拡張:
//   Mv1Model 以外の形式 (独自メッシュ等) を扱いたい場合は Register に
//   独自の Loader を渡すか、別オーバーロードを追加すれば良い
class ModelManager : public Manager {
public:
    using TypedUniquePtr = ModelPool::TypedUniquePtr;
    using ModelLoader = std::function<std::unique_ptr<IModel>(const std::string& path)>;

    static ModelManager& Instance() noexcept;

    // 既定 (MV1) でテンプレートをロード。.mv1 / .x / .fbx に対応。
    bool Register(const std::string& key, const std::string& filePath, size_t maxPoolSize = 32);

    // 任意 Loader を使ってテンプレートをロード (独自モデル用拡張点)
    bool Register(const std::string& key, const std::string& filePath, ModelLoader loader, size_t maxPoolSize = 32);

    // テンプレート + プールを破棄
    bool Unregister(const std::string& key);

    bool IsRegistered(const std::string& key) const;

    // プールから複製を取り出す。返却 unique_ptr が破棄されると自動でプールに戻る。
    // 未登録 / 失敗時は空の unique_ptr を返す。
    TypedUniquePtr Acquire(const std::string& key);

    // 生のテンプレート参照 (情報取得用 / 描画には Acquire したインスタンスを使うこと)
    IModel* GetTemplate(const std::string& key) const;

    // プールの未使用ストックを掃除
    size_t TrimUnused(const std::string& key, double maxIdleSeconds);
    size_t TrimAllUnused(double maxIdleSeconds);
    void ClearAll();

    void Update(float /*dt*/) override {}

    ModelManager(const ModelManager&) = delete;
    ModelManager& operator=(const ModelManager&) = delete;

private:
    ModelManager() = default;
    ~ModelManager() override = default;

    struct Entry {
        std::unique_ptr<IModel> templateModel; // 1 回だけ読み込んだ重いリソース
        std::unique_ptr<ModelPool> pool;       // テンプレートから複製するためのプール
    };

    mutable std::mutex _mtx;
    std::unordered_map<std::string, Entry> _entries;
};
