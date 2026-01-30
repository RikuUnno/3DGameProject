#pragma once

#include <string>
#include <unordered_map>
#include <memory>

using VariantMap = std::unordered_map<std::string, std::string>; // パラメータマップ定義

class GameObject {
public:
    GameObject() = default;
    virtual ~GameObject() = default;

    // ライフサイクル
    virtual void Awake() {}
    virtual void Start() {}
    virtual void Update() {}
    virtual void Draw() {}
    virtual void OnDestroy() {}

    // プール/再利用フック
    virtual void OnAcquire(const VariantMap& params) {} // 取得時初期化
    virtual void OnRelease() {}                         // 返却時クリア

    // Prototype: 深い複製（派生で実装）
    virtual std::unique_ptr<GameObject> Clone() const { return nullptr; }

    // 再利用設定
    // - poolKey が空でない場合はプール返却対象として扱う（ObjectManager::Release が参照）
    std::string poolKey;

    // 所属情報（シーン終了時の一括回収用）
    // - SceneManager が現在シーンのIDを払い出し、ObjectManager::Spawn が設定する
    int ownerSceneId = -1;

    // ID 管理（ObjectManager の検索/削除用）
    int GetId() const { return id_; }
    void SetId(int id) { id_ = id; }

private:
    int id_ = -1;
};
