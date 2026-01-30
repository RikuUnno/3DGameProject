#pragma once
#include <vector>
#include <memory>
#include <unordered_map>
#include <mutex>
#include <string>
#include "ObjectPool.h"

class GameObject;
using VariantMap = std::unordered_map<std::string, std::string>;

class ObjectManager {
public:
	// シングルトンインスタンス取得
	static ObjectManager& Instance() noexcept;

	// オブジェクト管理 API
	GameObject* Spawn(const std::string& key, const VariantMap& params = {});   // オブジェクト取得
	void RegisterPool(const std::string& key, size_t maxSize = 64);             // プール登録
	void Release(GameObject* obj);                                              // オブジェクト返却
	void UpdateAll();                                                  // 全オブジェクト更新
	void DrawAll();                                                             // 全オブジェクト描画
	GameObject* FindById(int id) const;                                         // ID で検索
	bool RemoveById(int id);                                                    // ID で削除

	// ---- Scene integration ----
	// 現在アクティブなシーンID（Spawnされたオブジェクトへ ownerSceneId を設定する）
	void SetCurrentSceneId(int sceneId);
	int CurrentSceneId() const;
	// 指定シーンIDに所属するオブジェクトを一括で Releaseする（シーン終了時用）
	void ReleaseBySceneId(int sceneId);

	// ---- Pool maintenance (generic / safe) ----
	// 未使用ストック（freeList）のみを破棄する
	bool ClearPool(const std::string& key);
	// 指定秒以上未使用のストックを破棄する（戻り値: 削除数）
	size_t TrimPoolUnused(const std::string& key, double maxIdleSeconds);
	// 全プールに対して TrimUnused を実行（戻り値: 総削除数）
	size_t TrimAllPoolsUnused(double maxIdleSeconds);
	// プール登録を解除する（freeList は破棄）。使用中が残っている場合は false.
	bool UnregisterPool(const std::string& key);

#ifdef _DEBUG
	// デバッグ表示（Releaseではコンパイルされない）
	void DebugDraw(int x = 10, int y = 90) const;
#endif

	// コピー禁止
	ObjectManager(const ObjectManager&) = delete;
	ObjectManager& operator=(const ObjectManager&) = delete;

private:
	// コンストラクタ・デストラクタ
	ObjectManager() = default;
	virtual ~ObjectManager();

	// _objects は ObjectPool::UniquePtr を保持する（プール由来/工場由来の両方を格納可能）
	std::vector<ObjectPool::UniquePtr> _objects;

	// プール管理コンテナ
	std::unordered_map<std::string, std::unique_ptr<class ObjectPool>> _pools; // key -> ObjectPool
	mutable std::mutex _mtx; // スレッド安全用ミューテックス

	int _currentSceneId = 0; // 現在のシーンID

#ifdef _DEBUG
	// デバッグ用統計
	size_t _debugTotalSpawn = 0;
	size_t _debugTotalDeleted = 0;
#endif
};
