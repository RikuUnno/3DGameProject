#pragma once

#include "SceneTpl.h"
#include "Assert.h" // runtime / マクロ用
#include <memory>
#include <vector>
#include <utility> // std::forward
#include <type_traits> // std::is_base_of_v

// シーン管理クラス（シングルトン）
// - IScene を std::unique_ptrで所有し、切替/プッシュ/ポップを管理する。
// - 遷移要求は Request* 系で保留し、ProcessPendingChanges()で反映する設計。
class SceneManager {
public:
	// シングルトン取得
	static SceneManager& Instance() noexcept;

	// フレームごとの呼び出し
	void Update();
	void Draw();

	// 即時切替（呼び出し元が安全な場合に使用）
	void ChangeScene(std::unique_ptr<IScene> scene);

	// 型から直接生成して即時切替（引数をコンストラクタに転送）
	template<typename T, typename... Args>
	void ChangeSceneByType(Args&&... args) {
		// コンパイル時チェック：型が IScene を継承しているかを早期に検出
		static_assert(std::is_base_of_v<IScene, T>, "T must derive from IScene");

		ChangeScene(std::make_unique<T>(std::forward<Args>(args)...));
	}

	// スタック操作（即時）
	void PushScene(std::unique_ptr<IScene> scene);
	void PopScene();

	// 保留リクエスト（Update 中など安全でないタイミングで呼ぶ）
	void RequestChange(std::unique_ptr<IScene> scene);
	void RequestPush(std::unique_ptr<IScene> scene);
	void RequestPop();

	// フレーム末に呼んで保留中の遷移を処理する（Mainループの最後で呼ぶ）
	void ProcessPendingChanges();

	// シーンがあるか
	bool HasScene() const { return !m_stack_.empty(); }

	// 現在アクティブなシーンID（シーン切替ごとに増加）
	int CurrentSceneId() const noexcept { return m_currentSceneId_; }

	// コピー/ムーブ禁止（シングルトン）
	SceneManager(const SceneManager&) = delete;
	SceneManager& operator=(const SceneManager&) = delete;
	SceneManager(SceneManager&&) = delete;
	SceneManager& operator=(SceneManager&&) = delete;

private:
	SceneManager() = default;
	~SceneManager() = default;

	// スタック（トップが現在アクティブなシーン）
	std::vector<std::unique_ptr<IScene>> m_stack_;

	// 保留中のシーン遷移
	std::unique_ptr<IScene> m_pendingChange_;
	std::unique_ptr<IScene> m_pendingPush_;
	bool m_pendingPop_ = false;

	int m_currentSceneId_ =0;
};
