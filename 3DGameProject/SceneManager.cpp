#include "SceneManager.h"
#include "ObjectManager.h"

// シングルトン取得
SceneManager& SceneManager::Instance() noexcept {
	static SceneManager inst;
	return inst;
}

void SceneManager::Update() {
	if (!m_stack_.empty()) {
		m_stack_.back()->Update();
	}
}

void SceneManager::Draw() {
	if (!m_stack_.empty()) {
		// 必要なら下層も描画するロジックを追加可能
		m_stack_.back()->Draw();
	}
}

void SceneManager::ChangeScene(std::unique_ptr<IScene> scene) {
	//既存シーンを破棄する前に、そのシーンに紐づくオブジェクトを一括回収
	if (!m_stack_.empty()) {
		ObjectManager::Instance().ReleaseBySceneId(m_currentSceneId_);
	}

	//既存シーンを破棄
	if (!m_stack_.empty()) {
		m_stack_.back()->OnDestroy();
		m_stack_.clear();
	}

	// シーンIDを進めて、新シーンで Spawnされるオブジェクトに反映
	++m_currentSceneId_;
	ObjectManager::Instance().SetCurrentSceneId(m_currentSceneId_);

	if (scene) {
		scene->Awake();
		scene->Start();
		m_stack_.push_back(std::move(scene));
	}
}

void SceneManager::PushScene(std::unique_ptr<IScene> scene) {
	if (!m_stack_.empty()) {
		m_stack_.back()->OnSuspend();
	}
	// Push はシーンスタックを保持するので sceneId は進めない（同一シーン空間として扱う）
	if (scene) {
		scene->Awake();
		scene->Start();
		m_stack_.push_back(std::move(scene));
	}
}

void SceneManager::PopScene() {
	if (m_stack_.empty()) return;

	// Popするシーンに紐づく、現在 sceneId のオブジェクトを一括回収
	ObjectManager::Instance().ReleaseBySceneId(m_currentSceneId_);

	m_stack_.back()->OnDestroy();
	m_stack_.pop_back();
	if (!m_stack_.empty()) {
		m_stack_.back()->OnResume();
	}
}

void SceneManager::RequestChange(std::unique_ptr<IScene> scene) {
	m_pendingChange_ = std::move(scene);
	// Request を呼んだ時点ではすぐに切り替えず、ProcessPendingChanges() で実行する想定
}

void SceneManager::RequestPush(std::unique_ptr<IScene> scene) {
	m_pendingPush_ = std::move(scene);
}

void SceneManager::RequestPop() {
	m_pendingPop_ = true;
}

void SceneManager::ProcessPendingChanges() {
	// 優先順位：Change > Pop > Push
	if (m_pendingChange_) {
		ChangeScene(std::move(m_pendingChange_));
	}
	if (m_pendingPop_) {
		PopScene();
		m_pendingPop_ = false;
	}
	if (m_pendingPush_) {
		PushScene(std::move(m_pendingPush_));
	}
}