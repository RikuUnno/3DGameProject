#include "SceneManager.h"
#include "ObjectManager.h"

// シングルトン取得
SceneManager& SceneManager::Instance() noexcept {
	static SceneManager inst;
	return inst;
}

void SceneManager::Update() {
	if (!_stack.empty()) {
		_stack.back()->Update();
	}
}

void SceneManager::Draw() {
	if (!_stack.empty()) {
		_stack.back()->Draw();
	}
}

void SceneManager::ChangeScene(std::unique_ptr<IScene> scene) {
	//既存シーンを破棄する前に、そのシーンに紐づくオブジェクトを一括回収
	if (!_stack.empty()) {
		ObjectManager::Instance().ReleaseBySceneId(_currentSceneId);
	}

	//既存シーンを破棄
	if (!_stack.empty()) {
		_stack.back()->OnDestroy();
		_stack.clear();
	}

	// シーンIDを進めて、新シーンで Spawnされるオブジェクトに反映
	++_currentSceneId;
	ObjectManager::Instance().SetCurrentSceneId(_currentSceneId);

	if (scene) {
		scene->Awake();
		scene->Start();
		_stack.push_back(std::move(scene));
	}
}

void SceneManager::PushScene(std::unique_ptr<IScene> scene) {
	if (!_stack.empty()) {
		_stack.back()->OnSuspend();
	}
	// Push はシーンスタックを保持するので sceneId は進めない（同一シーン空間として扱う）
	if (scene) {
		scene->Awake();
		scene->Start();
		_stack.push_back(std::move(scene));
	}
}

void SceneManager::PopScene() {
	if (_stack.empty()) return;

	// Popするシーンに紐づく、現在 sceneId のオブジェクトを一括回収
	ObjectManager::Instance().ReleaseBySceneId(_currentSceneId);

	_stack.back()->OnDestroy();
	_stack.pop_back();
	if (!_stack.empty()) {
		_stack.back()->OnResume();
	}
}

void SceneManager::RequestChange(std::unique_ptr<IScene> scene) {
	_pendingChange = std::move(scene);
	// Request を呼んだ時点ではすぐに切り替えず、ProcessPendingChanges() で実行する想定
}

void SceneManager::RequestPush(std::unique_ptr<IScene> scene) {
	_pendingPush = std::move(scene);
}

void SceneManager::RequestPop() {
	_pendingPop = true;
}

void SceneManager::ProcessPendingChanges() {
	// 優先順位：Change > Pop > Push
	if (_pendingChange) {
		ChangeScene(std::move(_pendingChange));
	}
	if (_pendingPop) {
		PopScene();
		_pendingPop = false;
	}
	if (_pendingPush) {
		PushScene(std::move(_pendingPush));
	}
}