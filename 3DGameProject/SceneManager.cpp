#include "SceneManager.h"
#include "ObjectManager.h"
#include "CameraManager.h"

// ƒVƒ“ƒOƒ‹ƒgƒ“Žæ“¾
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
	if (!_stack.empty()) {
		_stack.back()->End();
	}

	if (!_stack.empty()) {
		ObjectManager::Instance().ReleaseBySceneId(_currentSceneId);
		CameraManager::Instance().ReleaseBySceneId(_currentSceneId);
	}

	if (!_stack.empty()) {
		_stack.back()->OnDestroy();
		_stack.clear();
	}

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
	if (scene) {
		scene->Awake();
		scene->Start();
		_stack.push_back(std::move(scene));
	}
}

void SceneManager::PopScene() {
	if (_stack.empty()) return;

	_stack.back()->End();

	ObjectManager::Instance().ReleaseBySceneId(_currentSceneId);
	CameraManager::Instance().ReleaseBySceneId(_currentSceneId);

	_stack.back()->OnDestroy();
	_stack.pop_back();
	if (!_stack.empty()) {
		_stack.back()->OnResume();
	}
}

void SceneManager::RequestChange(std::unique_ptr<IScene> scene) {
	_pendingChange = std::move(scene);
}

void SceneManager::RequestPush(std::unique_ptr<IScene> scene) {
	_pendingPush = std::move(scene);
}

void SceneManager::RequestPop() {
	_pendingPop = true;
}

void SceneManager::ProcessPendingChanges() {
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