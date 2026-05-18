#include "SceneManager.h"
#include "ObjectManager.h"
#include "CameraManager.h"
#include "PerformanceMonitor.h"
#include "PhysicsMonitor.h"

// シングルトン取得
SceneManager& SceneManager::Instance() noexcept {
	static SceneManager inst;
	return inst;
}

void SceneManager::Update(float dtSec) {
	if (!_stack.empty()) {
		_stack.back()->Update(dtSec);
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
		PerformanceMonitor::Instance().SetCurrentSceneName(_stack.back()->Name());
		PerformanceMonitor::Instance().SetVisible(false);
		PhysicsMonitor::Instance().SetVisible(false);
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
		PerformanceMonitor::Instance().SetCurrentSceneName(_stack.back()->Name());
		PerformanceMonitor::Instance().SetVisible(false);
		PhysicsMonitor::Instance().SetVisible(false);
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
		// PerformanceMonitorにシーン名を設定
		PerformanceMonitor::Instance().SetCurrentSceneName(_stack.back()->Name());
	} else {
		PerformanceMonitor::Instance().SetCurrentSceneName("NoScene");
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