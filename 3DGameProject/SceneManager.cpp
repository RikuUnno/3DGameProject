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

// フレームごとの呼び出し
void SceneManager::Update(float dtSec) {
	if (!_stack.empty()) {
		_stack.back()->Update(dtSec);
	}
}

// 描画呼び出し
void SceneManager::Draw() {
	if (!_stack.empty()) {
		_stack.back()->Draw();
	}
}

// 即時切替（呼び出し直後に安全な場合に使用）
void SceneManager::ChangeScene(std::unique_ptr<IScene> scene) {
	if (!_stack.empty()) {		// 現在のシーンがあれば終了処理
		_stack.back()->End();
	}

	if (!_stack.empty()) {		// シーン切替前のオブジェクトとカメラを一括解放
		ObjectManager::Instance().ReleaseBySceneId(_currentSceneId);	// 現在のシーンIDに紐づくオブジェクトをすべて解放
		CameraManager::Instance().ReleaseBySceneId(_currentSceneId);	// 現在のシーンIDに紐づくカメラをすべて解放
	}

	if (!_stack.empty()) {		// 現在のシーンがあれば破棄処理
		_stack.back()->OnDestroy();
		_stack.clear();
	}

	// シーン切替ごとにシーンIDを加算していく
	++_currentSceneId;
	ObjectManager::Instance().SetCurrentSceneId(_currentSceneId);

	if (scene) {	// 新しいシーンがあれば生成・開始処理
		scene->Awake();																// 生成直後の処理
		scene->Start();																// 開始直後の処理
		_stack.push_back(std::move(scene));											// スタックに新しいシーンをプッシュ
		PerformanceMonitor::Instance().SetCurrentSceneName(_stack.back()->Name());	// PerformanceMonitorにシーン名を設定
		PerformanceMonitor::Instance().SetVisible(false);							// デフォルトは非表示
		PhysicsMonitor::Instance().SetVisible(false);								// デフォルトは非表示
	}
}

// push運用（任意）[pushしたシーンは pop されるまでスタック上に残り続ける。pop されるまでは Update/Draw の呼び出し対象になる]
void SceneManager::PushScene(std::unique_ptr<IScene> scene) {
	if (!_stack.empty()) {
		_stack.back()->OnSuspend();
	}
	if (scene) {
		scene->Awake();																// 生成直後の処理
		scene->Start();																// 開始直後の処理
		_stack.push_back(std::move(scene));											// スタックに新しいシーンをプッシュ
		PerformanceMonitor::Instance().SetCurrentSceneName(_stack.back()->Name());	// PerformanceMonitorにシーン名を設定
		PerformanceMonitor::Instance().SetVisible(false);							// デフォルトは非表示
		PhysicsMonitor::Instance().SetVisible(false);								// デフォルトは非表示
	}
}

// pop運用（任意）[pop されたシーンはスタックから削除され、以降 Update/Draw の呼び出し対象から外れる]
void SceneManager::PopScene() {
	if (_stack.empty()) return;

	_stack.back()->End();

	ObjectManager::Instance().ReleaseBySceneId(_currentSceneId);					// 現在のシーンIDに紐づくオブジェクトをすべて解放
	CameraManager::Instance().ReleaseBySceneId(_currentSceneId);					// 現在のシーンIDに紐づくカメラをすべて解放

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

// 保留中リクエスト（Update 中など安全でないタイミングで呼ぶ）
void SceneManager::RequestChange(std::unique_ptr<IScene> scene) {
	_pendingChange = std::move(scene);
}

// 保留中リクエスト（Update 中など安全でないタイミングで呼ぶ）
void SceneManager::RequestPush(std::unique_ptr<IScene> scene) {
	_pendingPush = std::move(scene);
}

// 保留中リクエスト（Update 中など安全でないタイミングで呼ぶ）
void SceneManager::RequestPop() {
	_pendingPop = true;
}

// 保留中リクエストの処理（Update 中など安全でないタイミングで呼ぶ）	
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