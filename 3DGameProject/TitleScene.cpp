#include "TitleScene.h"
#include "MenuScene.h"
#include "SceneManager.h"
#include "Time.h"
#include "KeyInput.h"
#include "DxLib.h"

#include <memory>

static constexpr double kShowFMessageSeconds = 1.0; // F 押下で表示する時間

void TitleScene::Start() {
}

void TitleScene::Update() {
	// Enter 押下でメニューへ遷移（独立して判定）
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_RETURN)) {
		SceneManager::Instance().RequestChange(std::make_unique<MenuScene>());
	}
}

void TitleScene::Draw() {
	DrawString(10, 10, "Title Scene - Press Enter", GetColor(255, 255, 255));
}
