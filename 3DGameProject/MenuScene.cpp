#include "MenuScene.h"
#include "TitleScene.h"
#include "DxLib.h"
#include "KeyInput.h"
#include "SceneManager.h"

#include "ObjectFactory.h"
#include "ObjectController.h"
#include "ObjectManager.h"
#include "GameObject.h"

#include <memory>
#include <string>

namespace {
	// このシーン内だけで動く「生成/プール」デモ用の簡易 GameObject
	class DemoObject final : public GameObject {
	public:
		void OnAcquire(const VariantMap& params) override {
			label_ = "DemoObject";
			auto it = params.find("label");
			if (it != params.end()) label_ = it->second;
		}

		void OnRelease() override {
			label_.clear();
		}

		void Draw() override {
			DrawString(10, 40, label_.c_str(), GetColor(200, 255, 200));
		}

	private:
		std::string label_;
	};

	bool g_demoInitialized = false;
	ObjectController g_controller;
	int g_demoCreateCount = 0;
}

void MenuScene::Start() {
	//1呼び出しで完結：Factory登録 + Pool登録(初回のみ) + Spawn + Controller登録
	g_controller.SpawnAuto("DemoObject", [](const VariantMap&) {
		++g_demoCreateCount;
		return std::make_unique<DemoObject>();
		}, 8, { {"label", "DemoObject生成（シーン開始）"} }
			);
}

void MenuScene::Update() {
	// シーン内で生成したオブジェクトの Update をまとめて実行
	g_controller.UpdateAll();

	// Spaceで「このシーンが生成したものを全部返却」してからシーン遷移
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_SPACE)) {
		g_controller.ReleaseAll();
		SceneManager::Instance().ChangeScene(std::make_unique<TitleScene>());
	}
}

void MenuScene::Draw() {
	DrawString(10, 10, "メニューシーン - Spaceで戻る", GetColor(255, 255, 255));
	DrawString(10, 25, "[デモ] ObjectController: Spawn/Update/Draw/ReleaseAll", GetColor(180, 255, 180));
	DrawFormatString(10, 70, GetColor(255, 255, 0), "[デモ] Factory new回数: %d", g_demoCreateCount);

	// シーン内で生成したオブジェクトの Draw をまとめて実行
	g_controller.DrawAll();

	// ObjectManager のデバッグ表示は Main 側で全シーン共通描画する
}