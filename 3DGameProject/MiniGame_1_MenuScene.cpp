#include "MiniGame_1_MenuScene.h"
#include "MiniGame_1_MenuScene.h"
#include "MiniGame_1_StageScene.h"
#include "GameMenuScene.h"
#include "SceneTransition.h"
#include "KeyInput.h"
#include "DxLib.h"

namespace {
	// トランジション共通設定
	void StartTransition(std::unique_ptr<IScene> next) {
		SceneTransition::Params p;
		p.mode            = SceneTransition::Mode::MaskImage;
		p.durationSec     = 0.6;
		p.maskGraphPath   = "Data/Transition/mask.png";
		p.pixelShaderPath = "Data/Transition/mask_transition.pso";
		SceneTransition::Instance().Start(std::move(next), p, 0.5f);
	}
}

void MiniGame_1_MenuScene::Start()
{
	_selectedIndex = 0;	// 最初は「ゲームスタート」を選択
	_decided       = false;
}

void MiniGame_1_MenuScene::Update(float dtSec)
{
	if (_decided) return; // 決定済みなら操作を受け付けない

	auto& key = KeyInput::Instance();

	// 上下キーで項目移動
	if (key.IsKeyInputTrigger(KEY_INPUT_UP) || key.IsKeyInputTrigger(KEY_INPUT_W)) {
		_selectedIndex = (_selectedIndex - 1 + 2) % 2; // 0, 1 を循環
	}
	if (key.IsKeyInputTrigger(KEY_INPUT_DOWN) || key.IsKeyInputTrigger(KEY_INPUT_S)) {
		_selectedIndex = (_selectedIndex + 1) % 2;
	}

	// Enter または Space で決定
	if (key.IsKeyInputTrigger(KEY_INPUT_RETURN) || key.IsKeyInputTrigger(KEY_INPUT_SPACE)) {
		_decided = true;
		if (_selectedIndex == 0) {
			// ゲームスタート → StageScene へ遷移
			StartTransition(std::make_unique<MiniGame_1_StageScene>());
		} else {
			// 戻る → GameMenuScene へ遷移
				StartTransition(std::make_unique<GameMenuScene>());
		}
	}

	// Esc でタイトルへ戻る
	if (key.IsKeyInputTrigger(KEY_INPUT_ESCAPE)) {
		_decided = true;
		StartTransition(std::make_unique<GameMenuScene>());
	}
}

void MiniGame_1_MenuScene::Draw()
{
	// タイトル
	DrawString(200, 100, "--- MiniGame 1 ---", GetColor(255, 255, 120));
	DrawString(200, 130, "  戦闘機飛行ゲーム", GetColor(200, 200, 200));

	// 項目 0: ゲームスタート
	const unsigned int colStart = (_selectedIndex == 0)
		? GetColor(255, 255,  80)	// 選択中: 黄色
		: GetColor(200, 200, 200);	// 非選択: グレー
	const char* markerStart = (_selectedIndex == 0) ? " > " : "   ";
	DrawFormatString(200, 280, colStart, "%sゲームスタート", markerStart);

	// 項目 1: 戻る
	const unsigned int colBack = (_selectedIndex == 1)
		? GetColor(255, 255,  80)
		: GetColor(200, 200, 200);
	const char* markerBack = (_selectedIndex == 1) ? " > " : "   ";
	DrawFormatString(200, 320, colBack, "%s戻る（メニュー）", markerBack);

	// 操作ガイド
	DrawString(200, 400, "W/S または ↑↓ : 項目移動", GetColor(150, 150, 150));
	DrawString(200, 420, "Space / Enter   : 決定", GetColor(150, 150, 150));
	DrawString(200, 440, "Esc             : タイトルへ戻る", GetColor(150, 150, 150));
}

