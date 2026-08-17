// Scene
#include "TitleScene.h"
#include "GameMenuScene.h"
#include "DemoMenuScene.h"

// System
#include "SceneManager.h"
#include "SceneTransition.h"
#include "Time.h"
#include "KeyInput.h"
#include "DxLib.h"

#include <memory>

namespace {
	void StartTransition(std::unique_ptr<IScene> next) {
		SceneTransition::Params p;
		p.mode            = SceneTransition::Mode::MaskImage;
		p.durationSec     = 0.6;
		p.maskGraphPath   = "Data/Transition/mask.png";
		p.pixelShaderPath = "Data/Transition/mask_transition.pso";
		SceneTransition::Instance().Start(std::move(next), p, 0.5f);
	}
}

void TitleScene::Start()
{
	_selectedIndex = 0;
	_decided       = false;
}

void TitleScene::Update(float /*dt*/)
{
	if (_decided) return;

	auto& key = KeyInput::Instance();

	constexpr int itemCount = 2;
	if (key.IsKeyInputTrigger(KEY_INPUT_UP) || key.IsKeyInputTrigger(KEY_INPUT_W)) {
		_selectedIndex = (_selectedIndex - 1 + itemCount) % itemCount;
	}
	if (key.IsKeyInputTrigger(KEY_INPUT_DOWN) || key.IsKeyInputTrigger(KEY_INPUT_S)) {
		_selectedIndex = (_selectedIndex + 1) % itemCount;
	}

	if (key.IsKeyInputTrigger(KEY_INPUT_RETURN) || key.IsKeyInputTrigger(KEY_INPUT_SPACE)) {
		_decided = true;
		if (_selectedIndex == 0) {
			StartTransition(std::make_unique<GameMenuScene>());
		} else {
			StartTransition(std::make_unique<DemoMenuScene>());
		}
	}
}

void TitleScene::Draw()
{
	DrawString(200, 100, "--- Title Scene ---", GetColor(255, 255, 120));
	DrawString(200, 130, "Select menu", GetColor(200, 200, 200));

	const unsigned int colGame = (_selectedIndex == 0)
		? GetColor(255, 255, 80)
		: GetColor(200, 200, 200);
	const char* markerGame = (_selectedIndex == 0) ? " > " : "   ";
	DrawFormatString(200, 180, colGame, "%sGame Menu", markerGame);

	const unsigned int colDemo = (_selectedIndex == 1)
		? GetColor(255, 255, 80)
		: GetColor(200, 200, 200);
	const char* markerDemo = (_selectedIndex == 1) ? " > " : "   ";
	DrawFormatString(200, 204, colDemo, "%sDemo Menu", markerDemo);

	// ëÄçÏÉKÉCÉh
	DrawString(200, 400, "W/S Ç‹ÇΩÇÕ Å™Å´ : çÄñ⁄à⁄ìÆ", GetColor(150, 150, 150));
	DrawString(200, 420, "Space / Enter   : åàíË", GetColor(150, 150, 150));
}
