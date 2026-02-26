#include "TitleScene.h"
#include "MenuScene.h"
#include "SceneManager.h"
#include "SceneTransition.h"
#include "Time.h"
#include "KeyInput.h"
#include "DxLib.h"

#include <memory>

static constexpr double kShowFMessageSeconds = 1.0; // F ‰Ÿ‰º‚Å•\¦‚·‚éŠÔ

void TitleScene::Start() {
}

void TitleScene::Update() {
	// Enter ‰Ÿ‰º‚Åƒƒjƒ…[‚Ö‘JˆÚ
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_RETURN)) {
		SceneTransition::Params p;
		p.mode = SceneTransition::Mode::MaskImage;
		p.durationSec = 0.6;
		p.maskGraphPath = "Data/Transition/mask.png";
		p.pixelShaderPath = "Data/Transition/mask_transition.pso";

		SceneTransition::Instance().Start(std::make_unique<MenuScene>(), p, 0.5f);
	}
}

void TitleScene::Draw() {
	DrawString(10, 10, "Title Scene - Press Enter", GetColor(255, 255, 255));
}
