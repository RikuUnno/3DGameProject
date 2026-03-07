#include "TitleScene.h"
#include "MenuScene.h"
#include "PhysicsScene.h"
#include "SceneManager.h"
#include "SceneTransition.h"
#include "Time.h"
#include "KeyInput.h"
#include "DxLib.h"

#include <memory>

static constexpr double kShowFMessageSeconds = 1.0; // F 表示用の秒数

void TitleScene::Start() {
}

void TitleScene::Update(float /*dt*/) {
	// Enter 入力でメニューへ遷移
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_RETURN)) {
		SceneTransition::Params p;
		p.mode = SceneTransition::Mode::MaskImage;
		p.durationSec = 0.6;
		p.maskGraphPath = "Data/Transition/mask.png";
		p.pixelShaderPath = "Data/Transition/mask_transition.pso";

		SceneTransition::Instance().Start(std::make_unique<MenuScene>(), p, 0.5f);
	}

	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_P)) {
		SceneTransition::Params p;
		p.mode = SceneTransition::Mode::MaskImage;
		p.durationSec = 0.6;
		p.maskGraphPath = "Data/Transition/mask.png";
		p.pixelShaderPath = "Data/Transition/mask_transition.pso";

		SceneTransition::Instance().Start(std::make_unique<PhysicsScene>(), p, 0.5f);
	}
}

void TitleScene::Draw() {
	DrawString(10, 10, "Title Scene - Press Enter", GetColor(255, 255, 255));
	DrawString(10, 30, "Press P : Physics Scene", GetColor(180, 255, 180));
}
