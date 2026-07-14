// Scene 一覧
#include "TitleScene.h"
#include "PhysicsScene.h"
#include "CameraScene.h"
#include "CollisionScene.h"
#include "ObjectPoolScene.h"
#include "TransformScene.h"
#include "CcdScene.h"
#include "MeshDemoScene.h"
#include "SkyBoxScene.h"
#include "MiniGame_1_MenuScene.h"

// System
#include "SceneManager.h"
#include "SceneTransition.h"
#include "Time.h"
#include "KeyInput.h"
#include "DxLib.h"

#include <memory>

static constexpr double kShowFMessageSeconds = 1.0;

void TitleScene::Start() {
}

void TitleScene::Update(float /*dt*/) {
	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_P)) {
		SceneTransition::Params p;
		p.mode = SceneTransition::Mode::MaskImage;
		p.durationSec = 0.6;
		p.maskGraphPath = "Data/Transition/mask.png";
		p.pixelShaderPath = "Data/Transition/mask_transition.pso";
		SceneTransition::Instance().Start(std::make_unique<PhysicsScene>(), p, 0.5f);
	}

	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_C)) {
		SceneTransition::Params p;
		p.mode = SceneTransition::Mode::MaskImage;
		p.durationSec = 0.6;
		p.maskGraphPath = "Data/Transition/mask.png";
		p.pixelShaderPath = "Data/Transition/mask_transition.pso";
		SceneTransition::Instance().Start(std::make_unique<CameraScene>(), p, 0.5f);
	}

	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_H)) {
		SceneTransition::Params p;
		p.mode = SceneTransition::Mode::MaskImage;
		p.durationSec = 0.6;
		p.maskGraphPath = "Data/Transition/mask.png";
		p.pixelShaderPath = "Data/Transition/mask_transition.pso";
		SceneTransition::Instance().Start(std::make_unique<CollisionScene>(), p, 0.5f);
	}

	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_O)) {
		SceneTransition::Params p;
		p.mode = SceneTransition::Mode::MaskImage;
		p.durationSec = 0.6;
		p.maskGraphPath = "Data/Transition/mask.png";
		p.pixelShaderPath = "Data/Transition/mask_transition.pso";
		SceneTransition::Instance().Start(std::make_unique<ObjectPoolScene>(), p, 0.5f);
	}

	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_V)) {
		SceneTransition::Params p;
		p.mode = SceneTransition::Mode::MaskImage;
		p.durationSec = 0.6;
		p.maskGraphPath = "Data/Transition/mask.png";
		p.pixelShaderPath = "Data/Transition/mask_transition.pso";
		SceneTransition::Instance().Start(std::make_unique<TransformScene>(), p, 0.5f);
	}

	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_X)) {
		SceneTransition::Params p;
		p.mode = SceneTransition::Mode::MaskImage;
		p.durationSec = 0.6;
		p.maskGraphPath = "Data/Transition/mask.png";
		p.pixelShaderPath = "Data/Transition/mask_transition.pso";
		SceneTransition::Instance().Start(std::make_unique<CcdScene>(), p, 0.5f);
	}

	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_M)) {
		SceneTransition::Params p;
		p.mode = SceneTransition::Mode::MaskImage;
		p.durationSec = 0.6;
		p.maskGraphPath = "Data/Transition/mask.png";
		p.pixelShaderPath = "Data/Transition/mask_transition.pso";
		SceneTransition::Instance().Start(std::make_unique<MeshDemoScene>(), p, 0.5f);
	}

	if (KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_K)) {
		SceneTransition::Params p;
		p.mode = SceneTransition::Mode::MaskImage;
		p.durationSec = 0.6;
		p.maskGraphPath = "Data/Transition/mask.png";
		p.pixelShaderPath = "Data/Transition/mask_transition.pso";
		SceneTransition::Instance().Start(std::make_unique<SkyBoxScene>(), p, 0.5f);
	}


	// MiniGame

	if(KeyInput::Instance().IsKeyInputTrigger(KEY_INPUT_1)) {
		SceneTransition::Params p;
		p.mode = SceneTransition::Mode::MaskImage;
		p.durationSec = 0.6;
		p.maskGraphPath = "Data/Transition/mask.png";
		p.pixelShaderPath = "Data/Transition/mask_transition.pso";
		SceneTransition::Instance().Start(std::make_unique<MiniGame_1_MenuScene>(), p, 0.5f);
	}
}

void TitleScene::Draw() {
	DrawString(10, 10, "タイトルシーン", GetColor(255, 255, 255));
	DrawString(10, 30, "P : 物理デモ", GetColor(180, 255, 180));
	DrawString(10, 50, "C : カメラデモ", GetColor(180, 220, 255));
	DrawString(10, 70, "H : 衝突デモ", GetColor(255, 220, 180));
	DrawString(10, 90, "O : ObjectPoolデモ", GetColor(220, 255, 180));
	DrawString(10, 110, "V : Transformデモ", GetColor(220, 200, 255));
	DrawString(10, 130, "X : CCDデモ", GetColor(255, 200, 200));
	DrawString(10, 150, "M : Meshデモ", GetColor(180, 255, 220));
	DrawString(10, 170, "K : SkyBoxデモ", GetColor(180, 200, 255));

	// MiniGame
	DrawString(10, 200, "1 : ミニゲーム1", GetColor(255, 255, 0));
}
