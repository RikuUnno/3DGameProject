#include "PachinkoGameMenu.h"

#include "PachinkoGameStage.h"
#include "GameMenuScene.h"
#include "SceneTransition.h"
#include "KeyInput.h"
#include "DxLib.h"

namespace {
	// シーン遷移開始
    void StartTransition(std::unique_ptr<IScene> next) {
        SceneTransition::Params p;
        p.mode = SceneTransition::Mode::MaskImage;
        p.durationSec = 0.6;
        p.maskGraphPath = "Data/Transition/mask.png";
        p.pixelShaderPath = "Data/Transition/mask_transition.pso";
        SceneTransition::Instance().Start(std::move(next), p, 0.5f);
    }
}

void PachinkoGame_MenuScene::Start() {
    _selectedIndex = 0;
    _decided = false;
}

void PachinkoGame_MenuScene::Update(float /*dtSec*/) {
    if (_decided) return;
    auto& key = KeyInput::Instance();

    if (key.IsKeyInputTrigger(KEY_INPUT_UP) || key.IsKeyInputTrigger(KEY_INPUT_W)) {
        _selectedIndex = (_selectedIndex - 1 + 2) % 2;
    }
    if (key.IsKeyInputTrigger(KEY_INPUT_DOWN) || key.IsKeyInputTrigger(KEY_INPUT_S)) {
        _selectedIndex = (_selectedIndex + 1) % 2;
    }

    if (key.IsKeyInputTrigger(KEY_INPUT_RETURN) || key.IsKeyInputTrigger(KEY_INPUT_SPACE)) {
        _decided = true;
        if (_selectedIndex == 0) StartTransition(std::make_unique<PachinkoGame_StageScene>());
        else StartTransition(std::make_unique<GameMenuScene>());
    }

    if (key.IsKeyInputTrigger(KEY_INPUT_ESCAPE)) {
        _decided = true;
        StartTransition(std::make_unique<GameMenuScene>());
    }
}

void PachinkoGame_MenuScene::Draw() {
    DrawString(200, 100, "--- Pachinko Game ---", GetColor(255, 255, 120));
    DrawString(200, 130, "パチンコシュミレーション", GetColor(200, 200, 200));

    const unsigned int colStart = (_selectedIndex == 0) ? GetColor(255, 255, 80) : GetColor(200, 200, 200);
    const unsigned int colBack  = (_selectedIndex == 1) ? GetColor(255, 255, 80) : GetColor(200, 200, 200);
    DrawFormatString(200, 280, colStart, "%sStart", (_selectedIndex == 0) ? " > " : "   ");
    DrawFormatString(200, 320, colBack,  "%sBack",  (_selectedIndex == 1) ? " > " : "   ");

    // 操作ガイド
    DrawString(200, 400, "W/S または ↑↓ : 項目移動", GetColor(150, 150, 150));
    DrawString(200, 420, "Space / Enter   : 決定", GetColor(150, 150, 150));
    DrawString(200, 440, "Esc             : タイトルへ戻る", GetColor(150, 150, 150));
}
