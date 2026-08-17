#include "GameMenuScene.h"

#include "MiniGame_1_MenuScene.h"
#include "TitleScene.h"

#include "SceneTransition.h"
#include "KeyInput.h"
#include "DxLib.h"

#include <functional>
#include <memory>
#include <vector>

namespace {
    void StartTransition(std::unique_ptr<IScene> next) {
        SceneTransition::Params p;
        p.mode            = SceneTransition::Mode::MaskImage;
        p.durationSec     = 0.6;
        p.maskGraphPath   = "Data/Transition/mask.png";
        p.pixelShaderPath = "Data/Transition/mask_transition.pso";
        SceneTransition::Instance().Start(std::move(next), p, 0.5f);
    }

    struct MenuItem {
        const char* label;
        std::function<std::unique_ptr<IScene>()> factory;
    };

    const std::vector<MenuItem> kItems = {
        { "MiniGame 1", []{ return std::make_unique<MiniGame_1_MenuScene>(); } },
        { "Back",       []{ return std::make_unique<TitleScene>(); } },
    };
}

void GameMenuScene::Start()
{
    _selectedIndex = 0;
    _decided       = false;
}

void GameMenuScene::Update(float /*dt*/)
{
    if (_decided) return;

    auto& key = KeyInput::Instance();
    const int itemCount = static_cast<int>(kItems.size());

    if (key.IsKeyInputTrigger(KEY_INPUT_UP) || key.IsKeyInputTrigger(KEY_INPUT_W)) {
        _selectedIndex = (_selectedIndex - 1 + itemCount) % itemCount;
    }
    if (key.IsKeyInputTrigger(KEY_INPUT_DOWN) || key.IsKeyInputTrigger(KEY_INPUT_S)) {
        _selectedIndex = (_selectedIndex + 1) % itemCount;
    }

    if (key.IsKeyInputTrigger(KEY_INPUT_RETURN) || key.IsKeyInputTrigger(KEY_INPUT_SPACE)) {
        _decided = true;
        StartTransition(kItems[_selectedIndex].factory());
    }

    if (key.IsKeyInputTrigger(KEY_INPUT_ESCAPE)) {
        _decided = true;
        StartTransition(std::make_unique<TitleScene>());
    }
}

void GameMenuScene::Draw()
{
    DrawString(200, 100, "--- Game Menu ---", GetColor(255, 255, 120));
    DrawString(200, 130, "ゲーム一覧", GetColor(200, 200, 200));

    const int itemCount = static_cast<int>(kItems.size());
    for (int i = 0; i < itemCount; ++i) {
        const unsigned int col = (i == _selectedIndex)
            ? GetColor(255, 255, 80)
            : GetColor(200, 200, 200);
        const char* marker = (i == _selectedIndex) ? " > " : "   ";
        DrawFormatString(200, 180 + i * 24, col, "%s%s", marker, kItems[i].label);
    }

    const int guideY = 180 + itemCount * 24 + 12;

    // 操作ガイド
    DrawString(200, 400, "W/S または ↑↓ : 項目移動", GetColor(150, 150, 150));
    DrawString(200, 420, "Space / Enter   : 決定", GetColor(150, 150, 150));
    DrawString(200, 440, "Esc             : タイトルへ戻る", GetColor(150, 150, 150));
}
