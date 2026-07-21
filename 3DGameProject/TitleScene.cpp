// Scene 一覧
#include "TitleScene.h"
#include "PhysicsScene.h"
#include "CameraScene.h"
#include "CollisionScene.h"
#include "ObjectPoolScene.h"
#include "TransformScene.h"
#include "CcdScene.h"
#include "SkyBoxScene.h"
#include "MiniGame_1_MenuScene.h"

// System
#include "SceneManager.h"
#include "SceneTransition.h"
#include "Time.h"
#include "KeyInput.h"
#include "DxLib.h"

#include <memory>
#include <functional>
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

	// ─────────────────────────────────────────────
	// シーン追加手順:
	//   1. 上部に対応シーンの #include を追加
	//   2. 下の kItems に { "表示名", []{ return make_unique<シーン名>(); } } を追記
	// ─────────────────────────────────────────────
	struct MenuItem {
		const char* label;
		std::function<std::unique_ptr<IScene>()> factory;
	};

	const std::vector<MenuItem> kItems = {
		// デモシーン
		{ "物理デモ",        []{ return std::make_unique<PhysicsScene>();    } },
		{ "カメラデモ",      []{ return std::make_unique<CameraScene>();     } },
		{ "衝突デモ",        []{ return std::make_unique<CollisionScene>();  } },
		{ "ObjectPoolデモ",  []{ return std::make_unique<ObjectPoolScene>(); } },
		{ "Transformデモ",   []{ return std::make_unique<TransformScene>();  } },
		{ "CCDデモ",         []{ return std::make_unique<CcdScene>();        } },
		{ "SkyBoxデモ",      []{ return std::make_unique<SkyBoxScene>();     } },
		// ミニゲーム
		{ "MiniGame 1 - 戦闘機飛行", []{ return std::make_unique<MiniGame_1_MenuScene>(); } },
		// ↑ここに追加
	};
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
}

void TitleScene::Draw()
{
	// タイトル
	DrawString(200, 100, "--- タイトルシーン ---", GetColor(255, 255, 120));
	DrawString(200, 130, "シーンを選択してください", GetColor(200, 200, 200));

	// 項目一覧
	const int itemCount = static_cast<int>(kItems.size());
	for (int i = 0; i < itemCount; ++i) {
		const unsigned int col = (i == _selectedIndex)
			? GetColor(255, 255,  80)
			: GetColor(200, 200, 200);
		const char* marker = (i == _selectedIndex) ? " > " : "   ";
		DrawFormatString(200, 180 + i * 24, col, "%s%s", marker, kItems[i].label);
	}

	// 操作ガイド
	const int guideY = 180 + itemCount * 24 + 12;
	DrawString(200, guideY,      "W/S または ↑↓ : 項目移動", GetColor(150, 150, 150));
	DrawString(200, guideY + 20, "Space / Enter   : 決定",    GetColor(150, 150, 150));
}
