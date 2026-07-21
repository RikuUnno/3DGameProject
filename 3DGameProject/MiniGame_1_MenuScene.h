#pragma once
#pragma once
#include "SceneTpl.h"
#include <string>

// MiniGame_1 のメニューシーン（ゲームスタート / 戻る を選択）
class MiniGame_1_MenuScene : public SceneTpl<MiniGame_1_MenuScene>
{
public:
	static std::string StaticName() { return "MiniGame_1_MenuScene"; }

	void Start() override;
	void Update(float dtSec) override;
	void Draw() override;

private:
	// メニュー選択状態
	int  _selectedIndex = 0;	// 現在選択中の項目（0=Start, 1=Back）
	bool _decided       = false;// 決定済みフラグ（連続入力防止）
};