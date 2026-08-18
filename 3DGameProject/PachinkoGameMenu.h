#pragma once

#include "SceneTpl.h"
#include <string>

class PachinkoGame_MenuScene : public SceneTpl<PachinkoGame_MenuScene>
{
public:
	static std::string StaticName() { return "PachinkoGame_MenuScene"; }

	void Start() override;
	void Update(float dtSec) override;
	void Draw() override;

private:
	int  _selectedIndex = 0;	// 現在選択中の項目（0=Start, 1=Back）
	bool _decided = false;		// 決定済みフラグ（連続入力防止）
};