#pragma once

#include "SceneTpl.h"
#include <string>

class MenuScene : public SceneTpl<MenuScene> {
public:
 static std::string StaticName() { return "MenuScene"; }

 void Start() override;
 void Update() override;
 void Draw() override;
};
