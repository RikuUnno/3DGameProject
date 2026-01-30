#pragma once

#include "SceneTpl.h"
#include <string>

class TitleScene : public SceneTpl<TitleScene> {
public:
 static std::string StaticName() { return "TitleScene"; }

 void Start() override;
 void Update() override;
 void Draw() override;
};
