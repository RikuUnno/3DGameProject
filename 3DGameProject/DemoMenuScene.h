#pragma once

#include "SceneTpl.h"
#include <string>

class DemoMenuScene : public SceneTpl<DemoMenuScene> {
public:
    static std::string StaticName() { return "DemoMenuScene"; }

    void Start() override;
    void Update(float dt) override;
    void Draw() override;

private:
    int  _selectedIndex = 0;
    bool _decided       = false;
};
