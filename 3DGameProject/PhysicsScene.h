#pragma once

#include "SceneTpl.h"
#include <string>

// PhysicsScene
// - 物理挙動を確認するための専用シーン
// - PhysicsDebugClass を複数配置し、落下・反発・摩擦・積み上がりを試せるようにする
class PhysicsScene : public SceneTpl<PhysicsScene> {
public:
	static std::string StaticName() { return "PhysicsScene"; }

	void Start() override;
	void Update(float dtSec) override;
	void Draw() override;
};