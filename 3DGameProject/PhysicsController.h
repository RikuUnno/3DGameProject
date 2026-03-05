#pragma once

// PhysicsController
// - Manager を操作する層（入力/デバッグ/設定変更など）
// -物理そのものの管理は PhysicsManager が担当
class PhysicsController {
public:
	virtual ~PhysicsController() = default;

	// 毎フレーム呼ばれる
	virtual void Update(float dt) =0;
};
