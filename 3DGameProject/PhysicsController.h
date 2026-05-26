#pragma once

// PhysicsController - 物理シーンの制御用抽象クラス
// - Manager を補う役割 (初期化/デバッグ/設定変更など)
// - 各シーンの物理パラメータの管理は PhysicsManager が担当
class PhysicsController {
public:
	virtual ~PhysicsController() = default;

	// 毎フレーム呼び出す
	virtual void Update(float dt) = 0;
};
