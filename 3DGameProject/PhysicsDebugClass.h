#pragma once

#include <memory>
#include <string>

#include "GameObject.h"
#include "PhysicsBody.h"

class SphereCollider;
class BoxCollider;
class CapsuleCollider;
class Collider;

// PhysicsDebugClass
// - 物理確認用の汎用デバッグオブジェクト
// - 1クラスで Box / Sphere / Capsule を切り替えて使えるようにする
// - Scene から VariantMap を受け取り、形状・質量・摩擦などをその場で構成する
class PhysicsDebugClass : public GameObject {
public:
	// 使用するコライダー形状
	enum class ShapeType {
		Box,
		Sphere,
		Capsule,
	};

	PhysicsDebugClass();
	virtual ~PhysicsDebugClass() override;

	void Awake() override;
	void Start() override;
	void Update(float dt) override;
	void Draw() override;
	void End() override;
	void OnDestroy() override;

	// Pool から取得/返却される時の初期化・後片付け
	void OnAcquire(const VariantMap& params) override;
	void OnRelease() override;

	// 現在使用中の Collider / PhysicsBody 取得
	Collider* GetCollider() const noexcept;
	PhysicsBody* GetPhysicsBody() noexcept { return &_physicsBody; }
	const PhysicsBody* GetPhysicsBody() const noexcept { return &_physicsBody; }

protected:
	// 派生クラス既定形状
	virtual ShapeType DefaultShapeType() const noexcept { return ShapeType::Box; }

private:
	// ColliderManager との登録/解除
	void ReleaseCollider_();
	void EnsureColliderRegistered_();

	// 外部パラメータから形状・物理値を構築
	void ConfigureFromParams_(const VariantMap& params);
	void CreateCollider_(ShapeType shapeType);
	void ApplyVisualDefaults_();

	// VariantMap から安全に値を読み出すヘルパー
	static float ParseFloat_(const VariantMap& params, const char* key, float defaultValue);
	static int ParseInt_(const VariantMap& params, const char* key, int defaultValue);
	static bool ParseBool_(const VariantMap& params, const char* key, bool defaultValue);
	static std::string ParseString_(const VariantMap& params, const char* key, const std::string& defaultValue);

private:
	// 形状は切り替え式。未使用形状も保持して再生成コストを抑える
	std::unique_ptr<SphereCollider> _sphereCollider;
	std::unique_ptr<BoxCollider> _boxCollider;
	std::unique_ptr<CapsuleCollider> _capsuleCollider;

	// 物理本体
	PhysicsBody _physicsBody{};

	// 管理状態
	bool _registeredToColliderManager = false;
	ShapeType _shapeType = ShapeType::Box;

	// デバッグ描画用の補助設定
	unsigned int _drawColor = 0;
	float _drawRadius = 0.5f;
	VECTOR _drawHalfExtents = VGet(0.5f, 0.5f, 0.5f);
	float _drawHeight = 2.0f;
};

// PhysicsDebugBox
// - Box 専用の物理デバッグオブジェクト
class PhysicsDebugBox : public PhysicsDebugClass {
public:
	static std::string StaticPoolKey() { return "PhysicsDebugBox"; }

protected:
	ShapeType DefaultShapeType() const noexcept override { return ShapeType::Box; }
};

// PhysicsDebugSphere
// - Sphere 専用の物理デバッグオブジェクト
class PhysicsDebugSphere : public PhysicsDebugClass {
public:
	static std::string StaticPoolKey() { return "PhysicsDebugSphere"; }

protected:
	ShapeType DefaultShapeType() const noexcept override { return ShapeType::Sphere; }
};

// PhysicsDebugCapsule
// - Capsule 専用の物理デバッグオブジェクト
class PhysicsDebugCapsule : public PhysicsDebugClass {
public:
	static std::string StaticPoolKey() { return "PhysicsDebugCapsule"; }

protected:
	ShapeType DefaultShapeType() const noexcept override { return ShapeType::Capsule; }
};