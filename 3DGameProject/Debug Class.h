#pragma once

#include <memory>

#include "GameObject.h"

class CapsuleCollider;

// Player を疑似再現するデバッグ用クラス（ObjectPool 対応）
class DebugPlayer : public GameObject
{
public:
	DebugPlayer();
	virtual ~DebugPlayer() override;

	void Awake() override;
	void Start() override;
	void Update() override;
	void Draw() override;
	void End() override;
	void OnDestroy() override;

	void OnAcquire(const VariantMap& params) override;
	void OnRelease() override;

	CapsuleCollider* GetCollider() const noexcept;

	// Unity風コールバック（cppで実装）
	void OnCollisionEnter(Collider* self, Collider* other) override;
	void OnCollisionStay(Collider* self, Collider* other) override;
	void OnCollisionExit(Collider* self, Collider* other) override;

	void OnTriggerStay(Collider* self, Collider* other) override;
	void OnTriggerExit(Collider* self, Collider* other) override;

	// 衝突中の色変更用
	void SetColliding(bool colliding) noexcept { _isColliding = colliding; }
	bool IsColliding() const noexcept { return _isColliding; }

private:
	std::unique_ptr<CapsuleCollider> collider_;
	bool registeredToColliderMgr_ = false;
	bool _isColliding = false;
};

// Enemy を疑似再現するデバッグ用クラス（ObjectPool 対応）
class DebugEnemy : public GameObject
{
public:
	DebugEnemy();
	virtual ~DebugEnemy() override;

	void Awake() override;
	void Start() override;
	void Update() override;
	void Draw() override;
	void End() override;
	void OnDestroy() override;

	void OnAcquire(const VariantMap& params) override;
	void OnRelease() override;

	CapsuleCollider* GetCollider() const noexcept;

	// Unity風コールバック（cppで実装）
	void OnCollisionEnter(Collider* self, Collider* other) override;
	void OnCollisionStay(Collider* self, Collider* other) override;
	void OnCollisionExit(Collider* self, Collider* other) override;

	void OnTriggerStay(Collider* self, Collider* other) override;
	void OnTriggerExit(Collider* self, Collider* other) override;

private:
	std::unique_ptr<CapsuleCollider> collider_;
	bool registeredToColliderMgr_ = false;
	bool _isColliding = false;
};