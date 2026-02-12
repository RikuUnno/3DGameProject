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

private:
	std::unique_ptr<CapsuleCollider> collider_;
	bool registeredToColliderMgr_ = false;
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

private:
	std::unique_ptr<CapsuleCollider> collider_;
	bool registeredToColliderMgr_ = false;
};