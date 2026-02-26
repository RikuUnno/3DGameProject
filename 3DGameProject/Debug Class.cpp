#include "Debug Class.h"

#include "CapsuleCollider.h"
#include "ColliderManager.h"
#include "SceneManager.h"
#include "LayerMask.h"

// ---------------- DebugPlayer ----------------

DebugPlayer::DebugPlayer() {
	_ownerSceneId = SceneManager::Instance().CurrentSceneId();
	collider_ = std::make_unique<CapsuleCollider>();
	collider_->owner = this;
	collider_->layer = layerMask::PLAYER;
	collider_->mask = mask::ALL;
}

DebugPlayer::~DebugPlayer() {
	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::GetInstance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
}

void DebugPlayer::Awake() {}
void DebugPlayer::Start() {}

void DebugPlayer::Update() {
	//ここでは移動などは行わない（MenuScene側のデモで動かす）
}

void DebugPlayer::Draw() {
	if (collider_) collider_->DrawDebug();
}

void DebugPlayer::End() {}

void DebugPlayer::OnDestroy() {
	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::GetInstance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
}

void DebugPlayer::OnAcquire(const VariantMap& /*params*/) {
	_ownerSceneId = SceneManager::Instance().CurrentSceneId();
	SetActive(true);
	if (!registeredToColliderMgr_ && collider_) {
		ColliderManager::GetInstance().RegisterCollider(collider_.get());
		registeredToColliderMgr_ = true;
	}
}

void DebugPlayer::OnRelease() {
	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::GetInstance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
	SetActive(false);
}

CapsuleCollider* DebugPlayer::GetCollider() const noexcept {
	return collider_.get();
}


void DebugPlayer::OnCollisionEnter(Collider* /*self*/, Collider* /*other*/) {
	_isColliding = true;
}

void DebugPlayer::OnCollisionStay(Collider* self, Collider* /*other*/) {
	_isColliding = true;
	if (!self) return;
	self->SetDebugColor(GetColor(255,80,80));
}

void DebugPlayer::OnCollisionExit(Collider* self, Collider* /*other*/) {
	_isColliding = false;
	if (!self) return;
	self->ClearDebugColor();
}

void DebugPlayer::OnTriggerStay(Collider* self, Collider* /*other*/) {
	if (!self) return;
	self->SetDebugColor(GetColor(80, 80, 255));
}

void DebugPlayer::OnTriggerExit(Collider* self, Collider* /*other*/) {
	if (!self) return;
	self->ClearDebugColor();
}

// ---------------- DebugEnemy ----------------

DebugEnemy::DebugEnemy() {
	_ownerSceneId = SceneManager::Instance().CurrentSceneId();
	collider_ = std::make_unique<CapsuleCollider>();
	collider_->owner = this;
	collider_->layer = layerMask::ENEMY;
	collider_->mask = mask::ALL;
	collider_->isTrigger = true; // プレイヤーと当たるが物理的な反応はしないトリガーにする例
}

DebugEnemy::~DebugEnemy() {
	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::GetInstance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
}

void DebugEnemy::Awake() {}
void DebugEnemy::Start() {}

void DebugEnemy::Update() {
	//ここでは移動などは行わない（MenuScene側のデモで動かす）
}

void DebugEnemy::Draw() {
	if (collider_) collider_->DrawDebug();
}

void DebugEnemy::End() {}

void DebugEnemy::OnDestroy() {
	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::GetInstance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
}

void DebugEnemy::OnAcquire(const VariantMap& /*params*/) {
	_ownerSceneId = SceneManager::Instance().CurrentSceneId();
	SetActive(true);
	if (!registeredToColliderMgr_ && collider_) {
		ColliderManager::GetInstance().RegisterCollider(collider_.get());
		registeredToColliderMgr_ = true;
	}
}

void DebugEnemy::OnRelease() {
	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::GetInstance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
	SetActive(false);
}

CapsuleCollider* DebugEnemy::GetCollider() const noexcept {
	return collider_.get();
}

void DebugEnemy::OnCollisionEnter(Collider* /*self*/, Collider* /*other*/) {
	_isColliding = true;
}

void DebugEnemy::OnCollisionStay(Collider* self, Collider* /*other*/) {
	_isColliding = true;
	if (!self) return;
	self->SetDebugColor(GetColor(255, 80, 80));
}

void DebugEnemy::OnCollisionExit(Collider* self, Collider* /*other*/) {
	_isColliding = false;
	if (!self) return;
	self->ClearDebugColor();
}

void DebugEnemy::OnTriggerStay(Collider* self, Collider* /*other*/) {
	if (!self) return;
	self->SetDebugColor(GetColor(80, 80, 255));
}

void DebugEnemy::OnTriggerExit(Collider* self, Collider* /*other*/) {
	if (!self) return;
	self->ClearDebugColor();
}