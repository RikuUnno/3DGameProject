#include "Debug Class.h"

#include "CapsuleCollider.h"
#include "BoxCollider.h"
#include "ColliderManager.h"
#include "SceneManager.h"
#include "LayerMask.h"
#include "PhysicsManager.h"

// ---------------- DebugHat ----------------

DebugHat::DebugHat() {
	_ownerSceneId = SceneManager::Instance().CurrentSceneId();
	collider_ = std::make_unique<BoxCollider>();
	collider_->owner = this;
	// Debug用途: プレイヤーと同じレイヤー（必要なら専用に）
	collider_->layer = layerMask::PLAYER;
	collider_->mask = mask::ALL;

	// 帽子サイズ
	collider_->_box.halfExtents = VGet(0.25f,0.15f,0.25f);

	// 親子追従の見た目用。押し戻しで位置を変えないようにする
	isStatic = true;

	_physicsBody._owner = this;
	_physicsBody._enabled = false;
}

DebugHat::~DebugHat() {
	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::Instance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
	PhysicsManager::Instance().UnregisterBody(&_physicsBody);
}

void DebugHat::Awake() {}
void DebugHat::Start() {}
void DebugHat::Update() {}

void DebugHat::Draw() {
	if (collider_) collider_->DrawDebug();
}

void DebugHat::End() {}

void DebugHat::OnDestroy() {
	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::Instance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
	PhysicsManager::Instance().UnregisterBody(&_physicsBody);
}

void DebugHat::OnAcquire(const VariantMap& /*params*/) {
	_ownerSceneId = SceneManager::Instance().CurrentSceneId();
	SetActive(true);
	if (!registeredToColliderMgr_ && collider_) {
		ColliderManager::Instance().RegisterCollider(collider_.get());
		registeredToColliderMgr_ = true;
	}

	_physicsBody._owner = this;
	_physicsBody.Reset();
	_physicsBody._enabled = false;
	PhysicsManager::Instance().RegisterBody(&_physicsBody);
}

void DebugHat::OnRelease() {
	// 親子関係は Scene 側で管理するが、安全のため切り離す
	transform.SetParent(nullptr);

	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::Instance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
	PhysicsManager::Instance().UnregisterBody(&_physicsBody);
	SetActive(false);
}

BoxCollider* DebugHat::GetCollider() const noexcept {
	return collider_.get();
}

void DebugHat::OnCollisionStay(Collider* self, Collider* /*other*/) {
	if (!self) return;
	self->SetDebugColor(GetColor(255,0,255));
}

void DebugHat::OnCollisionExit(Collider* self, Collider* /*other*/) {
	if (!self) return;
	self->ClearDebugColor();
}

// ---------------- DebugGround ----------------

DebugGround::DebugGround() {
	_ownerSceneId = SceneManager::Instance().CurrentSceneId();
	collider_ = std::make_unique<BoxCollider>();
	collider_->owner = this;

	// 床は「全レイヤーと当たる」
	collider_->layer = layerMask::DEFAULT;
	collider_->mask = mask::ALL;

	// グリッドの少し下に置く床（大きめ）
	// ※中心は Transformから同期されるので、ここはサイズだけ設定
	collider_->_box.halfExtents = VGet(50.0f,0.5f,50.0f);

	// 動かない固定物
	isStatic = true;

	// 親へのイベント送信は不要なら切れる（必要ならtrueのままでもOK）
	// collider_->sendEventsToOwner = false;
}

DebugGround::~DebugGround() {
	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::Instance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
}

void DebugGround::Awake() {}
void DebugGround::Start() {}
void DebugGround::Update() {}

void DebugGround::Draw() {
	if (collider_) {
		collider_->DrawDebug();
		// collider_->DrawDebugAABB(); //必要なら
	}
}

void DebugGround::End() {}

void DebugGround::OnDestroy() {
	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::Instance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
}

void DebugGround::OnAcquire(const VariantMap& /*params*/) {
	_ownerSceneId = SceneManager::Instance().CurrentSceneId();
	SetActive(true);
	if (!registeredToColliderMgr_ && collider_) {
		ColliderManager::Instance().RegisterCollider(collider_.get());
		registeredToColliderMgr_ = true;
	}
}

void DebugGround::OnRelease() {
	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::Instance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
	SetActive(false);
}

BoxCollider* DebugGround::GetCollider() const noexcept {
	return collider_.get();
}

// ---------------- DebugPlayer ----------------

DebugPlayer::DebugPlayer() {
	_ownerSceneId = SceneManager::Instance().CurrentSceneId();
	collider_ = std::make_unique<CapsuleCollider>();
	collider_->owner = this;
	collider_->layer = layerMask::PLAYER;
	collider_->mask = mask::ALL;

	_physicsBody._owner = this;
	_physicsBody.Reset();
	_physicsBody._enabled = true;
}

DebugPlayer::~DebugPlayer() {
	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::Instance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
	PhysicsManager::Instance().UnregisterBody(&_physicsBody);
}

void DebugPlayer::Awake() {}
void DebugPlayer::Start() {}

void DebugPlayer::Update() {
	//ここでは移動などは行わない（MenuScene用デバッグ）
}

void DebugPlayer::Draw() {
	if (collider_) collider_->DrawDebug();
}

void DebugPlayer::End() {}

void DebugPlayer::OnDestroy() {
	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::Instance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
	PhysicsManager::Instance().UnregisterBody(&_physicsBody);
}

void DebugPlayer::OnAcquire(const VariantMap& /*params*/) {
	_ownerSceneId = SceneManager::Instance().CurrentSceneId();
	SetActive(true);
	if (!registeredToColliderMgr_ && collider_) {
		ColliderManager::Instance().RegisterCollider(collider_.get());
		registeredToColliderMgr_ = true;
	}

	_physicsBody._owner = this;
	_physicsBody.Reset();
	_physicsBody._enabled = true;
	PhysicsManager::Instance().RegisterBody(&_physicsBody);
}

void DebugPlayer::OnRelease() {
	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::Instance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
	PhysicsManager::Instance().UnregisterBody(&_physicsBody);
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
	self->SetDebugColor(GetColor(80,80,255));
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

	// 押し戻しで動かさない
	// isStatic = true;

	_physicsBody._owner = this;
	_physicsBody.Reset();
	_physicsBody._enabled = false;
}

DebugEnemy::~DebugEnemy() {
	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::Instance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
	PhysicsManager::Instance().UnregisterBody(&_physicsBody);
}

void DebugEnemy::Awake() {}
void DebugEnemy::Start() {}

void DebugEnemy::Update() {
	//ここでは移動などは行わない（MenuScene用デバッグ）
}

void DebugEnemy::Draw() {
	if (collider_) collider_->DrawDebug();
}

void DebugEnemy::End() {}

void DebugEnemy::OnDestroy() {
	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::Instance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
	PhysicsManager::Instance().UnregisterBody(&_physicsBody);
}

void DebugEnemy::OnAcquire(const VariantMap& /*params*/) {
	_ownerSceneId = SceneManager::Instance().CurrentSceneId();
	SetActive(true);
	if (!registeredToColliderMgr_ && collider_) {
		ColliderManager::Instance().RegisterCollider(collider_.get());
		registeredToColliderMgr_ = true;
	}

	_physicsBody._owner = this;
	_physicsBody.Reset();
	_physicsBody._enabled = false;
	PhysicsManager::Instance().RegisterBody(&_physicsBody);
}

void DebugEnemy::OnRelease() {
	if (registeredToColliderMgr_ && collider_) {
		ColliderManager::Instance().UnregisterCollider(collider_.get());
		registeredToColliderMgr_ = false;
	}
	PhysicsManager::Instance().UnregisterBody(&_physicsBody);
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
	self->SetDebugColor(GetColor(255,80,80));
}

void DebugEnemy::OnCollisionExit(Collider* self, Collider* /*other*/) {
	_isColliding = false;
	if (!self) return;
	self->ClearDebugColor();
}

void DebugEnemy::OnTriggerStay(Collider* self, Collider* /*other*/) {
	if (!self) return;
	self->SetDebugColor(GetColor(80,80,255));
}

void DebugEnemy::OnTriggerExit(Collider* self, Collider* /*other*/) {
	if (!self) return;
	self->ClearDebugColor();
}