#include "ColliderManager.h"

void ColliderManager::Update() {
	// “–‚½‚è”»’è‚ÌXVˆ—‚ğ‚±‚±‚ÉÀ‘•
}

void ColliderManager::DrawDebugAll() {
	for (auto* collider : colliders_) {
		collider->DrawDebug();
	}
}

void ColliderManager::DrawDebugAABBAll() {
	for (auto* collider : colliders_) {
		collider->DrawDebugAABB();
	}
}

void ColliderManager::RegisterCollider(Collider* collider) {
	colliders_.push_back(collider);
}

void ColliderManager::UnregisterCollider(Collider* collider) {
	colliders_.erase(std::remove(colliders_.begin(), colliders_.end(), collider), colliders_.end());
}

void ColliderManager::SpatialPartitioning() {
	// ‹óŠÔ•ªŠ„‚ÌÀ‘•
}

void ColliderManager::CheckLayerMaskCollisions() {
	// Layer/Mask‚É‚æ‚é“–‚½‚è”»’è‚ÌÀ‘•
}

void ColliderManager::CheckAABBCollisions() {
	// AABB“¯m‚Ì“–‚½‚è”»’è‚ÌÀ‘•
}

void ColliderManager::CheckDetailedCollisions() {
	// Ú×‚È“–‚½‚è”»’è‚ÌÀ‘•
}

