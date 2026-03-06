#include "PhysicsManager.h"

#include <algorithm>
#include <cmath>

#include "PhysicsController.h"
#include "PhysicsBody.h"
#include "GameObject.h"
#include "ColliderManager.h"

void PhysicsManager::Shutdown() {
	const bool was = _shuttingDown.exchange(true, std::memory_order_relaxed);
	if (was) return;
	_controllers.clear();
	_bodies.clear();
}

void PhysicsManager::Update(float dt) {
	if (IsShuttingDown()) return;
	if (dt <= 0.0f) dt = 0.0f;

	//1) Controller更新（入力・デバッグ操作など）
	for (auto* c : _controllers) {
		if (!c) continue;
		c->Update(dt);
	}

	//2)物理（最小）: 重力 +速度積分
	for (auto* body : _bodies) {
		if (!body) continue;
		if (!body->_enabled) continue;
		if (body->_isKinematic) continue;
		if (!body->_owner) continue;
		if (!body->_owner->IsActive()) continue;

		if (body->_useGravity) {
			body->_velocity.y += _gravityY * dt;
		}

		// linear damping（簡易）
		if (body->_linearDamping > 0.0f) {
			const float damp = std::fmax(0.0f, 1.0f - (body->_linearDamping * dt));
			body->_velocity = VScale(body->_velocity, damp);
		}

		// integrate (local)
		VECTOR p = body->_owner->transform.LocalPosition();
		p = VAdd(p, VScale(body->_velocity, dt));

		// --- temporary: ground plane ---
		if (_groundPlaneEnabled) {
			if (p.y < _groundPlaneY) {
				p.y = _groundPlaneY;
				if (body->_velocity.y < 0.0f) body->_velocity.y = 0.0f;
			}
		}

		body->_owner->transform.SetLocalPosition(p);
	}

	// コライダー更新（押し戻しは ColliderManager 側で行う）
	ColliderManager::Instance().Update();

	// 接触情報に基づく速度修正（簡易インパルス）
	const auto& contacts = ColliderManager::Instance().GetContacts();
	if (!contacts.empty()) {
		auto FindBodyByOwner = [&](GameObject* owner) -> PhysicsBody* {
			for (auto* b : _bodies) {
				if (!b) continue;
				if (b->_owner == owner) return b;
			}
			return nullptr;
		};

		for (const auto& ct : contacts) {
			if (!ct.a || !ct.b) continue;
			GameObject* oa = ct.a->owner;
			GameObject* ob = ct.b->owner;
			if (!oa && !ob) continue;

			PhysicsBody* bodyA = oa ? FindBodyByOwner(oa) : nullptr;
			PhysicsBody* bodyB = ob ? FindBodyByOwner(ob) : nullptr;

			// inverse mass
			const float invA = (bodyA && bodyA->_enabled && !bodyA->_isKinematic && bodyA->_mass > 1e-6f) ? (1.0f / bodyA->_mass) : 0.0f;
			const float invB = (bodyB && bodyB->_enabled && !bodyB->_isKinematic && bodyB->_mass > 1e-6f) ? (1.0f / bodyB->_mass) : 0.0f;
			const float invSum = invA + invB;
			if (invSum <= 1e-8f) continue;

			// 現在の速度
			VECTOR vA = bodyA ? bodyA->_velocity : VGet(0,0,0);
			VECTOR vB = bodyB ? bodyB->_velocity : VGet(0,0,0);

			// 相対速度の法線成分 v_rel = (vB - vA)・n
			float vrel = (vB.x - vA.x) * ct.normal.x + (vB.y - vA.y) * ct.normal.y + (vB.z - vA.z) * ct.normal.z;
			if (vrel >= 0.0f) {
				// すでに離れている。だが接触がある（押し戻しで位置補正済み）場合、地面判定で小さな下向き速度を抑える
				if (std::fabs(ct.normal.y) > 0.707f) {
					if (bodyB && !bodyB->_isKinematic) bodyB->_velocity.y = (bodyB->_velocity.y < 0.0f) ? 0.0f : bodyB->_velocity.y;
					if (bodyA && !bodyA->_isKinematic) bodyA->_velocity.y = (bodyA->_velocity.y < 0.0f) ? 0.0f : bodyA->_velocity.y;
				}
				continue;
			}

			// 反発係数
			float e = 0.0f;
			if (bodyA && bodyB) e = (bodyA->_restitution < bodyB->_restitution) ? bodyA->_restitution : bodyB->_restitution;
			else if (bodyA) e = bodyA->_restitution;
			else if (bodyB) e = bodyB->_restitution;

			// インパルス
			float j = -(1.0f + e) * vrel / invSum;

			// 速度へ適用
			if (bodyA) {
				bodyA->_velocity.x -= ct.normal.x * j * invA;
				bodyA->_velocity.y -= ct.normal.y * j * invA;
				bodyA->_velocity.z -= ct.normal.z * j * invA;
			}
			if (bodyB) {
				bodyB->_velocity.x += ct.normal.x * j * invB;
				bodyB->_velocity.y += ct.normal.y * j * invB;
				bodyB->_velocity.z += ct.normal.z * j * invB;
			}

			// 地面寄りの接触なら垂直速度を抑える（安定化）
			if (std::fabs(ct.normal.y) > 0.707f) {
				// 下向き速度をゼロにすることで "落ち続ける" 問題を回避
				if (bodyB && !bodyB->_isKinematic) bodyB->_velocity.y = (bodyB->_velocity.y < 0.0f) ? 0.0f : bodyB->_velocity.y;
				if (bodyA && !bodyA->_isKinematic) bodyA->_velocity.y = (bodyA->_velocity.y < 0.0f) ? 0.0f : bodyA->_velocity.y;
			}
		}
	}
}

void PhysicsManager::Register(PhysicsController* controller) {
	if (IsShuttingDown()) return;
	if (!controller) return;
	if (std::find(_controllers.begin(), _controllers.end(), controller) != _controllers.end()) return;
	_controllers.push_back(controller);
}

void PhysicsManager::Unregister(PhysicsController* controller) {
	if (IsShuttingDown()) return;
	if (!controller) return;
	auto it = std::remove(_controllers.begin(), _controllers.end(), controller);
	_controllers.erase(it, _controllers.end());
}

void PhysicsManager::RegisterBody(PhysicsBody* body) {
	if (IsShuttingDown()) return;
	if (!body) return;
	if (std::find(_bodies.begin(), _bodies.end(), body) != _bodies.end()) return;
	_bodies.push_back(body);
}

void PhysicsManager::UnregisterBody(PhysicsBody* body) {
	if (IsShuttingDown()) return;
	if (!body) return;
	auto it = std::remove(_bodies.begin(), _bodies.end(), body);
	_bodies.erase(it, _bodies.end());
}
