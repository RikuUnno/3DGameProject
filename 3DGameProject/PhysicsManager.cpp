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

// 物理更新の入口。
// 大きい dt をそのまま1回で積分すると、薄い壁を飛び越えやすい。
// そのため fixed substep に分割して、判定回数を増やしている。
void PhysicsManager::Update(float dt) {
	if (IsShuttingDown()) return;
	if (dt <= 0.0f) dt = 0.0f;

	// Controller は入力や外部制御を受け持つため、まず元の dt で1回更新する。
	for (auto* c : _controllers) {
		if (!c) continue;
		c->Update(dt);
	}

	// 1サブステップの最大長。
	// 小さくするほどすり抜け耐性は上がるが、計算回数は増える。
	constexpr float kMaxSubStepDt = 1.0f / 120.0f;
	constexpr int kMaxSubSteps = 8;
	const float clampedDt = (std::min)(dt, kMaxSubStepDt * static_cast<float>(kMaxSubSteps));
	int subStepCount = (clampedDt > 0.0f) ? static_cast<int>(std::ceil(clampedDt / kMaxSubStepDt)) : 0;
	if (subStepCount < 1) subStepCount = 1;
	if (subStepCount > kMaxSubSteps) subStepCount = kMaxSubSteps;
	const float stepDt = clampedDt / static_cast<float>(subStepCount);

	// 各サブステップ後の接触から速度補正を行う。
	// 位置の押し戻しは ColliderManager 側、速度の補正は PhysicsManager 側で担当する。
	auto ResolveContacts = [&]() {
		const auto& contacts = ColliderManager::Instance().GetContacts();
		if (contacts.empty()) return;

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

			// inverse mass。
			// 質量0扱いのものは「動かない」とみなし、逆質量を0にする。
			const float invA = (bodyA && bodyA->_enabled && !bodyA->_isKinematic && bodyA->_mass > 1e-6f) ? (1.0f / bodyA->_mass) : 0.0f;
			const float invB = (bodyB && bodyB->_enabled && !bodyB->_isKinematic && bodyB->_mass > 1e-6f) ? (1.0f / bodyB->_mass) : 0.0f;
			const float invSum = invA + invB;
			if (invSum <= 1e-8f) continue;

			VECTOR vA = bodyA ? bodyA->_velocity : VGet(0,0,0);
			VECTOR vB = bodyB ? bodyB->_velocity : VGet(0,0,0);

			// 相対速度の法線成分。
			// (vB - vA)・n が正なら離れる方向、負なら食い込む方向。
			float vrel = (vB.x - vA.x) * ct.normal.x + (vB.y - vA.y) * ct.normal.y + (vB.z - vA.z) * ct.normal.z;
			if (vrel >= 0.0f) {
				// 既に離れる向きなら反発は不要。
				// ただし床接触では小さな下向き速度を消して安定化する。
				if (std::fabs(ct.normal.y) > 0.707f) {
					if (bodyB && !bodyB->_isKinematic) bodyB->_velocity.y = (bodyB->_velocity.y < 0.0f) ? 0.0f : bodyB->_velocity.y;
					if (bodyA && !bodyA->_isKinematic) bodyA->_velocity.y = (bodyA->_velocity.y < 0.0f) ? 0.0f : bodyA->_velocity.y;
				}
				continue;
			}

			float e = 0.0f;
			if (bodyA && bodyB) e = (bodyA->_restitution < bodyB->_restitution) ? bodyA->_restitution : bodyB->_restitution;
			else if (bodyA) e = bodyA->_restitution;
			else if (bodyB) e = bodyB->_restitution;

			// 法線方向インパルス。
			// j = -(1+e) * vrel / (invMassA + invMassB)
			float j = -(1.0f + e) * vrel / invSum;

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

			if (std::fabs(ct.normal.y) > 0.707f) {
				if (bodyB && !bodyB->_isKinematic) bodyB->_velocity.y = (bodyB->_velocity.y < 0.0f) ? 0.0f : bodyB->_velocity.y;
				if (bodyA && !bodyA->_isKinematic) bodyA->_velocity.y = (bodyA->_velocity.y < 0.0f) ? 0.0f : bodyA->_velocity.y;
			}
		}
	};

	for (int step = 0; step < subStepCount; ++step) {
		// 各サブステップで「積分 → 衝突判定 → 速度補正」を回す。
		// これにより1フレーム内の移動量を小さく保ち、トンネリングを抑える。
		for (auto* body : _bodies) {
			if (!body) continue;
			if (!body->_enabled) continue;
			if (body->_isKinematic) continue;
			if (!body->_owner) continue;
			if (!body->_owner->IsActive()) continue;

			if (body->_useGravity) {
				body->_velocity.y += _gravityY * stepDt;
			}

			if (body->_linearDamping > 0.0f) {
				const float damp = std::fmax(0.0f, 1.0f - (body->_linearDamping * stepDt));
				body->_velocity = VScale(body->_velocity, damp);
			}

			// オイラー積分。
			// p(t+dt) = p(t) + v * dt
			VECTOR p = body->_owner->transform.LocalPosition();
			p = VAdd(p, VScale(body->_velocity, stepDt));

			if (_groundPlaneEnabled) {
				if (p.y < _groundPlaneY) {
					p.y = _groundPlaneY;
					if (body->_velocity.y < 0.0f) body->_velocity.y = 0.0f;
				}
			}

			body->_owner->transform.SetLocalPosition(p);
		}

		ColliderManager::Instance().Update();
		ResolveContacts();
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
