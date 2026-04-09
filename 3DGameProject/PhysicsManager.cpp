#include "PhysicsManager.h"
#include "PhysicsManager.h"

#include <algorithm>
#include <cmath>

#include "PhysicsController.h"
#include "PhysicsBody.h"
#include "GameObject.h"
#include "ColliderManager.h"
#include "Collider.h"
#include "Transform.h"
#include "ThreadPool.h"

namespace {
	inline float Dot3(const VECTOR& a, const VECTOR& b) noexcept {
		return a.x * b.x + a.y * b.y + a.z * b.z;
	}

	inline float LenSq(const VECTOR& v) noexcept {
		return Dot3(v, v);
	}

	inline float Len3(const VECTOR& v) noexcept {
		return std::sqrt((std::max)(LenSq(v), 0.0f));
	}

	inline VECTOR SafeNormalize(const VECTOR& v, const VECTOR& fallback = VGet(0, 1, 0)) noexcept {
		const float len = Len3(v);
		if (len > 1e-6f) {
			return VScale(v, 1.0f / len);
		}
		return fallback;
	}

	inline void ClampMagnitude(VECTOR& v, float maxMagnitude) noexcept {
		if (maxMagnitude <= 0.0f) return;
		const float lenSq = LenSq(v);
		const float maxSq = maxMagnitude * maxMagnitude;
		if (lenSq <= maxSq) return;
		const float len = std::sqrt((std::max)(lenSq, 1e-8f));
		v = VScale(v, maxMagnitude / len);
	}

	inline VECTOR TangentFromRelativeVelocity(const VECTOR& relativeVelocity, const VECTOR& normal) noexcept {
		const float vn = Dot3(relativeVelocity, normal);
		return VSub(relativeVelocity, VScale(normal, vn));
	}
}

void PhysicsManager::Shutdown() {
	const bool was = _shuttingDown.exchange(true, std::memory_order_relaxed);
	if (was) return;
	std::lock_guard lk(_mtx);
	_controllers.clear();
	_bodies.clear();
	_accumulator = 0.0f;
}

void PhysicsManager::SetFixedDeltaTime(float fixedDeltaTime) noexcept {
	_fixedDeltaTime = (fixedDeltaTime > 1e-4f) ? fixedDeltaTime : (1.0f / 120.0f);
}

void PhysicsManager::SetMaxSubSteps(int maxSubSteps) noexcept {
	_maxSubSteps = (maxSubSteps > 1) ? maxSubSteps : 1;
}

void PhysicsManager::SetSolverIterations(int solverIterations) noexcept {
	_solverIterations = (solverIterations > 1) ? solverIterations : 1;
}

void PhysicsManager::Update(float dt) {
	if (IsShuttingDown()) return;
	if (dt < 0.0f) dt = 0.0f;

	for (auto* c : _controllers) {
		if (!c) continue;
		c->Update(dt);
	}

	const float maxAccumulatedDt = _fixedDeltaTime * static_cast<float>(_maxSubSteps);
	_accumulator += (std::min)(dt, maxAccumulatedDt);

	int subStepCount = 0;
	while (_accumulator + 1e-6f >= _fixedDeltaTime && subStepCount < _maxSubSteps) {
		StepSimulation(_fixedDeltaTime);
		_accumulator -= _fixedDeltaTime;
		++subStepCount;
	}

	if (_accumulator < 0.0f) {
		_accumulator = 0.0f;
	}
}

void PhysicsManager::StepSimulation(float stepDt) {
	IntegrateBodies(stepDt);
	ColliderManager::Instance().Update(stepDt);

	for (int i = 0; i < _solverIterations; ++i) {
		SolveContacts(stepDt);
	}

	for (auto* body : _bodies) {
		if (!body) continue;
		ApplyBodyConstraints(body);
		UpdateSleepState(body, stepDt);
	}
}

void PhysicsManager::IntegrateBodies(float stepDt) {
	const size_t bodyCount = _bodies.size();
	if (bodyCount == 0) return;

	// 各ボディの積分は独立しているため並列に実行可能
	const bool groundEnabled = _groundPlaneEnabled;
	const float groundY = _groundPlaneY;
	const float gravityY = _gravityY;

	ThreadPool::Instance().ParallelFor(0, bodyCount, [&](size_t idx) {
		PhysicsBody* body = _bodies[idx];
		if (!body) return;
		if (!body->_enabled) return;
		if (!body->_owner) return;
		if (!body->_owner->IsActive()) return;

		body->_previousPosition = body->_owner->transform.LocalPosition();
		body->_previousRotation = body->_owner->transform.LocalRotation();

		if (body->_isKinematic) {
			if (body->_hasMovePositionTarget) {
				body->_owner->transform.SetLocalPosition(body->_movePositionTarget);
				body->_hasMovePositionTarget = false;
			}
			if (body->_hasMoveRotationTarget) {
				body->_owner->transform.SetLocalRotation(body->_moveRotationTarget);
				body->_hasMoveRotationTarget = false;
			}
			body->ClearAccumulators();
			body->_velocity = VGet(0, 0, 0);
			body->_angularVelocity = VGet(0, 0, 0);
			return;
		}

		if (body->_isSleeping && LenSq(body->_force) <= 1e-8f && LenSq(body->_torque) <= 1e-8f) {
			return;
		}
		if (LenSq(body->_force) > 1e-8f || LenSq(body->_torque) > 1e-8f) {
			body->WakeUp();
		}

		const float inverseMass = body->InverseMass();
		if (inverseMass <= 0.0f) {
			body->ClearAccumulators();
			return;
		}

		VECTOR acceleration = VScale(body->_force, inverseMass);
		if (body->_useGravity) {
			acceleration.y += gravityY * body->_gravityScale;
		}
		body->_velocity = VAdd(body->_velocity, VScale(acceleration, stepDt));

		if (!body->_freezeRotation) {
			// angular acceleration = I^{-1} * torque (using diagonal inertia tensor)
			const VECTOR angularAcceleration = body->ApplyInverseInertia(body->_torque);
			body->_angularVelocity = VAdd(body->_angularVelocity, VScale(angularAcceleration, stepDt));
		}

		if (body->_linearDamping > 0.0f) {
			const float linearDamp = 1.0f / (1.0f + body->_linearDamping * stepDt);
			body->_velocity = VScale(body->_velocity, linearDamp);
		}
		if (!body->_freezeRotation && body->_angularDamping > 0.0f) {
			const float angularDamp = 1.0f / (1.0f + body->_angularDamping * stepDt);
			body->_angularVelocity = VScale(body->_angularVelocity, angularDamp);
		}

		ApplyBodyConstraints(body);
		ClampMagnitude(body->_velocity, body->_maxLinearSpeed);
		if (!body->_freezeRotation) {
			ClampMagnitude(body->_angularVelocity, body->_maxAngularSpeed);
		}

		VECTOR position = body->_owner->transform.LocalPosition();
		position = VAdd(position, VScale(body->_velocity, stepDt));
		if (groundEnabled && position.y < groundY) {
			position.y = groundY;
			if (body->_velocity.y < 0.0f) {
				body->_velocity.y = 0.0f;
			}
			// Ground plane contact friction: apply mild angular damping.
			// Must be weak enough to allow gravity to right a tilted box.
			if (!body->_freezeRotation && body->_friction > 0.0f) {
				const float groundFrictionDamp = 1.0f / (1.0f + body->_friction * 0.5f * stepDt);
				body->_angularVelocity = VScale(body->_angularVelocity, groundFrictionDamp);
			}
		}
		body->_owner->transform.SetLocalPosition(position);

		if (!body->_freezeRotation) {
			const float angularSpeed = Len3(body->_angularVelocity);
			if (angularSpeed > 1e-6f) {
				const VECTOR axis = VScale(body->_angularVelocity, 1.0f / angularSpeed);
				const Quaternion deltaRotation = Quaternion::FromAxisAngleRad(axis, angularSpeed * stepDt);
				body->_owner->transform.SetLocalRotation(deltaRotation * body->_owner->transform.LocalRotation());
			}
		}

		body->_hasMovePositionTarget = false;
		body->_hasMoveRotationTarget = false;
		body->ClearAccumulators();
	}, 4); // grainSize=4: 4ボディ以下はシーケンシャル
}

void PhysicsManager::SolveContacts(float stepDt) {
	(void)stepDt;
	const auto& contacts = ColliderManager::Instance().GetContacts();
	if (contacts.empty()) return;

	for (const auto& ct : contacts) {
		if (!ct.a || !ct.b) continue;

		GameObject* ownerA = ct.a->owner;
		GameObject* ownerB = ct.b->owner;
		PhysicsBody* bodyA = ownerA ? FindBodyByOwner(ownerA) : nullptr;
		PhysicsBody* bodyB = ownerB ? FindBodyByOwner(ownerB) : nullptr;

		const float invA = (bodyA && bodyA->IsDynamic() && ownerA && ownerA->IsActive()) ? bodyA->InverseMass() : 0.0f;
		const float invB = (bodyB && bodyB->IsDynamic() && ownerB && ownerB->IsActive()) ? bodyB->InverseMass() : 0.0f;
		const float invSum = invA + invB;
		if (invSum <= 1e-8f) continue;

		const VECTOR normal = SafeNormalize(ct.normal, VGet(0, 1, 0));

		// Relative vectors from body centers to contact point
		const VECTOR centerA = (ownerA) ? ownerA->transform.WorldPosition() : VGet(0, 0, 0);
		const VECTOR centerB = (ownerB) ? ownerB->transform.WorldPosition() : VGet(0, 0, 0);
		const VECTOR rA = VSub(ct.point, centerA);
		const VECTOR rB = VSub(ct.point, centerB);

		// Point velocities: v + omega x r
		const VECTOR velA_linear = bodyA ? bodyA->_velocity : VGet(0, 0, 0);
		const VECTOR velA_angular = bodyA ? VCross(bodyA->_angularVelocity, rA) : VGet(0, 0, 0);
		const VECTOR velB_linear = bodyB ? bodyB->_velocity : VGet(0, 0, 0);
		const VECTOR velB_angular = bodyB ? VCross(bodyB->_angularVelocity, rB) : VGet(0, 0, 0);
		const VECTOR velA_point = VAdd(velA_linear, velA_angular);
		const VECTOR velB_point = VAdd(velB_linear, velB_angular);
		VECTOR relativeVelocity = VSub(velB_point, velA_point);
		const float normalVelocity = Dot3(relativeVelocity, normal);

		float restitution = 0.0f;
		if (bodyA && bodyB) restitution = (std::min)(bodyA->_restitution, bodyB->_restitution);
		else if (bodyA) restitution = bodyA->_restitution;
		else if (bodyB) restitution = bodyB->_restitution;
		if (std::fabs(normalVelocity) < 0.25f) {
			restitution = 0.0f;
		}

		// Effective inverse mass including rotational contribution via inertia tensor:
		// K = 1/mA + 1/mB + n . ((I_A^{-1} (rA x n)) x rA) + n . ((I_B^{-1} (rB x n)) x rB)
		const VECTOR rAxN = VCross(rA, normal);
		const VECTOR rBxN = VCross(rB, normal);

		float angularTermA = 0.0f;
		if (bodyA && invA > 0.0f && !bodyA->_freezeRotation) {
			const VECTOR iInvRAxN = bodyA->ApplyInverseInertia(rAxN);
			angularTermA = Dot3(VCross(iInvRAxN, rA), normal);
		}
		float angularTermB = 0.0f;
		if (bodyB && invB > 0.0f && !bodyB->_freezeRotation) {
			const VECTOR iInvRBxN = bodyB->ApplyInverseInertia(rBxN);
			angularTermB = Dot3(VCross(iInvRBxN, rB), normal);
		}
		const float effectiveInvMass = invSum + angularTermA + angularTermB;
		if (effectiveInvMass <= 1e-8f) continue;

		float normalImpulseMagnitude = 0.0f;
		if (normalVelocity < 0.0f) {
			normalImpulseMagnitude = (-(1.0f + restitution) * normalVelocity) / effectiveInvMass;
			if (normalImpulseMagnitude < 0.0f) {
				normalImpulseMagnitude = 0.0f;
			}
		}

		const VECTOR normalImpulse = VScale(normal, normalImpulseMagnitude);
		if (bodyA && invA > 0.0f) {
			bodyA->_velocity = VSub(bodyA->_velocity, VScale(normalImpulse, invA));
			if (!bodyA->_freezeRotation) {
				bodyA->_angularVelocity = VSub(bodyA->_angularVelocity,
					bodyA->ApplyInverseInertia(VCross(rA, normalImpulse)));
			}
			bodyA->WakeUp();
		}
		if (bodyB && invB > 0.0f) {
			bodyB->_velocity = VAdd(bodyB->_velocity, VScale(normalImpulse, invB));
			if (!bodyB->_freezeRotation) {
				bodyB->_angularVelocity = VAdd(bodyB->_angularVelocity,
					bodyB->ApplyInverseInertia(VCross(rB, normalImpulse)));
			}
			bodyB->WakeUp();
		}

		// Friction (tangential impulse)
		const VECTOR velA_point2 = VAdd(bodyA ? bodyA->_velocity : VGet(0, 0, 0),
			bodyA ? VCross(bodyA->_angularVelocity, rA) : VGet(0, 0, 0));
		const VECTOR velB_point2 = VAdd(bodyB ? bodyB->_velocity : VGet(0, 0, 0),
			bodyB ? VCross(bodyB->_angularVelocity, rB) : VGet(0, 0, 0));
		relativeVelocity = VSub(velB_point2, velA_point2);
		VECTOR tangent = TangentFromRelativeVelocity(relativeVelocity, normal);
		const float tangentLenSq = LenSq(tangent);
		if (tangentLenSq > 1e-8f && normalImpulseMagnitude > 0.0f) {
			tangent = VScale(tangent, 1.0f / std::sqrt(tangentLenSq));

			const VECTOR rAxT = VCross(rA, tangent);
			const VECTOR rBxT = VCross(rB, tangent);
			float angTermTA = 0.0f;
			if (bodyA && invA > 0.0f && !bodyA->_freezeRotation) {
				angTermTA = Dot3(VCross(bodyA->ApplyInverseInertia(rAxT), rA), tangent);
			}
			float angTermTB = 0.0f;
			if (bodyB && invB > 0.0f && !bodyB->_freezeRotation) {
				angTermTB = Dot3(VCross(bodyB->ApplyInverseInertia(rBxT), rB), tangent);
			}
			const float effectiveInvMassT = invSum + angTermTA + angTermTB;

			float tangentImpulseMagnitude = -Dot3(relativeVelocity, tangent) / effectiveInvMassT;
			float friction = 0.0f;
			if (bodyA && bodyB) friction = std::sqrt((std::max)(0.0f, bodyA->_friction) * (std::max)(0.0f, bodyB->_friction));
			else if (bodyA) friction = (std::max)(0.0f, bodyA->_friction);
			else if (bodyB) friction = (std::max)(0.0f, bodyB->_friction);

			const float maxFrictionImpulse = friction * normalImpulseMagnitude;
			tangentImpulseMagnitude = std::clamp(tangentImpulseMagnitude, -maxFrictionImpulse, maxFrictionImpulse);
			const VECTOR tangentImpulse = VScale(tangent, tangentImpulseMagnitude);

			if (bodyA && invA > 0.0f) {
				bodyA->_velocity = VSub(bodyA->_velocity, VScale(tangentImpulse, invA));
				if (!bodyA->_freezeRotation) {
					bodyA->_angularVelocity = VSub(bodyA->_angularVelocity,
						bodyA->ApplyInverseInertia(VCross(rA, tangentImpulse)));
				}
			}
			if (bodyB && invB > 0.0f) {
				bodyB->_velocity = VAdd(bodyB->_velocity, VScale(tangentImpulse, invB));
				if (!bodyB->_freezeRotation) {
					bodyB->_angularVelocity = VAdd(bodyB->_angularVelocity,
						bodyB->ApplyInverseInertia(VCross(rB, tangentImpulse)));
				}
			}
		}

		if (std::fabs(normal.y) > 0.707f) {
			if (normal.y > 0.0f) {
				if (bodyA && invA > 0.0f && bodyA->_velocity.y > 0.0f) {
					bodyA->_velocity.y = 0.0f;
				}
				if (bodyB && invB > 0.0f && bodyB->_velocity.y < 0.0f) {
					bodyB->_velocity.y = 0.0f;
				}
			}
			else {
				if (bodyA && invA > 0.0f && bodyA->_velocity.y < 0.0f) {
					bodyA->_velocity.y = 0.0f;
				}
				if (bodyB && invB > 0.0f && bodyB->_velocity.y > 0.0f) {
					bodyB->_velocity.y = 0.0f;
				}
			}
		}

		// Rolling friction: when in contact, apply a small damping to angular velocity
		// to simulate rolling resistance. This must be weak enough to allow restoring
		// torques (e.g., gravity righting a tilted box) to work properly.
		if (normalImpulseMagnitude > 0.0f) {
			float friction = 0.0f;
			if (bodyA && bodyB) friction = std::sqrt((std::max)(0.0f, bodyA->_friction) * (std::max)(0.0f, bodyB->_friction));
			else if (bodyA) friction = (std::max)(0.0f, bodyA->_friction);
			else if (bodyB) friction = (std::max)(0.0f, bodyB->_friction);

			if (friction > 0.0f) {
				// Use a small rolling friction coefficient (much less than sliding friction)
				const float rollingCoeff = friction * 0.05f;
				auto ApplyRollingFriction = [&](PhysicsBody* body, float invM, const VECTOR& r) {
					if (!body || invM <= 0.0f || body->_freezeRotation) return;
					const float angSpeed = Len3(body->_angularVelocity);
					if (angSpeed <= 1e-7f) return;
					const float rLen = Len3(r);
					const float maxRollingImpulse = rollingCoeff * normalImpulseMagnitude * (std::max)(rLen, 0.01f);
					const VECTOR ii = body->InverseInertiaDiag();
					const float avgInvI = (ii.x + ii.y + ii.z) / 3.0f;
					float angularReduction = maxRollingImpulse * avgInvI;
					angularReduction = (std::min)(angularReduction, angSpeed);
					const float scale = 1.0f - angularReduction / angSpeed;
					body->_angularVelocity = VScale(body->_angularVelocity, (std::max)(scale, 0.0f));
				};
				ApplyRollingFriction(bodyA, invA, rA);
				ApplyRollingFriction(bodyB, invB, rB);
			}
		}

		if (bodyA) ApplyBodyConstraints(bodyA);
		if (bodyB) ApplyBodyConstraints(bodyB);
	}
}

void PhysicsManager::UpdateSleepState(PhysicsBody* body, float stepDt) {
	if (!body) return;
	if (!body->IsDynamic()) return;
	if (body->_hasMovePositionTarget || body->_hasMoveRotationTarget) {
		body->WakeUp();
		return;
	}
	if (LenSq(body->_force) > 1e-8f || LenSq(body->_torque) > 1e-8f) {
		body->WakeUp();
		return;
	}

	const float linearThresholdSq = body->_sleepLinearThreshold * body->_sleepLinearThreshold;
	const float angularThresholdSq = body->_sleepAngularThreshold * body->_sleepAngularThreshold;
	const bool slowEnough = LenSq(body->_velocity) <= linearThresholdSq && LenSq(body->_angularVelocity) <= angularThresholdSq;
	if (!slowEnough) {
		body->WakeUp();
		return;
	}

	body->_sleepTimer += stepDt;
	if (body->_sleepTimer >= body->_sleepTimeThreshold) {
		body->Sleep();
	}
}

void PhysicsManager::ApplyBodyConstraints(PhysicsBody* body) const {
	if (!body) return;
	if (body->_freezeRotation) {
		body->_angularVelocity = VGet(0, 0, 0);
	}
	if (body->_isSleeping) {
		body->_velocity = VGet(0, 0, 0);
		body->_angularVelocity = VGet(0, 0, 0);
	}
}

PhysicsBody* PhysicsManager::FindBodyByOwner(GameObject* owner) const {
	if (!owner) return nullptr;
	for (auto* body : _bodies) {
		if (!body) continue;
		if (body->_owner == owner) return body;
	}
	return nullptr;
}

void PhysicsManager::Register(PhysicsController* controller) {
	if (IsShuttingDown()) return;
	if (!controller) return;
	std::lock_guard lk(_mtx);
	if (std::find(_controllers.begin(), _controllers.end(), controller) != _controllers.end()) return;
	_controllers.push_back(controller);
}

void PhysicsManager::Unregister(PhysicsController* controller) {
	if (IsShuttingDown()) return;
	if (!controller) return;
	std::lock_guard lk(_mtx);
	auto it = std::remove(_controllers.begin(), _controllers.end(), controller);
	_controllers.erase(it, _controllers.end());
}

void PhysicsManager::RegisterBody(PhysicsBody* body) {
	if (IsShuttingDown()) return;
	if (!body) return;
	std::lock_guard lk(_mtx);
	if (std::find(_bodies.begin(), _bodies.end(), body) != _bodies.end()) return;
	_bodies.push_back(body);
	if (body->_owner) {
		body->_previousPosition = body->_owner->transform.LocalPosition();
		body->_previousRotation = body->_owner->transform.LocalRotation();
		// Compute inertia tensor from associated collider shape
		Collider* col = ColliderManager::Instance().FindColliderByOwner(body->_owner);
		body->ComputeInertia(col);
	}
}

void PhysicsManager::UnregisterBody(PhysicsBody* body) {
	if (IsShuttingDown()) return;
	if (!body) return;
	std::lock_guard lk(_mtx);
	auto it = std::remove(_bodies.begin(), _bodies.end(), body);
	_bodies.erase(it, _bodies.end());
}
