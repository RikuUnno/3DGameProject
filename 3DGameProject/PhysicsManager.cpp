#include "PhysicsManager.h"

#include <algorithm>
#include <cmath>

#include "PhysicsController.h"
#include "PhysicsBody.h"
#include "GameObject.h"
#include "ColliderManager.h"
#include "Transform.h"

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
	for (auto* body : _bodies) {
		if (!body) continue;
		if (!body->_enabled) continue;
		if (!body->_owner) continue;
		if (!body->_owner->IsActive()) continue;

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
			continue;
		}

		if (body->_isSleeping && LenSq(body->_force) <= 1e-8f && LenSq(body->_torque) <= 1e-8f) {
			continue;
		}
		if (LenSq(body->_force) > 1e-8f || LenSq(body->_torque) > 1e-8f) {
			body->WakeUp();
		}

		const float inverseMass = body->InverseMass();
		if (inverseMass <= 0.0f) {
			body->ClearAccumulators();
			continue;
		}

		VECTOR acceleration = VScale(body->_force, inverseMass);
		if (body->_useGravity) {
			acceleration.y += _gravityY * body->_gravityScale;
		}
		body->_velocity = VAdd(body->_velocity, VScale(acceleration, stepDt));

		if (!body->_freezeRotation) {
			const VECTOR angularAcceleration = VScale(body->_torque, inverseMass);
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
		if (_groundPlaneEnabled && position.y < _groundPlaneY) {
			position.y = _groundPlaneY;
			if (body->_velocity.y < 0.0f) {
				body->_velocity.y = 0.0f;
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
	}
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
		const VECTOR velocityA = bodyA ? bodyA->_velocity : VGet(0, 0, 0);
		const VECTOR velocityB = bodyB ? bodyB->_velocity : VGet(0, 0, 0);
		VECTOR relativeVelocity = VSub(velocityB, velocityA);
		const float normalVelocity = Dot3(relativeVelocity, normal);

		float restitution = 0.0f;
		if (bodyA && bodyB) restitution = (std::min)(bodyA->_restitution, bodyB->_restitution);
		else if (bodyA) restitution = bodyA->_restitution;
		else if (bodyB) restitution = bodyB->_restitution;
		if (std::fabs(normalVelocity) < 0.25f) {
			restitution = 0.0f;
		}

		float normalImpulseMagnitude = 0.0f;
		if (normalVelocity < 0.0f) {
			normalImpulseMagnitude = (-(1.0f + restitution) * normalVelocity) / invSum;
			if (normalImpulseMagnitude < 0.0f) {
				normalImpulseMagnitude = 0.0f;
			}
		}

		const VECTOR normalImpulse = VScale(normal, normalImpulseMagnitude);
		if (bodyA && invA > 0.0f) {
			bodyA->_velocity = VSub(bodyA->_velocity, VScale(normalImpulse, invA));
			bodyA->WakeUp();
		}
		if (bodyB && invB > 0.0f) {
			bodyB->_velocity = VAdd(bodyB->_velocity, VScale(normalImpulse, invB));
			bodyB->WakeUp();
		}

		relativeVelocity = VSub(bodyB ? bodyB->_velocity : VGet(0, 0, 0), bodyA ? bodyA->_velocity : VGet(0, 0, 0));
		VECTOR tangent = TangentFromRelativeVelocity(relativeVelocity, normal);
		const float tangentLenSq = LenSq(tangent);
		if (tangentLenSq > 1e-8f && normalImpulseMagnitude > 0.0f) {
			tangent = VScale(tangent, 1.0f / std::sqrt(tangentLenSq));
			float tangentImpulseMagnitude = -Dot3(relativeVelocity, tangent) / invSum;
			float friction = 0.0f;
			if (bodyA && bodyB) friction = std::sqrt((std::max)(0.0f, bodyA->_friction) * (std::max)(0.0f, bodyB->_friction));
			else if (bodyA) friction = (std::max)(0.0f, bodyA->_friction);
			else if (bodyB) friction = (std::max)(0.0f, bodyB->_friction);

			const float maxFrictionImpulse = friction * normalImpulseMagnitude;
			tangentImpulseMagnitude = std::clamp(tangentImpulseMagnitude, -maxFrictionImpulse, maxFrictionImpulse);
			const VECTOR tangentImpulse = VScale(tangent, tangentImpulseMagnitude);

			if (bodyA && invA > 0.0f) {
				bodyA->_velocity = VSub(bodyA->_velocity, VScale(tangentImpulse, invA));
			}
			if (bodyB && invB > 0.0f) {
				bodyB->_velocity = VAdd(bodyB->_velocity, VScale(tangentImpulse, invB));
			}
		}

		if (std::fabs(normal.y) > 0.707f) {
			if (bodyA && invA > 0.0f && bodyA->_velocity.y < 0.0f) {
				bodyA->_velocity.y = 0.0f;
			}
			if (bodyB && invB > 0.0f && bodyB->_velocity.y < 0.0f) {
				bodyB->_velocity.y = 0.0f;
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
	if (body->_owner) {
		body->_previousPosition = body->_owner->transform.LocalPosition();
		body->_previousRotation = body->_owner->transform.LocalRotation();
	}
}

void PhysicsManager::UnregisterBody(PhysicsBody* body) {
	if (IsShuttingDown()) return;
	if (!body) return;
	auto it = std::remove(_bodies.begin(), _bodies.end(), body);
	_bodies.erase(it, _bodies.end());
}
