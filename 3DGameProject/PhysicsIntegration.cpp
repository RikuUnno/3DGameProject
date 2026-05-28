#include "PhysicsManager_Internal.h"

void PhysicsManager::GatherBodySoA() {
    const size_t n = _bodies.size();
    _bodySoA.Resize(n);
    ThreadPool::Instance().ParallelForBarrier(0, n, [&](size_t i) {
        PhysicsBody* b = _bodies[i];
        if (!b || !b->_owner) { _bodySoA.flags[i] = 0; return; }
        uint8_t f = 0;
        if (b->_enabled && b->_owner->IsActive())                                   f |= 1;
        if (b->_isKinematic)                                                         f |= 2;
        if (b->_isSleeping)                                                          f |= 4;
        if (b->_useGravity)                                                          f |= 8;
        if (b->_freezeRotation)                                                      f |= 16;
        if (b->_detectContinuous && b->_ccdQuality >= CcdQuality::Default)           f |= 32;
        _bodySoA.flags[i]           = f;
        _bodySoA.position[i]        = b->_owner->transform.LocalPosition();
        _bodySoA.velocity[i]        = b->_velocity;
        _bodySoA.angularVelocity[i] = b->_angularVelocity;
        _bodySoA.force[i]           = b->_force;
        _bodySoA.torque[i]          = b->_torque;
        _bodySoA.inverseMass[i]     = b->InverseMass();
        _bodySoA.linearDamping[i]   = b->_linearDamping;
        _bodySoA.angularDamping[i]  = b->_angularDamping;
        _bodySoA.gravityScale[i]    = b->_gravityScale;
    }, 64);
}

void PhysicsManager::ScatterBodySoA(float /*stepDt*/) {
    const size_t n = _bodies.size();
    ThreadPool::Instance().ParallelForBarrier(0, n, [&](size_t i) {
        if (!(_bodySoA.flags[i] & 1)) return;
        PhysicsBody* b = _bodies[i];
        if (!b || !b->_owner) return;
        b->_velocity        = _bodySoA.velocity[i];
        b->_angularVelocity = _bodySoA.angularVelocity[i];
    }, 64);
}

void PhysicsManager::IntegrateBodies(float stepDt) {
    const size_t bodyCount = _bodies.size();
    if (bodyCount == 0) return;

    const bool    groundEnabled = _groundPlaneEnabled;
    const VECTOR  groundN       = _groundPlaneNormal;
    const float   groundD       = _groundPlaneD;
    const VECTOR  gravity       = _gravity;

    // フェーズ1: Gather SoA + 速度積分（統合、1バリア）
    _bodySoA.Resize(bodyCount);
    ThreadPool::Instance().ParallelForBarrier(0, bodyCount, [&](size_t idx) {
        if (idx + 2 < bodyCount && _bodies[idx + 2]) {
#if defined(_MSC_VER)
            _mm_prefetch(reinterpret_cast<const char*>(_bodies[idx + 2]), _MM_HINT_T0);
#endif
        }
        PhysicsBody* b = _bodies[idx];
        if (!b || !b->_owner) { _bodySoA.flags[idx] = 0; return; }

        uint8_t f = 0;
        if (b->_enabled && b->_owner->IsActive())                         f |= 1;
        if (b->_isKinematic)                                               f |= 2;
        if (b->_isSleeping)                                                f |= 4;
        if (b->_useGravity)                                                f |= 8;
        if (b->_freezeRotation)                                            f |= 16;
        if (b->_detectContinuous && b->_ccdQuality >= CcdQuality::Default) f |= 32;
        _bodySoA.flags[idx]           = f;
        _bodySoA.position[idx]        = b->_owner->transform.LocalPosition();
        _bodySoA.velocity[idx]        = b->_velocity;
        _bodySoA.angularVelocity[idx] = b->_angularVelocity;
        _bodySoA.force[idx]           = b->_force;
        _bodySoA.torque[idx]          = b->_torque;
        _bodySoA.inverseMass[idx]     = b->InverseMass();
        _bodySoA.linearDamping[idx]   = b->_linearDamping;
        _bodySoA.angularDamping[idx]  = b->_angularDamping;
        _bodySoA.gravityScale[idx]    = b->_gravityScale;

        if (!(f & 1)) return;
        if (f & 2)    return;
        if ((f & 4) && LenSq(_bodySoA.force[idx]) <= 1e-8f
                    && LenSq(_bodySoA.torque[idx]) <= 1e-8f) return;

        const float invM = _bodySoA.inverseMass[idx];
        if (invM <= 0.0f) return;

        VECTOR acc = VScale(_bodySoA.force[idx], invM);
        if (f & 8) acc = VAdd(acc, VScale(gravity, _bodySoA.gravityScale[idx]));
        _bodySoA.velocity[idx] = VAdd(_bodySoA.velocity[idx], VScale(acc, stepDt));

        if (!(f & 16)) {
            const VECTOR angAcc = b->ApplyInverseInertia(_bodySoA.torque[idx]);
            _bodySoA.angularVelocity[idx] = VAdd(_bodySoA.angularVelocity[idx], VScale(angAcc, stepDt));
        }
        if (_bodySoA.linearDamping[idx] > 0.0f)
            _bodySoA.velocity[idx] = VScale(_bodySoA.velocity[idx],
                1.0f / (1.0f + _bodySoA.linearDamping[idx] * stepDt));
        if (!(f & 16) && _bodySoA.angularDamping[idx] > 0.0f)
            _bodySoA.angularVelocity[idx] = VScale(_bodySoA.angularVelocity[idx],
                1.0f / (1.0f + _bodySoA.angularDamping[idx] * stepDt));
    }, 64);

    // フェーズ2: Scatter + 位置/回転更新（統合、1バリア）
    ThreadPool::Instance().ParallelForBarrier(0, bodyCount, [&](size_t idx) {
        if (_bodySoA.flags[idx] & 1) {
            PhysicsBody* bs = _bodies[idx];
            if (bs && bs->_owner) {
                bs->_velocity        = _bodySoA.velocity[idx];
                bs->_angularVelocity = _bodySoA.angularVelocity[idx];
            }
        }

        PhysicsBody* body = _bodies[idx];
        if (!body || !body->_enabled || !body->_owner || !body->_owner->IsActive()) return;

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
            body->_velocity = VGet(0,0,0);
            body->_angularVelocity = VGet(0,0,0);
            return;
        }

        if (body->_isSleeping) {
            const float lSq = body->_sleepLinearThreshold * body->_sleepLinearThreshold;
            const float aSq = body->_sleepAngularThreshold * body->_sleepAngularThreshold;
            const bool moving = LenSq(body->_velocity) > lSq || LenSq(body->_angularVelocity) > aSq;
            if (moving) {
                body->WakeUp();
            } else if (LenSq(body->_force) <= 1e-8f && LenSq(body->_torque) <= 1e-8f) {
                return;
            }
        }
        if (LenSq(body->_force) > 1e-8f || LenSq(body->_torque) > 1e-8f) body->WakeUp();

        const float inverseMass = body->InverseMass();
        if (inverseMass <= 0.0f) { body->ClearAccumulators(); return; }

        ApplyBodyConstraints(body);
        ClampMagnitude(body->_velocity, body->_maxLinearSpeed);
        if (!body->_freezeRotation) ClampMagnitude(body->_angularVelocity, body->_maxAngularSpeed);

        // CCD 速度クランプは無効化（Speculative Contactsのみ使用）
        // Speculative Contactsが予測接触を生成するため、速度制限は不要

        VECTOR pos = body->_owner->transform.LocalPosition();
        pos = VAdd(pos, VScale(body->_velocity, stepDt));

        if (groundEnabled) {
            const float dist = HalfPlaneDistance(pos, groundN, groundD);
            if (dist < 0.0f) {
                pos = VSub(pos, VScale(groundN, dist));
                const float vn = Dot3(body->_velocity, groundN);
                if (vn < 0.0f) body->_velocity = VSub(body->_velocity, VScale(groundN, vn));
                if (!body->_freezeRotation && body->_friction > 0.0f)
                    body->_angularVelocity = VScale(body->_angularVelocity,
                        1.0f / (1.0f + body->_friction * 0.5f * stepDt));
            }
        }

        // 場外落下または速度爆発を検出してリセット
        const float speedSq = LenSq(body->_velocity);
        const float maxSpd  = body->_maxLinearSpeed;
        if (pos.y < -200.0f || speedSq > maxSpd * maxSpd) {
            pos = body->_previousPosition;
            body->_velocity        = VGet(0, 0, 0);
            body->_angularVelocity = VGet(0, 0, 0);
            body->Sleep();
        }
        body->_owner->transform.SetLocalPosition(pos);

        if (!body->_freezeRotation) {
            const float angSpd = Len3(body->_angularVelocity);
            if (angSpd > 1e-6f) {
                const VECTOR& w = body->_angularVelocity;
                Quaternion q = body->_owner->transform.LocalRotation();
                Quaternion wq(w.x * 0.5f * stepDt, w.y * 0.5f * stepDt, w.z * 0.5f * stepDt, 0.0f);
                Quaternion dq = Quaternion::Multiply(wq, q);
                q.x += dq.x; q.y += dq.y; q.z += dq.z; q.w += dq.w;
                body->_owner->transform.SetLocalRotation(q.Normalized());
            }
        }

        body->_hasMovePositionTarget = false;
        body->_hasMoveRotationTarget = false;
        body->ClearAccumulators();
        SanitizeVec(body->_velocity);
        SanitizeVec(body->_angularVelocity);
        {
            VECTOR p = body->_owner->transform.LocalPosition();
            if (!IsFiniteVec(p)) {
                body->_owner->transform.SetLocalPosition(body->_previousPosition);
                body->_velocity        = VGet(0,0,0);
                body->_angularVelocity = VGet(0,0,0);
            }
        }
    }, 64);
}
