#include "PachinkoSensor.h"

#include "BoxCollider.h"
#include "ColliderManager.h"
#include "DxLib.h"
#include "LayerMask.h"

PachinkoSensor::PachinkoSensor() {
    _boxCollider = std::make_unique<BoxCollider>();
    _boxCollider->owner = this;
}

PachinkoSensor::~PachinkoSensor() = default;

void PachinkoSensor::OnDestroy() {
    ReleaseFromManagers_();
}

void PachinkoSensor::OnAcquire(const VariantMap& params) {
    PrepareForAcquire_();

    _hitCount = 0;
    onHit = nullptr;
    _sensorName = ParseStringParam_(params, "name", "Sensor");

    // 位置・スケール適用
    ApplyTransformFromParams_(params);

    // ハーフエクステントの設定（デフォルト: x=1.0, y=0.2, z=1.0）
    const float hx = ParseFloatParam_(params, "hx", 1.0f);
    const float hy = ParseFloatParam_(params, "hy", 0.2f);
    const float hz = ParseFloatParam_(params, "hz", 1.0f);

    // 描画色（デフォルト: 黒）
    const unsigned int defaultColor = GetColor(0, 0, 0);
    _boxCollider->SetDebugColor(defaultColor);

    // BoxCollider 設定
    _boxCollider->owner = this;
    _boxCollider->isTrigger = true;             // 常にトリガー
	_boxCollider->layer = layerMask::SENSOR;         // センサーレイヤー
    _boxCollider->mask  = mask::BALL;
    _boxCollider->enableCCD = false;
    _boxCollider->sendEventsToOwner = true;
    _boxCollider->SetDebugColor(_drawColor);

    // ローカル座標での Box サイズ指定
    _boxCollider->_box.halfExtents = VGet(hx, hy, hz);
    _boxCollider->_box.center      = VGet(0.0f, 0.0f, 0.0f);
    _boxCollider->UpdateShape();

    ColliderManager::Instance().RegisterCollider(_boxCollider.get());
    _registered = true;
}

void PachinkoSensor::OnRelease() {
    ReleaseFromManagers_();
    PrepareForRelease_();
}

void PachinkoSensor::Draw() {
    if (_boxCollider) _boxCollider->DrawDebug();
}

void PachinkoSensor::OnTriggerEnter(Collider* /*self*/, Collider* other) {
    ++_hitCount;
    if (onHit) onHit(other);
}

void PachinkoSensor::ReleaseFromManagers_() {
    if (!_registered) return;
    if (_boxCollider) ColliderManager::Instance().UnregisterCollider(_boxCollider.get());
    _registered = false;
}
