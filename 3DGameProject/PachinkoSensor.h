#pragma once

#include <functional>
#include <memory>
#include <string>

#include "GameObject.h"

class BoxCollider;
class Collider;

// パチンコ入賞判定センサー
// - isTrigger=true の BoxCollider を持つ静的オブジェクト
// - 球が触れると OnTriggerEnter が呼ばれ _hitCount が増加する
// - 外部コールバック onHit を設定すると入賞時に任意処理を実行できる
class PachinkoSensor : public GameObject {
public:
    static std::string StaticPoolKey() { return "PachinkoSensor"; }

    PachinkoSensor();
    ~PachinkoSensor() override;

    // --- ライフサイクル ---
    void Awake() override {}
    void Start() override {}
    void Update(float /*dt*/) override {}
    void Draw() override;
    void End() override {}
    void OnDestroy() override;

    // --- プール/再利用フック ---
    void OnAcquire(const VariantMap& params) override;
    void OnRelease() override;

    // --- トリガーコールバック ---
    void OnTriggerEnter(Collider* self, Collider* other) override;

    // --- 外部設定 ---
    // 入賞時に呼ばれるコールバック（other=触れた球のコライダー）
    std::function<void(Collider* other)> onHit;

    // 入賞回数取得
    int GetHitCount() const noexcept { return _hitCount; }
    void ResetHitCount() noexcept { _hitCount = 0; }

    // センサー名（UI表示用）
    const std::string& GetSensorName() const noexcept { return _sensorName; }

private:
    void ReleaseFromManagers_();

private:
    std::unique_ptr<BoxCollider> _boxCollider;
    bool _registered = false;
    int  _hitCount   = 0;
    unsigned int _drawColor = 0;
    std::string _sensorName;
};
