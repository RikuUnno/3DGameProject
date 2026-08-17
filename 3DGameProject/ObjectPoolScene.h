#pragma once
#include <deque>
#include <string>
#include "CameraController.h"
#include "CameraTags.h"
#include "ObjectPoolDebugClass.h"
#include "SceneTpl.h"
class ObjectPoolScene : public SceneTpl<ObjectPoolScene> {
public:
    static std::string StaticName() { return "ObjectPoolScene"; }
    void Start() override;
    void End() override;
    void Update(float dtSec) override;
    void Draw() override;
private:
    void SpawnBall_();
    void ReleaseExpiredBalls_();
    static void DrawGridFloor_(float y, int halfCells, float step);
private:
    CameraController _cameraController;
    CameraController::CameraId _cameraId = 0;
    std::deque<ObjectPoolDebugClass*> _liveBalls;
    float _spawnAccumSec = 0.0f;
    int   _spawnSerial   = 0;
    static constexpr float _autoSpawnIntervalSec = 0.35f;
};