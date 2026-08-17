#pragma once
#include <array>
#include <deque>
#include <string>
#include "CameraController.h"
#include "CameraTags.h"
#include "PhysicsDebugClass.h"
#include "SceneTpl.h"
class CcdScene : public SceneTpl<CcdScene> {
public:
    static std::string StaticName() { return "CcdScene"; }
    void Start() override;
    void End() override;
    void Update(float dtSec) override;
    void Draw() override;
private:
    struct Lane { float z=0; const char* label=""; bool ccd=true; float threshold=8.0f; unsigned int color=0; };
    void RegisterPools_();
    void SpawnSetup_();
    void SpawnStaticBox_(float px,float py,float pz,float hx,float hy,float hz,unsigned int color);
    void FireAllLanes_();
    void FireLane_(int index);
    void ReleaseOldestIfNeeded_(std::deque<PhysicsDebugClass*>& list);
    void ReleaseOldProjectiles_();
    void ClearProjectiles_();
private:
    CameraController _cameraController;
    CameraController::CameraId _cameraId = 0;
    std::array<std::deque<PhysicsDebugClass*>, 4> _trackers{};
    float _spawnAccumSec = 0.0f;
    bool  _autoFire = true;
    static constexpr float  _autoFireIntervalSec = 1.2f;
    static constexpr size_t _maxPerLane = 8;
    const std::array<Lane, 4> _lanes{{ {-6.0f,"CCD OFF",false,9999.0f,0}, {-2.0f,"CCD ON",true,8.0f,0}, {2.0f,"small",false,3.0f,0}, {6.0f,"large",false,150.0f,0} }};
};