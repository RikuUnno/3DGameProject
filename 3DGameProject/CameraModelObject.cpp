#include "CameraModelObject.h"
#include <cstdlib>
#include <array>
#include "DxLib.h"

CameraModelObject::~CameraModelObject()
{
    if (_modelHandle >= 0) {
        MV1DeleteModel(_modelHandle);
        _modelHandle = -1;
    }
}

void CameraModelObject::OnAcquire(const VariantMap& params)
{
    SetActive(true);
    isStatic = true;
    transform.SetParent(nullptr);
    ConfigureFromParams_(params);
    LoadModelIfNeeded_();
}

void CameraModelObject::OnRelease()
{
    transform.SetParent(nullptr);
    SetActive(false);
}

void CameraModelObject::Draw()
{
    if (_modelHandle < 0) return;
    MV1SetPosition(_modelHandle, transform.WorldPosition());
    MV1SetRotationXYZ(_modelHandle, transform.LocalEulerRad());
    MV1SetScale(_modelHandle, transform.LocalScale());
    MV1DrawModel(_modelHandle);
}

void CameraModelObject::ConfigureFromParams_(const VariantMap& params)
{
    auto f = [&](const char* key, float def) {
        auto it = params.find(key);
        return (it == params.end()) ? def : static_cast<float>(std::atof(it->second.c_str()));
    };
    auto s = [&](const char* key, const char* def) {
        auto it = params.find(key);
        return (it == params.end()) ? std::string(def) : it->second;
    };
    _modelPath = s("modelPath", "");
    transform.SetLocalPosition(VGet(f("px", 0.0f), f("py", 0.0f), f("pz", 0.0f)));
    transform.SetLocalEulerRad(VGet(f("pitch", 0.0f), f("yaw", 0.0f), f("roll", 0.0f)));
    const float sc = f("scale", 1.0f);
    transform.SetLocalScale(VGet(f("sx", sc), f("sy", sc), f("sz", sc)));
}

void CameraModelObject::LoadModelIfNeeded_()
{
    if (_modelPath.empty()) return;
    if (_loadedPath == _modelPath && _modelHandle >= 0) return;
    if (_modelHandle >= 0) {
        MV1DeleteModel(_modelHandle);
        _modelHandle = -1;
    }
    const std::array<std::string, 7> candidates = {
        _modelPath,
        std::string("./") + _modelPath,
        std::string("../") + _modelPath,
        std::string("3DGameProject/") + _modelPath,
        std::string("../3DGameProject/") + _modelPath,
        std::string("../../3DGameProject/") + _modelPath,
        std::string("C:/Users/rinsa/source/repos/3DGameProject/3DGameProject/") + _modelPath,
    };
    for (const auto& p : candidates) {
        _lastTriedPath = p;
        _modelHandle = MV1LoadModel(p.c_str());
        if (_modelHandle >= 0) {
            _loadedPath = _modelPath;
            return;
        }
    }
}
