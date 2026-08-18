#include "GameObject.h"

#include <cstdlib>

#include "SceneManager.h"

void GameObject::PrepareForAcquire_() {
    _ownerSceneId = SceneManager::Instance().CurrentSceneId();
    SetActive(true);
    transform.SetParent(nullptr);
}

void GameObject::PrepareForRelease_() {
    SetActive(false);
    transform.SetParent(nullptr);
}

void GameObject::ApplyTransformFromParams_(const VariantMap& params,
    const VECTOR& defaultPos,
    const VECTOR& defaultScale)
{
    const float px = ParseFloatParam_(params, "px", defaultPos.x);
    const float py = ParseFloatParam_(params, "py", defaultPos.y);
    const float pz = ParseFloatParam_(params, "pz", defaultPos.z);
    transform.SetLocalPosition(VGet(px, py, pz));

    const float pitch = ParseFloatParam_(params, "pitch", ParseFloatParam_(params, "rx", 0.0f));
    const float yaw = ParseFloatParam_(params, "yaw", ParseFloatParam_(params, "ry", 0.0f));
    const float roll = ParseFloatParam_(params, "roll", ParseFloatParam_(params, "rz", 0.0f));
    transform.SetLocalEulerRad(VGet(pitch, yaw, roll));

    const float sx = ParseFloatParam_(params, "sx", defaultScale.x);
    const float sy = ParseFloatParam_(params, "sy", defaultScale.y);
    const float sz = ParseFloatParam_(params, "sz", defaultScale.z);
    transform.SetLocalScale(VGet(sx, sy, sz));
}

float GameObject::ParseFloatParam_(const VariantMap& params, const char* key, float defaultValue) {
    auto it = params.find(key);
    if (it == params.end()) return defaultValue;
    return static_cast<float>(std::atof(it->second.c_str()));
}

int GameObject::ParseIntParam_(const VariantMap& params, const char* key, int defaultValue) {
    auto it = params.find(key);
    if (it == params.end()) return defaultValue;
    return std::atoi(it->second.c_str());
}

bool GameObject::ParseBoolParam_(const VariantMap& params, const char* key, bool defaultValue) {
    auto it = params.find(key);
    if (it == params.end()) return defaultValue;
    const std::string& s = it->second;
    return s == "1" || s == "true" || s == "TRUE" || s == "True";
}

std::string GameObject::ParseStringParam_(const VariantMap& params, const char* key, const std::string& defaultValue) {
    auto it = params.find(key);
    if (it == params.end()) return defaultValue;
    return it->second;
}
