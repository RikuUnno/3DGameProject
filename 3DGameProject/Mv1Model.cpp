#include "Mv1Model.h"

Mv1Model::~Mv1Model() {
    Reset();
}

bool Mv1Model::Load(const std::string& filePath) {
    Reset();
    _path = filePath;
    _handle = MV1LoadModel(filePath.c_str());
    return _handle != -1;
}

void Mv1Model::Reset() {
    if (_handle != -1) {
        MV1DeleteModel(_handle);
        _handle = -1;
    }
    _path.clear();
}

void Mv1Model::Draw() {
    if (!_visible || _handle == -1) return;
    MV1DrawModel(_handle);
}

std::unique_ptr<IModel> Mv1Model::Duplicate() const {
    if (_handle == -1) return nullptr;
    auto dup = std::make_unique<Mv1Model>();
    dup->_handle = MV1DuplicateModel(_handle);
    dup->_path = _path;
    if (dup->_handle == -1) return nullptr;
    return dup;
}

void Mv1Model::SetPosition(const VECTOR& pos) {
    if (_handle == -1) return;
    MV1SetPosition(_handle, pos);
}

void Mv1Model::SetRotation(const Quaternion& rot) {
    if (_handle == -1) return;
    // Quaternion -> Euler ‚Å MV1 ‚É“n‚·
    const VECTOR e = rot.ToEulerRad();
    MV1SetRotationXYZ(_handle, e);
}

void Mv1Model::SetScale(const VECTOR& scale) {
    if (_handle == -1) return;
    MV1SetScale(_handle, scale);
}

void Mv1Model::SetMatrix(const MATRIX& m) {
    if (_handle == -1) return;
    MV1SetMatrix(_handle, m);
}
