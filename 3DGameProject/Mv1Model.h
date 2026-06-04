#pragma once

#include "IModel.h"

// Mv1Model
// - DxLib の MV1 系 API を使った汎用モデル
// - .mv1 / .x / .fbx (Unity から FBX で書き出したもの含む) を MV1LoadModel で読み込み
// - Duplicate() は MV1DuplicateModel を使って軽量に複製する
//   (テクスチャ等のリソースは共有されるため Acquire 時のコストが低い)
class Mv1Model : public IModel {
public:
    Mv1Model() = default;
    ~Mv1Model() override;

    Mv1Model(const Mv1Model&) = delete;
    Mv1Model& operator=(const Mv1Model&) = delete;

    bool Load(const std::string& filePath) override;
    void Reset() override;
    void Draw() override;

    std::unique_ptr<IModel> Duplicate() const override;

    void SetPosition(const VECTOR& pos) override;
    void SetRotation(const Quaternion& rot) override;
    void SetScale(const VECTOR& scale) override;
    void SetMatrix(const MATRIX& m) override;

    int Handle() const noexcept { return _handle; }

private:
	int _handle = -1;   // MV1モデルハンドル -1 の場合は無効
};
