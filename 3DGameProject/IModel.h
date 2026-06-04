#pragma once

#include "DxLib.h"
#include "Math/Quaternion.h"
#include <string>
#include <memory>

// IModel
// - 描画用モデルの抽象基底
// - 拡張性: 既定実装は Mv1Model (DxLib MV1, .mv1/.fbx)
//   別形式 (自前メッシュ・スプライト等) を足したい場合はこの IF を継承する
// - 重い処理 (実ファイルのデコード) は Load() のみ。
//   ゲーム中の Acquire/Release はテンプレートからの「複製」だけで済むよう
//   各派生で軽量化すること (ModelPool が要求する想定)
class IModel {
public:
    virtual ~IModel() = default;

    // 実ファイル読み込み (重い)
    virtual bool Load(const std::string& filePath) = 0;
    // 状態クリア (ハンドル解放など)
    virtual void Reset() = 0;
    // 描画
    virtual void Draw() = 0;

    // テンプレートから軽量複製を作る (ModelPool 用)
    // nullptr を返した場合は ModelManager 側で改めて Load() される
    virtual std::unique_ptr<IModel> Duplicate() const { return nullptr; }

    // Transform 反映 (毎フレーム呼ばれることを想定して軽い実装で)
    virtual void SetPosition(const VECTOR& pos) = 0;
    virtual void SetRotation(const Quaternion& rot) = 0;
    virtual void SetScale(const VECTOR& scale) = 0;
    // MATRIX 直接設定 (Transform::WorldMatrix() をそのまま渡せる経路)
    virtual void SetMatrix(const MATRIX& m) = 0;

    // 可視フラグ
    void SetVisible(bool v) noexcept { _visible = v; }
    bool IsVisible() const noexcept { return _visible; }

    const std::string& Path() const noexcept { return _path; }

protected:
    std::string _path;
    bool _visible = true;
};
