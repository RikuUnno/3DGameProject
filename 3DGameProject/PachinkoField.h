#pragma once

#include "PachinkoFieldTpl.h"

// PachinkoField_Front: 前面のパチンコ盤面（透明な板）オブジェクト
// 摩擦をなくして、玉が滑るようにするために frictionless マテリアルを使用
class PachinkoField_Front : public PachinkoFieldTpl {
public:
    static std::string StaticPoolKey() { return "PachinkoField_Front"; }

protected:
    const char* FieldName_() const noexcept override { return "PachinkoField_Front"; }
    VECTOR DefaultHalfExtents_() const noexcept override { return VGet(4.2f, 7.0f, 0.08f); }
    unsigned int DefaultColor_() const noexcept override { return GetColor(120, 200, 255); }
    std::string DefaultMaterialName_() const override { return "frictionless"; }
    DrawStyle FieldDrawStyle_() const noexcept override { return DrawStyle::AABB; }
};

// PachinkoField_Back: 背面のパチンコ盤面（色付き板）オブジェクト 
// 摩擦をなくして、玉が滑るようにするために frictionless マテリアルを使用
class PachinkoField_Back : public PachinkoFieldTpl {
public:
    static std::string StaticPoolKey() { return "PachinkoField_Back"; }

protected:
    const char* FieldName_() const noexcept override { return "PachinkoField_Back"; }
    VECTOR DefaultHalfExtents_() const noexcept override { return VGet(4.2f, 7.0f, 0.08f); }
    unsigned int DefaultColor_() const noexcept override { return GetColor(25, 25, 25); }
    std::string DefaultMaterialName_() const override { return "frictionless"; }
    DrawStyle FieldDrawStyle_() const noexcept override { return DrawStyle::Solid; }
};

// PachinkoField_Side: 側面のパチンコ盤面（透明な板）オブジェクト
// 摩擦をなくして、玉が滑るようにするために frictionless マテリアルを使用
class PachinkoField_Side : public PachinkoFieldTpl {
public:
    static std::string StaticPoolKey() { return "PachinkoField_Side"; }

protected:
    const char* FieldName_() const noexcept override { return "PachinkoField_Side"; }
    VECTOR DefaultHalfExtents_() const noexcept override { return VGet(0.08f, 7.0f, 1.9f); }
    unsigned int DefaultColor_() const noexcept override { return GetColor(120, 200, 255); }
    std::string DefaultMaterialName_() const override { return "frictionless"; }
    DrawStyle FieldDrawStyle_() const noexcept override { return DrawStyle::AABB; }
};
