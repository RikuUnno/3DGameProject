#pragma once

#include "PachinkoFieldTpl.h"

// PachinkoField_Front: 前面のパチンコ盤面（透明な板）オブジェクト
// 摩擦をなくして、玉が滑るようにするために frictionless マテリアルを使用
class PachinkoField_Front : public PachinkoFieldTpl {
public:
    static std::string StaticPoolKey() { return "PachinkoField_Front"; }

protected:
    const char* FieldName_() const noexcept override { return "PachinkoField_Front"; }
    VECTOR DefaultHalfExtents_() const noexcept override { return VGet(15.0f, 4.0f, 0.5f); }
    unsigned int DefaultColor_() const noexcept override { return GetColor(180, 180, 200); }
    std::string DefaultMaterialName_() const override { return "frictionless"; }
};

// PachinkoField_Back: 背面のパチンコ盤面（色付き板）オブジェクト 
// 摩擦をなくして、玉が滑るようにするために frictionless マテリアルを使用
class PachinkoField_Back : public PachinkoFieldTpl {
public:
    static std::string StaticPoolKey() { return "PachinkoField_Back"; }

protected:
    const char* FieldName_() const noexcept override { return "PachinkoField_Back"; }
    VECTOR DefaultHalfExtents_() const noexcept override { return VGet(15.0f, 4.0f, 0.5f); }
    unsigned int DefaultColor_() const noexcept override { return GetColor(150, 150, 170); }
    std::string DefaultMaterialName_() const override { return "frictionless"; }
};

// PachinkoField_Side: 側面のパチンコ盤面（透明な板）オブジェクト
// 摩擦をなくして、玉が滑るようにするために frictionless マテリアルを使用
class PachinkoField_Side : public PachinkoFieldTpl {
public:
    static std::string StaticPoolKey() { return "PachinkoField_Side"; }

protected:
    const char* FieldName_() const noexcept override { return "PachinkoField_Side"; }
    VECTOR DefaultHalfExtents_() const noexcept override { return VGet(0.5f, 4.0f, 9.0f); }
    unsigned int DefaultColor_() const noexcept override { return GetColor(200, 200, 220); }
    std::string DefaultMaterialName_() const override { return "frictionless"; }
};
