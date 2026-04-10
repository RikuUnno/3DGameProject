#pragma once
#pragma once
#include <cmath>

// PhysicsMaterial
// 物理素材を定義するデータ構造。
// friction / restitution / density などの物理パラメータを一括で管理し、
// プリセットから簡単に適用できるようにする。
struct PhysicsMaterial {
    // 動摩擦係数（0=無摩擦、1=非常に高い摩擦）
    float friction = 0.5f;
    // 静止摩擦係数（動摩擦以上にする。省略時は friction * 1.2）
    float staticFriction = 0.6f;
    // 反発係数（0=完全非弾性、1=完全弾性）
    float restitution = 0.0f;
    // 密度（kg/m?）。質量の自動計算に使用。0以下なら密度から質量を計算しない。
    float density = 0.0f;
    // 線形減衰（空気抵抗的な減速）
    float linearDamping = 0.0f;
    // 角速度減衰
    float angularDamping = 0.05f;

    // --- 摩擦/反発の合成モード ---
    enum class CombineMode {
        Average,   // (a + b) / 2
        Minimum,   // min(a, b)
        Maximum,   // max(a, b)
        Multiply,  // sqrt(a * b)   ← 幾何平均
    };
    CombineMode frictionCombine = CombineMode::Multiply;
    CombineMode restitutionCombine = CombineMode::Minimum;

    // --- 合成ヘルパー ---
    static float Combine(float a, float b, CombineMode mode) noexcept {
        switch (mode) {
        case CombineMode::Average:  return (a + b) * 0.5f;
        case CombineMode::Minimum:  return (a < b) ? a : b;
        case CombineMode::Maximum:  return (a > b) ? a : b;
        case CombineMode::Multiply:
        default: {
            const float fa = (a > 0.0f) ? a : 0.0f;
            const float fb = (b > 0.0f) ? b : 0.0f;
            const float val = fa * fb;
            if (val <= 0.0f) return 0.0f;
            return std::sqrt(val);
        }
        }
    }

    // --- 2素材間の合成結果を返す ---
    // 合成モードは priority の高い方（Maximize > Multiply > Average > Minimum）を採用する。
    // 簡易版として、呼び出し側で明示的に選べるようにする。
    static float CombineFriction(const PhysicsMaterial& a, const PhysicsMaterial& b) noexcept {
        // 優先度の高い方のモードを採用
        CombineMode mode = (static_cast<int>(a.frictionCombine) >= static_cast<int>(b.frictionCombine))
            ? a.frictionCombine : b.frictionCombine;
        return Combine(a.friction, b.friction, mode);
    }

    static float CombineRestitution(const PhysicsMaterial& a, const PhysicsMaterial& b) noexcept {
        CombineMode mode = (static_cast<int>(a.restitutionCombine) >= static_cast<int>(b.restitutionCombine))
            ? a.restitutionCombine : b.restitutionCombine;
        return Combine(a.restitution, b.restitution, mode);
    }

    // 静止摩擦の合成
    static float CombineStaticFriction(const PhysicsMaterial& a, const PhysicsMaterial& b) noexcept {
        CombineMode mode = (static_cast<int>(a.frictionCombine) >= static_cast<int>(b.frictionCombine))
            ? a.frictionCombine : b.frictionCombine;
        return Combine(a.staticFriction, b.staticFriction, mode);
    }

    // ============================
    //  プリセット
    // ============================

    // デフォルト素材
    static PhysicsMaterial Default() noexcept {
        return {};
    }

    // 木材: 中程度の摩擦、低い反発
    static PhysicsMaterial Wood() noexcept {
        PhysicsMaterial m;
        m.friction = 0.5f;
        m.staticFriction = 0.6f;
        m.restitution = 0.15f;
        m.density = 600.0f;    // kg/m?
        m.linearDamping = 0.02f;
        m.angularDamping = 0.05f;
        return m;
    }

    // 金属: 低い摩擦、低い反発、高密度
    static PhysicsMaterial Metal() noexcept {
        PhysicsMaterial m;
        m.friction = 0.3f;
        m.staticFriction = 0.4f;
        m.restitution = 0.15f;
        m.density = 7800.0f;
        m.linearDamping = 0.0f;
        m.angularDamping = 0.02f;
        return m;
    }

    // ゴム: 高い摩擦、高い反発
    static PhysicsMaterial Rubber() noexcept {
        PhysicsMaterial m;
        m.friction = 0.9f;
        m.staticFriction = 1.0f;
        m.restitution = 0.8f;
        m.density = 1100.0f;
        m.linearDamping = 0.05f;
        m.angularDamping = 0.1f;
        m.frictionCombine = CombineMode::Maximum;
        m.restitutionCombine = CombineMode::Maximum;
        return m;
    }

    // 氷: 非常に低い摩擦、低い反発
    static PhysicsMaterial Ice() noexcept {
        PhysicsMaterial m;
        m.friction = 0.02f;
        m.staticFriction = 0.03f;
        m.restitution = 0.05f;
        m.density = 920.0f;
        m.linearDamping = 0.0f;
        m.angularDamping = 0.01f;
        m.frictionCombine = CombineMode::Minimum;
        return m;
    }

    // 石/コンクリート: 高い摩擦、低い反発、高密度
    static PhysicsMaterial Stone() noexcept {
        PhysicsMaterial m;
        m.friction = 0.7f;
        m.staticFriction = 0.8f;
        m.restitution = 0.05f;
        m.density = 2400.0f;
        m.linearDamping = 0.0f;
        m.angularDamping = 0.03f;
        return m;
    }

    // 弾むボール: 中摩擦、非常に高い反発
    static PhysicsMaterial Bouncy() noexcept {
        PhysicsMaterial m;
        m.friction = 0.4f;
        m.staticFriction = 0.5f;
        m.restitution = 0.95f;
        m.density = 500.0f;
        m.linearDamping = 0.01f;
        m.angularDamping = 0.02f;
        m.restitutionCombine = CombineMode::Maximum;
        return m;
    }

    // 名前からプリセットを取得（不明な名前は Default を返す）
    static PhysicsMaterial FromName(const char* name) noexcept {
        if (!name) return Default();
        // 簡易文字列比較（大文字小文字は無視しない）
        auto eq = [](const char* a, const char* b) -> bool {
            while (*a && *b) {
                char ca = *a, cb = *b;
                if (ca >= 'A' && ca <= 'Z') ca += 32;
                if (cb >= 'A' && cb <= 'Z') cb += 32;
                if (ca != cb) return false;
                ++a; ++b;
            }
            return *a == *b;
        };
        if (eq(name, "wood"))    return Wood();
        if (eq(name, "metal"))   return Metal();
        if (eq(name, "rubber"))  return Rubber();
        if (eq(name, "ice"))     return Ice();
        if (eq(name, "stone"))   return Stone();
        if (eq(name, "bouncy"))  return Bouncy();
        if (eq(name, "default")) return Default();
        return Default();
    }
};
