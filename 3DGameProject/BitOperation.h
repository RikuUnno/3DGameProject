#ifndef BIT_OPERATION_H
#define BIT_OPERATION_H

//  BitOperation.h - ビット演算用の軽量ヘルパー
namespace BitOperation {

    // --- 判定系 -------------------------------------------------

    // value の中に mask のビットが1つでも含まれていれば true
    // 等価: (value & mask) != 0
    template <class T>
    constexpr bool HasAny(T value, T mask) noexcept {
        return (value & mask) != T{};
    }

    // value が mask の全ビットを含んでいれば true
    // 等価: (value & mask) == mask
    template <class T>
    constexpr bool HasAll(T value, T mask) noexcept {
        return (value & mask) == mask;
    }

    // value が mask のビットを1つも含まなければ true
    // 等価: (value & mask) == 0
    template <class T>
    constexpr bool HasNone(T value, T mask) noexcept {
        return (value & mask) == T{};
    }

    // a と b に共通ビットが1つでもあれば true（対称判定）
    // 等価: (a & b) != 0
    template <class T>
    constexpr bool Overlaps(T a, T b) noexcept {
        return (a & b) != T{};
    }

    // 指定ビット（またはビット集合）が立っていれば true
    template <class T>
    constexpr bool Test(T value, T bit) noexcept {
        return (value & bit) != T{};
    }

    // レイヤー/マスクの双方向一致判定（衝突フィルタ向け）
    // 両者が互いを許可しているときのみ true
    // 等価: (layerA & maskB) != 0 && (layerB & maskA) != 0
    template <class T>
    constexpr bool MutualMatch(T layerA, T maskA, T layerB, T maskB) noexcept {
        return Overlaps(layerA, maskB) && Overlaps(layerB, maskA);
    }

    // --- ビット操作系 ------------------------------------------------

    // bits を立てた値を返す
    template <class T>
    constexpr T Set(T value, T bits) noexcept {
        return static_cast<T>(value | bits);
    }

    // bits を下げた値を返す
    template <class T>
    constexpr T Clear(T value, T bits) noexcept {
        return static_cast<T>(value & ~bits);
    }

    // bits を反転した値を返す
    template <class T>
    constexpr T Toggle(T value, T bits) noexcept {
        return static_cast<T>(value ^ bits);
    }

    // on=true なら bits を立てる / false なら bits を下げる
    // 分岐なし（ブランチレス）で処理
    template <class T>
    constexpr T SetTo(T value, T bits, bool on) noexcept {
        // on=true で全ビット1, false で全ビット0のマスクを作る
        const T m = static_cast<T>(-static_cast<T>(on));
        return static_cast<T>((value & ~bits) | (bits & m));
    }

    // --- ブランチレス補助 ------------------------------------------------
    // if や ?: を使わず、ビット演算のみで条件分岐相当を行う

    // cond=true なら全ビット1、false なら全ビット0
    template <class T>
    constexpr T MaskFromBool(bool cond) noexcept {
        return static_cast<T>(-static_cast<T>(cond));
    }

    // cond に応じて whenTrue / whenFalse を選ぶ（ブランチレス）
    //   cond ? whenTrue : whenFalse
    template <class T>
    constexpr T SelectIf(bool cond, T whenTrue, T whenFalse) noexcept {
        const T m = MaskFromBool<T>(cond);
        return static_cast<T>((whenTrue & m) | (whenFalse & ~m));
    }

    // (value & mask) 判定で whenSet / whenClear を選ぶ（ブランチレス）
    template <class T>
    constexpr T Select(T value, T mask, T whenSet, T whenClear) noexcept {
        return SelectIf(HasAny(value, mask), whenSet, whenClear);
    }

    // cond=true のときだけ addBits を OR する（ブランチレス）
    template <class T>
    constexpr T OrIf(T value, T addBits, bool cond) noexcept {
        return static_cast<T>(value | (addBits & MaskFromBool<T>(cond)));
    }

}

// --- 便利マクロ --------------------------------------------------
// BIT(n): n番目の1ビットマスクを作る（1 << n）
#ifndef BIT
#define BIT(n) (1 << (n))
#endif

#endif // BIT_OPERATION_H
