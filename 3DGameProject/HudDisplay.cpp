#include "HudDisplay.h"
#include "DxLib.h"

namespace {
    constexpr int kLineHeight = 20;			// 1行あたりの高さ
    constexpr int kMarginRight = 10;			// 画面右端からの余白
    constexpr int kMarginBottom = 10;			// 画面下端からの余白
}

void HudDisplay::Draw(int screenWidth, int screenHeight) const
{
    if (_lines.empty()) return;

    // 下から上へ積み上げるように描画（最初に登録した行が一番上に来る）
    const int totalHeight = static_cast<int>(_lines.size()) * kLineHeight;
    int y = screenHeight - kMarginBottom - totalHeight;

    for (const auto& line : _lines) {
        std::string text;
        if (line.isInt) {
            text = line.label + " " + std::to_string(static_cast<int>(line.current)) +
                "/" + std::to_string(static_cast<int>(line.max));
        } else {
            char buf[64];
            snprintf(buf, sizeof(buf), "%s %.0f/%.0f", line.label.c_str(), line.current, line.max);
            text = buf;
        }

        const int textWidth = GetDrawStringWidth(text.c_str(), static_cast<int>(text.size()));
        const int x = screenWidth - kMarginRight - textWidth;

        DrawString(x, y, text.c_str(), GetColor(255, 255, 255));
        y += kLineHeight;
    }
}
