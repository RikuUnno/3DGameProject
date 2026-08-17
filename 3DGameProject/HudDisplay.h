#pragma once
#include <string>
#include <vector>

// 画面右下に表示するステータスHUD
// ・体力や弾数など「現在値/最大値」形式のリソースを行として登録できる
// ・今後ミサイル等の武器を追加する場合は AddResource(...) で行を追加するだけでよい
class HudDisplay
{
public:
    // HUDに表示する1行分のリソース情報
    struct ResourceLine
    {
        std::string label;	// 表示名（例: "体力", "球数"）
        float current = 0.0f;	// 現在値
        float max     = 0.0f;	// 最大値
        bool  isInt   = true;	// 整数表示にするか（弾数等）小数表示にするか（体力等はfalseでも可）
    };

    // 表示行をすべてクリアする（毎フレームUpdateの先頭で呼ぶ想定）
    void Clear() { _lines.clear(); }

    // 表示行を追加する（体力・弾数・今後追加するミサイル等すべて共通）
    void AddResource(const std::string& label, float current, float max, bool isInt = true)
    {
        _lines.push_back({ label, current, max, isInt });
    }

    // 画面右下に登録済みの行を描画する
    // screenWidth / screenHeight : 画面サイズ（Info.h の WINDOW_WIDTH / WINDOW_HEIGHT を渡す）
    void Draw(int screenWidth, int screenHeight) const;

private:
    std::vector<ResourceLine> _lines;
};
