#pragma once

#include <string>
#include <functional>

class Assert {
public:
	// ハンドラが true を返すと標準のブレーク処理を行います（true = ブレーク）
	using Handler = std::function<bool(const char* expr, const char* file, int line, const char* func, const std::string& message)>;

	static void SetHandler(Handler handler);
	static void ClearHandler();

	// 基本的な失敗通知（メッセージ付き/無し）
	static void Fail(const char* expr, const char* file, int line, const char* func, const std::string& message) noexcept;
	static void FailFmt(const char* expr, const char* file, int line, const char* func, const char* fmt, ...) noexcept;

	// 条件チェック（false のとき Fail を呼ぶ）
	static void Check(bool condition, const char* expr, const char* file, int line, const char* func, const char* message = nullptr) noexcept;

private:
	static Handler s_handler;
};

// デバッグ時は ASSERT / ASSERT_MSG が動作して Fail を呼ぶ
#ifndef NDEBUG
#define ASSERT(expr) ((expr) ? (void)0 : Assert::Fail(#expr, __FILE__, __LINE__, __func__, std::string()))
#define ASSERT_MSG(expr, fmt, ...) ((expr) ? (void)0 : Assert::FailFmt(#expr, __FILE__, __LINE__, __func__, (fmt), __VA_ARGS__))
#else
#define ASSERT(expr) ((void)0)
#define ASSERT_MSG(expr, fmt, ...) ((void)0)
#endif

// 常時使えるアサート: リリース/デバッグ問わず Fail を呼ぶ（必要に応じて動作を変更）
#define VERIFY(expr) ((expr) ? (void)0 : Assert::Fail(#expr, __FILE__, __LINE__, __func__, std::string()))
#define VERIFY_MSG(expr, fmt, ...) ((expr) ? (void)0 : Assert::FailFmt(#expr, __FILE__, __LINE__, __func__, (fmt), __VA_ARGS__))


//互換エイリアス
#define ASSERT_ALWAYS VERIFY
#define ASSERT_ALWAYS_MSG VERIFY_MSG
