#ifndef ASSERT_H
#define ASSERT_H

#include <string>
#include <functional>

// Assert - カスタムアサートクラス
class Assert {
public:
	// カスタムアサートハンドラーの型定義
    using Handler = std::function<bool(const char* expr, const char* file, int line, const char* func, const std::string& message)>;

	// デフォルトのアサートハンドラー（例: メッセージボックス表示）
    static void SetHandler(Handler handler);

	// ハンドラーをリセットしてデフォルトに戻す
    static void ClearHandler();

	// アサート失敗時の呼び出し（ハンドラーが true を返すと例外を投げる）
    static void Fail(const char* expr, const char* file, int line, const char* func, const std::string& message) noexcept;

	// フォーマット付きアサート失敗呼び出し（可変引数を受け取る）
    static void FailFmt(const char* expr, const char* file, int line, const char* func, const char* fmt, ...) noexcept;

	// 条件式を評価して失敗した場合にアサートを呼び出す（マクロから呼ばれる）
    static void Check(bool condition, const char* expr, const char* file, int line, const char* func, const char* message = nullptr) noexcept;

private:
	// 現在のアサートハンドラー（nullptr の場合はデフォルト動作）
    static Handler s_handler;
};

// アサートマクロ
#ifndef NDEBUG	// デバッグビルドで有効化
#define ASSERT(expr) ((expr) ? (void)0 : Assert::Fail(#expr, __FILE__, __LINE__, __func__, std::string()))	// 条件式を文字列化してアサート呼び出し

// フォーマット付きアサートマクロ（可変引数を受け取る）
#define ASSERT_MSG(expr, fmt, ...) ((expr) ? (void)0 : Assert::FailFmt(#expr, __FILE__, __LINE__, __func__, (fmt), __VA_ARGS__))	// 条件式を文字列化してフォーマット付きアサート呼び出し
#else
#define ASSERT(expr) ((void)0)					// リリースビルドではアサートを無効化
#define ASSERT_MSG(expr, fmt, ...) ((void)0)	// リリースビルドではフォーマット付きアサートも無効化
#endif

// VERIFY マクロは常に有効（リリースビルドでも条件式を評価してアサート呼び出し）
#define VERIFY(expr) ((expr) ? (void)0 : Assert::Fail(#expr, __FILE__, __LINE__, __func__, std::string()))	// 条件式を文字化してアサート呼び出し

// フォーマット付き VERIFY マクロ（可変引数を受け取る）
#define VERIFY_MSG(expr, fmt, ...) ((expr) ? (void)0 : Assert::FailFmt(#expr, __FILE__, __LINE__, __func__, (fmt), __VA_ARGS__))	// 条件式を文字化してフォーマット付きアサート呼び出し

#define ASSERT_ALWAYS VERIFY			// 常に評価されるアサート（リリースビルドでも条件式を評価してアサート呼び出し）
#define ASSERT_ALWAYS_MSG VERIFY_MSG	// 常に評価されるフォーマット付きアサート（リリースビルドでも条件式を評価してフォーマット付きアサート呼び出し）

#endif // ASSERT_H
