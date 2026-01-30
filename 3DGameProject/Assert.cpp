#include "Assert.h"

#include <iostream>
#include <mutex>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <string>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

// static member definition
Assert::Handler Assert::s_handler = nullptr;
static std::mutex s_handler_mutex;

void Assert::SetHandler(Handler handler) {
	std::lock_guard<std::mutex> lk(s_handler_mutex);
	s_handler = std::move(handler);
}

void Assert::ClearHandler() {
	std::lock_guard<std::mutex> lk(s_handler_mutex);
	s_handler = nullptr;
}

static std::string FormatV(const char* fmt, va_list args) {
	if (!fmt) return std::string();
	va_list tmp;
	va_copy(tmp, args);
	int needed = std::vsnprintf(nullptr, 0, fmt, tmp);
	va_end(tmp);
	if (needed <= 0) return std::string();
	std::string buf((size_t)needed + 1, '\0');
	std::vsnprintf(buf.data(), buf.size(), fmt, args);
	if (!buf.empty() && buf.back() == '\0') buf.pop_back();
	return buf;
}

void Assert::FailFmt(const char* expr, const char* file, int line, const char* func, const char* fmt, ...) noexcept {
	va_list args;
	va_start(args, fmt);
	std::string msg = FormatV(fmt, args);
	va_end(args);
	Fail(expr, file, line, func, msg);
}

void Assert::Fail(const char* expr, const char* file, int line, const char* func, const std::string& message) noexcept {
	std::string out;
	out.reserve(512);
	out += "ASSERT FAILED: ";
	if (expr) out += expr;
	out += "\n at: ";
	if (file) out += file;
	out += ":";
	out += std::to_string(line);
	out += "\n func: ";
	if (func) out += func;
	out += "\n";
	if (!message.empty()) {
		out += " msg: ";
		out += message;
		out += "\n";
	}

	// ハンドラがあれば呼び出す。ハンドラが true を返したらブレーク処理を行う
	{
		std::lock_guard<std::mutex> lk(s_handler_mutex);
		if (s_handler) {
			bool shouldBreak = false;
			try {
				shouldBreak = s_handler(expr, file, line, func, message);
			}
			catch (...) {
				// ハンドラ例外は無視して続行
			}
			std::cerr << out;
			if (shouldBreak) {
#ifdef _WIN32
				if (IsDebuggerPresent()) { DebugBreak(); }
				else { std::abort(); }
#else
				std::raise(SIGTRAP);
				std::abort();
#endif
			}
			else {
				// ハンドラがブレーク指示を出さない場合は続行（必要ならここで abort にする）
				return;
			}
		}
	}

	// デフォルト動作: 標準エラーへ出力してデバッガがあればブレーク、なければ abort
	std::cerr << out;
#ifdef _WIN32
	if (IsDebuggerPresent()) {
		DebugBreak();
	}
	else {
		std::abort();
	}
#else
	std::raise(SIGTRAP);
	std::abort();
#endif
}

void Assert::Check(bool condition, const char* expr, const char* file, int line, const char* func, const char* message) noexcept {
	if (condition) return;
	if (message) {
		Fail(expr, file, line, func, std::string(message));
	}
	else {
		Fail(expr, file, line, func, std::string());
	}
}