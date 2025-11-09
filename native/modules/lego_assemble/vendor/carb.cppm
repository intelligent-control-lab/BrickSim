module;

#include <source_location>

#include <carb/InterfaceUtils.h>
#include <carb/logging/Log.h>
#include <carb/BindingsUtils.h>

// Declare as a Carbonite bindings module for Python so logging and builtins
// are registered even when imported outside Kit, and to define CARB globals.
CARB_BINDINGS("lego_assemble", "python")

export module lego_assemble.vendor.carb;

export namespace carb {
using carb::getCachedInterface;
} // namespace carb

namespace lego_assemble {

template <class... Args>
void _log(int level, std::source_location loc, const char *fmt,
          Args &&...args) {
	if (!(g_carbLogFn && g_carbLogLevel <= level))
		return;
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wformat-security"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-security"
#endif
	g_carbLogFn(g_carbClientName, level, loc.file_name(), loc.function_name(),
	            static_cast<int>(loc.line()), fmt, std::forward<Args>(args)...);
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
}

export template <class... Args>
void log_verbose(const char *fmt, Args &&...args) {
	_log(carb::logging::kLevelVerbose, std::source_location::current(), fmt,
	     std::forward<Args>(args)...);
}

export template <class... Args> void log_info(const char *fmt, Args &&...args) {
	_log(carb::logging::kLevelInfo, std::source_location::current(), fmt,
	     std::forward<Args>(args)...);
}

export template <class... Args> void log_warn(const char *fmt, Args &&...args) {
	_log(carb::logging::kLevelWarn, std::source_location::current(), fmt,
	     std::forward<Args>(args)...);
}

export template <class... Args>
void log_error(const char *fmt, Args &&...args) {
	_log(carb::logging::kLevelError, std::source_location::current(), fmt,
	     std::forward<Args>(args)...);
}

export template <class... Args>
void log_fatal(const char *fmt, Args &&...args) {
	_log(carb::logging::kLevelFatal, std::source_location::current(), fmt,
	     std::forward<Args>(args)...);
}

} // namespace lego_assemble
