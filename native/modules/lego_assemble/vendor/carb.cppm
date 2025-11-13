module;

#include <format>
#include <source_location>

#include <carb/BindingsUtils.h>
#include <carb/InterfaceUtils.h>
#include <carb/logging/Log.h>

// Declare as a Carbonite bindings module for Python so logging and builtins
// are registered even when imported outside Kit, and to define CARB globals.
CARB_BINDINGS("lego_assemble", "python")

export module lego_assemble.vendor.carb;

export namespace carb {
using carb::getCachedInterface;
} // namespace carb

namespace lego_assemble {

template <class... Args>
void _log(int level, std::source_location loc, std::format_string<Args...> fmt,
          Args &&...args) {
	if (!(g_carbLogFn && g_carbLogLevel <= level))
		return;
	g_carbLogFn(g_carbClientName, level, loc.file_name(), loc.function_name(),
	            static_cast<int>(loc.line()), "%s",
	            std::format(fmt, std::forward<Args>(args)...).c_str());
}

export template <class... Args>
void log_verbose(std::format_string<Args...> fmt, Args &&...args) {
	_log(carb::logging::kLevelVerbose, std::source_location::current(), fmt,
	     std::forward<Args>(args)...);
}

export template <class... Args>
void log_info(std::format_string<Args...> fmt, Args &&...args) {
	_log(carb::logging::kLevelInfo, std::source_location::current(), fmt,
	     std::forward<Args>(args)...);
}

export template <class... Args>
void log_warn(std::format_string<Args...> fmt, Args &&...args) {
	_log(carb::logging::kLevelWarn, std::source_location::current(), fmt,
	     std::forward<Args>(args)...);
}

export template <class... Args>
void log_error(std::format_string<Args...> fmt, Args &&...args) {
	_log(carb::logging::kLevelError, std::source_location::current(), fmt,
	     std::forward<Args>(args)...);
}

export template <class... Args>
void log_fatal(std::format_string<Args...> fmt, Args &&...args) {
	_log(carb::logging::kLevelFatal, std::source_location::current(), fmt,
	     std::forward<Args>(args)...);
}

} // namespace lego_assemble
