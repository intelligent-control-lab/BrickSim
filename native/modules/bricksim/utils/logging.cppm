export module bricksim.utils.logging;

import std;
import bricksim.vendor;

namespace bricksim {

// Correspond to carb logging levels
enum class LogLevel : std::int32_t {
	verbose = -2,
	info = -1,
	warn = 0,
	error = 1,
	fatal = 2
};

template <LogLevel Level, class... Args>
void log_impl(std::format_string<Args...> fmt, const std::source_location &loc,
              Args &&...args) {
	constexpr std::int32_t level = static_cast<std::int32_t>(Level);
	using namespace bricksim::vendor::carb;
	if (!(g_carbLogFn() && g_carbLogLevel() <= level))
		return;
	g_carbLogFn()(g_carbClientName(), level, loc.file_name(),
	              loc.function_name(), static_cast<int>(loc.line()), "%s",
	              std::format(fmt, std::forward<Args>(args)...).c_str());
}

#define DEFINE_LOG_WRAPPER(level)                                              \
	export template <class... Args> struct log_##level {                       \
		log_##level(std::format_string<Args...> fmt, Args &&...args,           \
		            const std::source_location &loc =                          \
		                std::source_location::current()) {                     \
			log_impl<LogLevel::level>(fmt, loc, std::forward<Args>(args)...);  \
		}                                                                      \
	};                                                                         \
	export template <class... Args>                                            \
	log_##level(std::format_string<Args...>, Args &&...)                       \
	    ->log_##level<Args...>;

DEFINE_LOG_WRAPPER(verbose)
DEFINE_LOG_WRAPPER(info)
DEFINE_LOG_WRAPPER(warn)
DEFINE_LOG_WRAPPER(error)
DEFINE_LOG_WRAPPER(fatal)

#undef DEFINE_LOG_WRAPPER

} // namespace bricksim
