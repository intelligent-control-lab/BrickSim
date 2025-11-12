export module lego_assemble.utils.usd_envs;

import std;
import lego_assemble.utils.strings;
import lego_assemble.vendor.pxr;

namespace lego_assemble {

export constexpr std::int64_t kNoEnv = -1;

static const pxr::SdfPath WorldPath("/World");
static const pxr::SdfPath EnvRootPath("/World/envs");

export pxr::SdfPath pathForEnv(std::int64_t env_id) {
	if (env_id == kNoEnv) {
		return WorldPath;
	} else {
		auto env_name = std::format("env_{}", env_id);
		return EnvRootPath.AppendChild(pxr::TfToken(env_name));
	}
}

export std::optional<std::int64_t> envIdFromPath(const pxr::SdfPath &path) {
	pxr::SdfPath p = path.GetPrimPath();
	if (!p.IsAbsolutePath()) {
		return std::nullopt;
	}

	// Trim path's length to one more than EnvRootPath
	std::size_t len = EnvRootPath.GetPathElementCount() + 1;
	while (p.GetPathElementCount() > len) {
		p = p.GetParentPath();
	}

	// Check if it's /World/envs/env_*
	if (p.GetPathElementCount() == len) {
		if (p.GetParentPath() == EnvRootPath) {
			const std::string &name = p.GetName();
			std::string_view sv = name;
			if (eat_prefix(sv, "env_")) {
				return parse_int<std::int64_t>(sv);
			}
		}
	}

	// Check if it's in /World
	while (p.GetPathElementCount() > 1) {
		p = p.GetParentPath();
	}
	if (p == WorldPath) {
		return kNoEnv;
	}

	return std::nullopt;
}

} // namespace lego_assemble
