export module lego_assemble.brick_naming;

import std;
import lego_assemble.utils.strings;
import lego_assemble.vendor.carb;
import lego_assemble.vendor.pxr;

namespace lego_assemble {

export using EnvId = std::int32_t;
export using BrickId = std::int64_t;

export constexpr EnvId kNoEnv = -1;

static const pxr::SdfPath WorldPath("/World");
static const pxr::SdfPath EnvRootPath("/World/envs");

export pxr::SdfPath pathForEnv(EnvId env_id) {
	if (env_id == kNoEnv) {
		return WorldPath;
	} else {
		auto env_name = std::format("env_{}", env_id);
		return EnvRootPath.AppendChild(pxr::TfToken(env_name));
	}
}

export std::string nameForBrick(BrickId brick_id) {
	return std::format("Brick_{}", brick_id);
}

export pxr::SdfPath pathForBrick(EnvId env_id, BrickId brick_id) {
	return pathForEnv(env_id).AppendChild(pxr::TfToken(nameForBrick(brick_id)));
}

export std::optional<EnvId> envIdFromPath(const pxr::SdfPath &path) {
	if (path == WorldPath) {
		return kNoEnv;
	}
	if (path.GetParentPath() != EnvRootPath) {
		return std::nullopt;
	}
	// Assume GetName() yields std::string (same as your original code).
	auto name_str = path.GetName();
	std::string_view name{name_str};

	if (!eat_prefix(name, "env_")) {
		return std::nullopt;
	}
	return parse_int<EnvId>(name);
}

export std::optional<BrickId> brickIdFromName(const std::string &name_in) {
	std::string_view name{name_in};
	if (!eat_prefix(name, "Brick_")) {
		return std::nullopt;
	}
	return parse_int<BrickId>(name);
}

export std::optional<std::tuple<EnvId, BrickId>>
brickIdFromPath(const pxr::SdfPath &path) {
	auto env_id = envIdFromPath(path.GetParentPath());
	if (!env_id) {
		return std::nullopt;
	}
	auto name_str = path.GetName();
	auto brick_id = brickIdFromName(name_str);
	if (!brick_id) {
		return std::nullopt;
	}
	return {{*env_id, *brick_id}};
}

export std::string nameForConn(BrickId brick0, BrickId brick1) {
	return std::format("Conn_{}_{}", brick0, brick1);
}

export pxr::SdfPath pathForConn(EnvId env_id, BrickId brick0, BrickId brick1) {
	return pathForEnv(env_id).AppendChild(
	    pxr::TfToken(nameForConn(brick0, brick1)));
}

export std::optional<std::tuple<BrickId, BrickId>>
connFromName(const std::string &name_in) {
	std::string_view sv{name_in};
	if (!eat_prefix(sv, "Conn_")) {
		return std::nullopt;
	}

	// Split on '_' without substr: make two string_views by indices.
	const auto delim_pos = sv.find('_');
	if (delim_pos == std::string_view::npos) {
		return std::nullopt;
	}
	std::string_view a{sv.data(), delim_pos};
	std::string_view b{sv.data() + delim_pos + 1, sv.size() - (delim_pos + 1)};

	auto brick0 = parse_int<BrickId>(a);
	if (!brick0)
		return std::nullopt;
	auto brick1 = parse_int<BrickId>(b);
	if (!brick1)
		return std::nullopt;

	return {{*brick0, *brick1}};
}

export std::optional<std::tuple<EnvId, BrickId, BrickId>>
connFromPath(const pxr::SdfPath &path) {
	auto env_id = envIdFromPath(path.GetParentPath());
	if (!env_id) {
		return std::nullopt;
	}
	auto name_str = path.GetName();
	auto conn_ids = connFromName(name_str);
	if (!conn_ids) {
		return std::nullopt;
	}
	auto [brick0, brick1] = *conn_ids;
	return {{*env_id, brick0, brick1}};
}

// Generate a connection path for two bricks.
// If their naming follows the "Brick_<id>" pattern, the "Brick_" prefix is omitted.
// Otherwise, the full names are used.
// The resulting connection name is "Conn_<brick0>_<brick1>".
// The connection path is created under the common parent of the two bricks.
// If the two bricks have different parents, a warning is logged.
export pxr::SdfPath safeConnPathForBricks(const pxr::SdfPath &brick0,
                                          const pxr::SdfPath &brick1) {
	auto brick0_parent = brick0.GetParentPath();
	auto brick1_parent = brick1.GetParentPath();
	if (brick0_parent != brick1_parent) {
		log_warn("Bricks %s and %s have different parents!", brick0.GetText(),
		         brick1.GetText());
	}

	// Use views to possibly drop the "Brick_" prefix without allocations.
	auto name0_str = brick0.GetName();
	auto name1_str = brick1.GetName();
	std::string_view name0{name0_str};
	std::string_view name1{name1_str};

	if (name0.starts_with("Brick_") && name1.starts_with("Brick_")) {
		name0.remove_prefix(6);
		name1.remove_prefix(6);
	}

	auto conn_name = std::format("Conn_{}_{}", name0, name1);
	return brick0_parent.AppendChild(pxr::TfToken(conn_name));
}

}; // namespace lego_assemble
