#pragma once

#include <format>
#include <optional>
#include <tuple>

#include <carb/logging/Log.h>

#include <pxr/usd/sdf/path.h>

namespace lego_assemble {

using EnvId = std::int32_t;
using BrickId = std::int64_t;

constexpr EnvId kNoEnv = -1;

inline const pxr::SdfPath WorldPath("/World");
inline const pxr::SdfPath EnvRootPath("/World/envs");

inline pxr::SdfPath pathForEnv(EnvId env_id) {
	if (env_id == kNoEnv) {
		return WorldPath;
	} else {
		auto env_name = std::format("env_{}", env_id);
		return EnvRootPath.AppendChild(pxr::TfToken(env_name));
	}
}

inline std::string nameForBrick(BrickId brick_id) {
	return std::format("Brick_{}", brick_id);
}

inline pxr::SdfPath pathForBrick(EnvId env_id, BrickId brick_id) {
	return pathForEnv(env_id).AppendChild(pxr::TfToken(nameForBrick(brick_id)));
}

inline std::optional<EnvId> envIdFromPath(const pxr::SdfPath &path) {
	if (path == WorldPath) {
		return kNoEnv;
	}
	if (path.GetParentPath() != EnvRootPath) {
		return std::nullopt;
	}
	auto name = path.GetName();
	if (!name.starts_with("env_")) {
		return std::nullopt;
	}
	try {
		return std::stoi(name.substr(4));
	} catch (const std::exception &) {
		return std::nullopt;
	}
}

inline std::optional<BrickId> brickIdFromName(const std::string &name) {
	if (!name.starts_with("Brick_")) {
		return std::nullopt;
	}
	try {
		return std::stoll(name.substr(6));
	} catch (const std::exception &) {
		return std::nullopt;
	}
}

inline std::optional<std::tuple<EnvId, BrickId>>
brickIdFromPath(const pxr::SdfPath &path) {
	auto env_id = envIdFromPath(path.GetParentPath());
	if (!env_id) {
		return std::nullopt;
	}
	auto name = path.GetName();
	auto brick_id = brickIdFromName(name);
	if (!brick_id) {
		return std::nullopt;
	}
	return {{*env_id, *brick_id}};
}

inline std::string nameForConn(BrickId brick0, BrickId brick1) {
	return std::format("Conn_{}_{}", brick0, brick1);
}

inline pxr::SdfPath pathForConn(EnvId env_id, BrickId brick0, BrickId brick1) {
	return pathForEnv(env_id).AppendChild(
	    pxr::TfToken(nameForConn(brick0, brick1)));
}

inline std::optional<std::tuple<BrickId, BrickId>>
connFromName(const std::string &name) {
	if (!name.starts_with("Conn_")) {
		return std::nullopt;
	}
	auto rest = name.substr(5);
	auto delim_pos = rest.find('_');
	if (delim_pos == std::string::npos) {
		return std::nullopt;
	}
	try {
		auto brick0 = std::stoll(rest.substr(0, delim_pos));
		auto brick1 = std::stoll(rest.substr(delim_pos + 1));
		return {{brick0, brick1}};
	} catch (const std::exception &) {
		return std::nullopt;
	}
}

inline std::optional<std::tuple<EnvId, BrickId, BrickId>>
connFromPath(const pxr::SdfPath &path) {
	auto env_id = envIdFromPath(path.GetParentPath());
	if (!env_id) {
		return std::nullopt;
	}
	auto name = path.GetName();
	auto conn_ids = connFromName(name);
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
inline pxr::SdfPath safeConnPathForBricks(const pxr::SdfPath &brick0,
                                          const pxr::SdfPath &brick1) {
	auto brick0_parent = brick0.GetParentPath();
	auto brick1_parent = brick1.GetParentPath();
	if (brick0_parent != brick1_parent) {
		CARB_LOG_WARN("Bricks %s and %s have different parents!",
		              brick0.GetText(), brick1.GetText());
	}
	auto name0 = brick0.GetName();
	auto name1 = brick1.GetName();
	if (name0.starts_with("Brick_") && name1.starts_with("Brick_")) {
		name0 = name0.substr(6);
		name1 = name1.substr(6);
	}
	auto conn_name = std::format("Conn_{}_{}", name0, name1);
	return brick0_parent.AppendChild(pxr::TfToken(conn_name));
}

}; // namespace lego_assemble
