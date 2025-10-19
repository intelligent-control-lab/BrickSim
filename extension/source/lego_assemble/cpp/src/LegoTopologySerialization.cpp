#include "LegoTopologySerialization.h"

namespace lego_assemble {

static constexpr auto SchemaString = "lego_assemble/lego_topology@1";

void to_json(nlohmann::ordered_json &j, const LegoTopology::Brick &brick) {
	j = nlohmann::ordered_json{
	    {"id", brick.id},
	    {"dimensions", brick.dimensions},
	    {"color", brick.color},
	};
}
void from_json(const nlohmann::ordered_json &j, LegoTopology::Brick &brick) {
	j.at("id").get_to(brick.id);
	j.at("dimensions").get_to(brick.dimensions);
	j.at("color").get_to(brick.color);
}

void to_json(nlohmann::ordered_json &j, const LegoTopology::Connection &brick) {
	j = nlohmann::ordered_json{
	    {"parent", brick.parent},
	    {"child", brick.child},
	    {"offset", brick.offset},
	    {"orientation", brick.orientation},
	};
}
void from_json(const nlohmann::ordered_json &j,
               LegoTopology::Connection &brick) {
	j.at("parent").get_to(brick.parent);
	j.at("child").get_to(brick.child);
	j.at("offset").get_to(brick.offset);
	j.at("orientation").get_to(brick.orientation);
}

void to_json(nlohmann::ordered_json &j, const LegoTopology::PoseHint &brick) {
	j = nlohmann::ordered_json{
	    {"brick", brick.brick},
	    {"pos", brick.pos},
	    {"rot", brick.rot},
	};
}
void from_json(const nlohmann::ordered_json &j, LegoTopology::PoseHint &brick) {
	j.at("brick").get_to(brick.brick);
	j.at("pos").get_to(brick.pos);
	j.at("rot").get_to(brick.rot);
}

void to_json(nlohmann::ordered_json &j, const LegoTopology &brick) {
	j = nlohmann::ordered_json{
	    {"schema", SchemaString},
	    {"bricks", brick.bricks},
	    {"connections", brick.connections},
	    {"pose_hints", brick.pose_hints},
	};
}
void from_json(const nlohmann::ordered_json &j, LegoTopology &brick) {
	if (j.contains("bricks")) {
		j.at("bricks").get_to(brick.bricks);
	}
	if (j.contains("connections")) {
		j.at("connections").get_to(brick.connections);
	}
	if (j.contains("pose_hints")) {
		j.at("pose_hints").get_to(brick.pose_hints);
	}
}

} // namespace lego_assemble
