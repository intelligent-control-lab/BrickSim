#pragma once

#include "LegoTopology.h"

#include <nlohmann/json.hpp>

namespace lego_assemble {

void to_json(nlohmann::ordered_json &j, const LegoTopology::Brick &brick);
void from_json(const nlohmann::ordered_json &j, LegoTopology::Brick &brick);

void to_json(nlohmann::ordered_json &j, const LegoTopology::Connection &brick);
void from_json(const nlohmann::ordered_json &j,
               LegoTopology::Connection &brick);

void to_json(nlohmann::ordered_json &j, const LegoTopology::PoseHint &brick);
void from_json(const nlohmann::ordered_json &j, LegoTopology::PoseHint &brick);

void to_json(nlohmann::ordered_json &j, const LegoTopology &brick);
void from_json(const nlohmann::ordered_json &j, LegoTopology &brick);

} // namespace lego_assemble
