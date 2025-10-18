#pragma once

#include "LegoGraph.h"

namespace lego_assemble {

// Initialize and tear down the joint manager.
bool initLegoJointManager();
bool deinitLegoJointManager();

void setLegoThresholds(const LegoGraph::Thresholds &thresholds);
void getLegoThresholds(LegoGraph::Thresholds &out);

} // namespace lego_assemble
