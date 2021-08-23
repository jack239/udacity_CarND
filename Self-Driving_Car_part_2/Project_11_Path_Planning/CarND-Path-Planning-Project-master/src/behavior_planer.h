#ifndef BEHAVIOR_PLANER_H_
#define BEHAVIOR_PLANER_H_
#include "state.h"

void update_state(
    VehicleState& state,
    const VehiclePos& ego,
    const std::vector<VehiclePos>& objects
);
#endif //BEHAVIOR_PLANER_H_
