#pragma once

#include "abstract_trigger_factory.hpp"
#include "pedestrian_state.hpp"
#include "pedestrian_trigger.hpp"

namespace path_planning { namespace behavioral {

    class PedestrianTriggerFactory : public AbstractTriggerFactory<PedestrianTrigger, PedestrianState, PedestrianTriggerID> {
        private:
            std::shared_ptr<PedestrianTrigger> enumToType(PedestrianTriggerID id, PedestrianState state) const override;
    };

}}