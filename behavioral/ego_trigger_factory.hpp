#pragma once

#include <path_planning/DynamicConfig.h>
#include "ego_trigger.hpp"
#include "ego_state.hpp"
#include "abstract_trigger_factory.hpp"

namespace path_planning { namespace behavioral {

    class EgoTriggerFactory : public AbstractTriggerFactory<EgoTrigger, EgoState, EgoTriggerID> {
        private:
            std::shared_ptr<EgoTrigger> enumToType(EgoTriggerID id, EgoState state) const override;
    };

} }