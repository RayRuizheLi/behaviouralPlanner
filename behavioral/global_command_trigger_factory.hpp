#pragma once

#include "abstract_trigger_factory.hpp"
#include "global_command_state.hpp"
#include "global_command_trigger.hpp"

namespace path_planning { namespace behavioral {

    class GlobalCommandTriggerFactory : public AbstractTriggerFactory<GlobalCommandTrigger, GlobalCommandState, GlobalCommandTriggerID> {
        private:
            std::shared_ptr<GlobalCommandTrigger> enumToType(GlobalCommandTriggerID id, GlobalCommandState state) const override;
    };

}}