#pragma once

#include "abstract_trigger_factory.hpp"
#include "stop_sign_state.hpp"
#include "stop_sign_trigger.hpp"

namespace path_planning { namespace behavioral {

    class StopSignTriggerFactory : public AbstractTriggerFactory<StopSignTrigger, StopSignState, StopSignTriggerID> {
        private:
            std::shared_ptr<StopSignTrigger> enumToType(StopSignTriggerID id, StopSignState state) const override;
    };

}}