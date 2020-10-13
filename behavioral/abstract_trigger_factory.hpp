#pragma once

#include <ros/ros.h>
#include <memory>

namespace path_planning { namespace behavioral {

    /** 
     * State is a class derived from AbstractState
     * Trigger is a class derived from Abstract Trigger
     * TriggerID is an enum class defined in the header file of Trigger
     */
    template<typename Trigger, typename State, typename TriggerID>
    class AbstractTriggerFactory {

        public:
            boost::optional<std::shared_ptr<Trigger>> tryCreateTrigger(TriggerID id, State state) const;

        private:
            virtual std::shared_ptr<Trigger> enumToType(TriggerID id, State state) const = 0;
    };

}}