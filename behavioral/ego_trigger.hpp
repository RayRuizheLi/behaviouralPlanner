#pragma once

#include <ros/ros.h>
#include <path_planning/DynamicConfig.h>
#include <memory>
#include <exception>
#include <algorithm>
#include "ego_state.hpp"
#include "DynamicNode/parameter_config.h"
#include "common/stopline.hpp"
#include "abstract_trigger.hpp"

namespace path_planning { namespace behavioral {

    class EgoTriggerFactory;

    enum class EgoTriggerID {CHANGE_LANE_LEFT, CHANGE_LANE_RIGHT, LANE_STABILIZED, STOP_LINE_ON_ROUTE, NO_STOP_LINE_ON_ROUTE, NO_GLOBAL_COMMANDS, HAS_GLOBAL_COMMANDS};

    class EgoTrigger : public AbstractTrigger<EgoTriggerID, EgoState> {
        protected:
            EgoTrigger(EgoTriggerID id, EgoState state): AbstractTrigger(id, state) {};
    };

    /** Conditions
     * 1) Based on road topolgy we should change lanes to the left
     *      Defined as TBD
     * OR
     * 2) Based on local planner's lateral devitation we should change lanes to the left
     *      Defined as the differnce in lateral deviation of the final state compared
     *      to the start state in the optional trajectory being greater than 
     *      config('LANE_CHANGE_LAT_DEV_CONDITION')
     */
    class TriggerLeftLaneChange : public EgoTrigger {
        friend EgoTriggerFactory;
        public:
            bool test() override;
        private:
            TriggerLeftLaneChange(EgoState ego_state): 
                EgoTrigger(EgoTriggerID::CHANGE_LANE_LEFT, ego_state) {};

    };

    /** Conditions
     * 1) Based on road topolgy we should change lanes to the right
     *      Defined as TBD
     * OR
     * 2) Based on local planner's lateral devitation we should change lanes to the right
     *      Defined as the differnce in lateral deviation of the final state compared
     *      to the start state in the optional trajectory being less than 
     *      -config('LANE_CHANGE_LAT_DEV_CONDITION')
     */
    class TriggerRightLaneChange : public EgoTrigger {
        friend EgoTriggerFactory;
        public:
            bool test() override;
        private:
            TriggerRightLaneChange(EgoState ego_state):
                EgoTrigger(EgoTriggerID::CHANGE_LANE_RIGHT, ego_state) {};

    };

    /** Conditions
     * Current lateral deviation is within config('LANE_STABILIZED_EPSILON') 
     * to the optimal lateraldeviation for at least 3 steps back in history
     */
    class TriggerLaneStabilized : public EgoTrigger {
        friend EgoTriggerFactory;
        public:
            bool test() override;
        private:
            TriggerLaneStabilized(EgoState ego_state): 
                EgoTrigger(EgoTriggerID::LANE_STABILIZED, ego_state) {};

    };

    /** Conditions
     * When a world stopping point is present
    */
    class TriggerStopLineOnRoute : public EgoTrigger {
        friend EgoTriggerFactory;
        public:
            bool test() override;
        private:
            TriggerStopLineOnRoute(EgoState ego_state): 
                EgoTrigger(EgoTriggerID::STOP_LINE_ON_ROUTE, ego_state) {};

    };

    /** Conditions
     * When a world stopping point is not present
     */
    class TriggerNoStopLineOnRoute : public EgoTrigger {
        friend EgoTriggerFactory;
        public:
            bool test() override;
        private:
            TriggerNoStopLineOnRoute(EgoState ego_state): 
                EgoTrigger(EgoTriggerID::NO_STOP_LINE_ON_ROUTE, ego_state) {};

    };

    /** Conditions
     * The global commands list queue is empty 
     */
    class TriggerNoGlobalCommands: public EgoTrigger {
        friend EgoTriggerFactory;
        public:
            bool test() override;
        private:
            TriggerNoGlobalCommands(EgoState ego_state): 
                EgoTrigger(EgoTriggerID::NO_GLOBAL_COMMANDS, ego_state) {};

    };

    /** Conditions
     * The global commands list queue is not empty 
     */
    class TriggerHasGlobalCommands: public EgoTrigger {
        friend EgoTriggerFactory;
        public:
            bool test() override;
        private:
            TriggerHasGlobalCommands(EgoState ego_state): 
                EgoTrigger(EgoTriggerID::HAS_GLOBAL_COMMANDS, ego_state) {};

    };

}}