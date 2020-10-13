#pragma once

#include "abstract_trigger.hpp"
#include "stop_sign_state.hpp"

namespace path_planning { namespace behavioral {

    class StopSignTriggerFactory;

    enum class StopSignTriggerID {PRESENT_ON_ROUTE, ACTIVATED, DEACTIVATED, EXPIRED};

    class StopSignTrigger : public AbstractTrigger<StopSignTriggerID, StopSignState> {
        protected:
            StopSignTrigger(StopSignTriggerID id, StopSignState state): AbstractTrigger(id, state) {};
    };

    /** 
     * Condition: When the stop sign's stop line cuts accross the reference line
     */

    class StopSignPresentOnRouteTrigger: public StopSignTrigger {
        friend StopSignTriggerFactory;
        public:
            bool test() override;

            StopResult stop_result;
        private:
            StopSignPresentOnRouteTrigger(StopSignState state): StopSignTrigger(StopSignTriggerID::PRESENT_ON_ROUTE, state) {};
    };


    /**
     * Condition: When the ego vehicle is within config('STOP_SIGN_DISTANCE_EPSILON') of the stopping point with near zero velocity
     */
    class StopSignActivatedTrigger: public StopSignTrigger {
        friend StopSignTriggerFactory;
        public:
            bool test() override;

        private:
            StopSignActivatedTrigger(StopSignState state): StopSignTrigger(StopSignTriggerID::ACTIVATED, state) {};
    };

    /**
     * Condition: When StopSignActivatedTrigger's condition does not hold
     */
    class StopSignDeactivatedTrigger: public StopSignTrigger {
        friend StopSignTriggerFactory;
        public:
            bool test() override;

        private:
            StopSignDeactivatedTrigger(StopSignState state): StopSignTrigger(StopSignTriggerID::DEACTIVATED, state) {};
    };

    /**
     * Condition: When the activated duration is greater than config('STOP_SIGN_TIME')
     */

    class StopSignExpiredTrigger: public StopSignTrigger {
        friend StopSignTriggerFactory;
        public:
            bool test() override;

        private:
            StopSignExpiredTrigger(StopSignState state): StopSignTrigger(StopSignTriggerID::EXPIRED, state) {};
    };

}}