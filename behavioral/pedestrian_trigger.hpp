#pragma once

#include "abstract_trigger.hpp"
#include "pedestrian_state.hpp"
#include "stop_result.hpp"

namespace path_planning { namespace behavioral {

    class PedestrianTriggerFactory;

    enum class PedestrianTriggerID {IN_ROI, MOVING, NOT_MOVING, NOT_IN_ROI, STOPPED};

    class PedestrianTrigger : public AbstractTrigger<PedestrianTriggerID, PedestrianState> {
        protected:
            PedestrianTrigger(PedestrianTriggerID id, PedestrianState state): AbstractTrigger(id, state) {};
    };

    /** 
     * Condition: When the pedestrian is near a crossing lane and the reference line
     */

    class PedestrianInRoITrigger: public PedestrianTrigger {
        friend PedestrianTriggerFactory;
        public:
            bool test() override;

            StopResult stop_result;

        private:
            PedestrianInRoITrigger(PedestrianState state): PedestrianTrigger(PedestrianTriggerID::IN_ROI, state) {};
    };


    /** 
     * Condition: When the magnitute of the pedestrian's velocity vector is greater than config('PEDESTRIAN_MOVEMENT_EPSILON')
     */

    class PedestrianMovingTrigger: public PedestrianTrigger {
        friend PedestrianTriggerFactory;
        public:
            bool test() override;

        private:
            PedestrianMovingTrigger(PedestrianState state): PedestrianTrigger(PedestrianTriggerID::MOVING, state) {};
    };

    /** 
     * Condition: When the magnitute of the pedestrian's velocity vector is less than config('PEDESTRIAN_MOVEMENT_EPSILON')
     */

    class PedestrianNotMovingTrigger: public PedestrianTrigger {
        friend PedestrianTriggerFactory;
        public:
            bool test() override;
            
        private:
            PedestrianNotMovingTrigger(PedestrianState state): PedestrianTrigger(PedestrianTriggerID::NOT_MOVING, state) {};
    };

    /** 
     * Condition: When the pedestrian is not near a crossing lane and is therefore not of concern
     */

    class PedestrianNotInRoI: public PedestrianTrigger {
        friend PedestrianTriggerFactory;
        public:
            bool test() override;

        private:
            PedestrianNotInRoI(PedestrianState state): PedestrianTrigger(PedestrianTriggerID::NOT_IN_ROI, state) {};
    };

    /** 
     * Condition: When a pedestrian has been paused for more than config('PEDESTRIAN_STOPPED_TIMER')
     */

    class PedestrianStoppedTrigger: public PedestrianTrigger {
        friend PedestrianTriggerFactory;
        public:
            bool test() override;

        private:
            PedestrianStoppedTrigger(PedestrianState state): PedestrianTrigger(PedestrianTriggerID::STOPPED, state) {};
    };

}}