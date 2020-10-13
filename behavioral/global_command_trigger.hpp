#pragma once

#include "global_command_state.hpp"
#include "abstract_trigger.hpp"
#include "global_command.hpp"
#include "stop_result.hpp"

namespace path_planning { namespace behavioral {

    class GlobalCommandTriggerFactory;
    enum class GlobalCommandTriggerID {TURN_RIGHT, TURN_LEFT, HEAD_STRAIGHT, STOP, NO_GLOBAL_COMMAND, HAS_GLOBAL_COMMAND, COMMAND_COMPLETE};

    class GlobalCommandTrigger: public AbstractTrigger<GlobalCommandTriggerID, GlobalCommandState> {
        protected: 
            GlobalCommandTrigger(GlobalCommandTriggerID id, GlobalCommandState state): AbstractTrigger(id, state) {};
    };
    
    /** Condition
    The current global command is RIGHT_TURN
     */
    class TriggerTurnRight : public GlobalCommandTrigger {
        friend GlobalCommandTriggerFactory;
        public:
            bool test() override;
        private:
            TriggerTurnRight(GlobalCommandState global_command_state): 
                GlobalCommandTrigger(GlobalCommandTriggerID::TURN_RIGHT, global_command_state) {};
    };

    /** Condition
    The current global command is LEFT_TURN
     */
    class TriggerTurnLeft : public GlobalCommandTrigger {
        friend GlobalCommandTriggerFactory;
        public:
            bool test() override;
        private:
            TriggerTurnLeft(GlobalCommandState global_command_state): 
                GlobalCommandTrigger(GlobalCommandTriggerID::TURN_LEFT, global_command_state) {};
    };

    /** Condition
    The current global command is STOP
     */
    class TriggerStop : public GlobalCommandTrigger {
        friend GlobalCommandTriggerFactory;
        public:
            bool test() override;
        private:
            TriggerStop(GlobalCommandState global_command_state): 
                GlobalCommandTrigger(GlobalCommandTriggerID::STOP, global_command_state) {};
    };

    /** Condition
    The current global command is STRAIGHT
     */
    class TriggerHeadStraight : public GlobalCommandTrigger {
        friend GlobalCommandTriggerFactory;
        public:
            bool test() override;
        private:
            TriggerHeadStraight(GlobalCommandState global_command_state): 
                GlobalCommandTrigger(GlobalCommandTriggerID::HEAD_STRAIGHT, global_command_state) {};
    };

    /** Condition
    The current global command is empty
     */
    class TriggerNoGlobalCommand : public GlobalCommandTrigger {
        friend GlobalCommandTriggerFactory;
        public:
            bool test() override;
        private:
            TriggerNoGlobalCommand(GlobalCommandState global_command_state): 
                GlobalCommandTrigger(GlobalCommandTriggerID::NO_GLOBAL_COMMAND, global_command_state) {};
    };

    /** Condition
    The current global command's cmd is NONE. 
     */
    class TriggerGlobalCommandComplete : public GlobalCommandTrigger {
        friend GlobalCommandTriggerFactory;
        public:
            bool test() override;
        private:
            TriggerGlobalCommandComplete(GlobalCommandState global_command_state): 
                GlobalCommandTrigger(GlobalCommandTriggerID::COMMAND_COMPLETE, global_command_state) {};
    };

    /** Condition
    Has global commands other than NONE.
     */
    class TriggerHasGlobalCommand : public GlobalCommandTrigger {
        friend GlobalCommandTriggerFactory;
        public:
            bool test() override;
        private:
            TriggerHasGlobalCommand(GlobalCommandState global_command_state): 
                GlobalCommandTrigger(GlobalCommandTriggerID::HAS_GLOBAL_COMMAND, global_command_state) {};
    };
}}