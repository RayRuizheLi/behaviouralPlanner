#pragma once

namespace path_planning { namespace behavioral {

    /** 
     * TriggerID is an enum class, defined in the header file of an AbstractTrigger derived class
     * State is a class derived from AbstractState
     */
    template<typename TriggerID, typename State>
    class AbstractTrigger {
        public:
            TriggerID getID() const;

            virtual bool test() = 0;

        protected:
            AbstractTrigger(TriggerID id, State state);

            State _state;
            const TriggerID _id;
    };

} }