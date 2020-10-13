#pragma once

#include <string>

namespace path_planning { namespace behavioral {

    /** 
     * StateAction is an enum class, defined in the header file of an AbstractState dervied class
     */
    template<typename StateAction>
    class AbstractState {

        public:
            StateAction getAction() const;
            void setAction(StateAction action);
            virtual std::string serializeAction() const = 0;
            virtual std::string serialize() const = 0;
        
        protected:
            AbstractState(StateAction action);
            StateAction _action;
    };

} }