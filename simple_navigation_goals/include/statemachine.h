#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include <vector>
#include "state.h"
#include "NoStateAvailableException.h"

class StateMachine {
private:
    std::vector<State*> states;
public:
    StateMachine();
    void addState(State* state);
    int size();
    int transit();
    State* getStateAt(int index) throw(NoStateAvailableException);
};

#endif // STATEMACHINE_H
