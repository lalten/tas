#include "../include/statemachine.h"

StateMachine::StateMachine(){

}

void StateMachine::addState(State* state) {
    this->states.insert(this->states.begin(), state);
}

int StateMachine::transit(){
    int result = this->states.back()->transit();
    this->states.pop_back();
    return result;
}

int StateMachine::size() {
    return this->states.size();
}

State* StateMachine::getStateAt(int index) throw(NoStateAvailableException) {
    if(this->states.size() > index) {
        return this->states.at(index);
    }
    throw NoStateAvailableException(index);
}
