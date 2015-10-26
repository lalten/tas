
#include "../include/arbitrarystate.h"
#include "../include/statemachine.h"
#include "../include/NoStateAvailableException.h"

int main(int argc, char** argv) {

    StateMachine* statemachine = new StateMachine();

    ArbitraryState* arbitraryState1 = new ArbitraryState(1,0,0,1);
    ArbitraryState* arbitraryState2 = new ArbitraryState(1,0,0,1);
    ArbitraryState* arbitraryState3 = new ArbitraryState(1,0,0,1);
    ArbitraryState* arbitraryState4 = new ArbitraryState(1,0,0,1);

    statemachine->addState(arbitraryState1);
    statemachine->addState(arbitraryState2);
    statemachine->addState(arbitraryState3);
    statemachine->addState(arbitraryState4);

    try {
        ArbitraryState = statemachine->getStateAt(3);
    } catch (NoStateAvailableException e) {
        //catch exception
    }

    while() {
        int result = statemachine->transit();
        if(result > 0) {
            // do it again
        } else {
            // proceed to next step
        }

    }

}
