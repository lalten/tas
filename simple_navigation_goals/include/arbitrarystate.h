#ifndef ARBITRARYSTATE_H
#define ARBITRARYSTATE_H

#include "state.h"

class ArbitraryState : public State {
public:
    ArbitraryState(double x, double y, double z, double rho);
    int transit(void);
};

#endif // ARBITRARYSTATE_H
