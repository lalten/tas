#ifndef STATE_H
#define STATE_H

class State {
private:
    double x, y, z;
    double rho;
public:
    State(double x, double y, double z, double rho);
    void setX(double x);
    double getX();
    void setY(double y);
    double getY();
    void setZ(double z);
    double getZ();
    void setRho(double rho);
    double getRho();
    virtual int transit(void){};
};

#endif // STATE_H
