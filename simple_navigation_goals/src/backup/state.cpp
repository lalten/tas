#include "../include/state.h"

State::State(double x, double y, double z, double rho) {
    this->rho = rho;
    this->x = x;
    this->y = y;
    this->z = z;
}

void State::setX(double x) {
    this->x = x;
}

void State::setY(double y) {
    this->y = y;
}

void State::setZ(double z) {
    this->z = z;
}

void State::setRho(double rho) {
    this->rho = rho;
}

double State::getX() {
    return this->x;
}

double State::getY() {
    return this->y;
}

double State::getZ() {
    return this->z;
}

double State::getRho() {
    return this->rho;
}

