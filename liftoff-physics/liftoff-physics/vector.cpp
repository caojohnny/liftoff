#include <iostream>
#include "vector.h"

liftoff::vector::vector() : x(0), y(0), z(0) {
}

liftoff::vector::vector(double x, double y, double z) : x(x), y(y), z(z) {}

liftoff::vector::vector(double k) : x(k), y(k), z(k) {
}

double liftoff::vector::get_x() const {
    return x;
}

liftoff::vector liftoff::vector::set_x(double x) {
    vector::x = x;
    return *this;
}

double liftoff::vector::get_y() const {
    return y;
}

liftoff::vector liftoff::vector::set_y(double y) {
    vector::y = y;
    return *this;
}

double liftoff::vector::get_z() const {
    return z;
}

liftoff::vector liftoff::vector::set_z(double z) {
    vector::z = z;
    return *this;
}

liftoff::vector liftoff::vector::set(const vector &vec) {
    set_x(vec.x);
    set_y(vec.y);
    set_z(vec.z);
    return *this;
}

liftoff::vector liftoff::vector::add(const vector &vec) {
    set_x(vector::x + vec.x);
    set_y(vector::y + vec.y);
    set_z(vector::z + vec.z);
    return *this;
}

liftoff::vector liftoff::vector::sub(const vector &vec) {
    set_x(vector::x - vec.x);
    set_y(vector::y - vec.y);
    set_z(vector::z - vec.z);
    return *this;
}

liftoff::vector liftoff::vector::mul(const vector &vec) {
    set_x(vector::x * vec.x);
    set_y(vector::y * vec.y);
    set_z(vector::z * vec.z);
    return *this;
}

liftoff::vector liftoff::vector::div(const vector &vec) {
    set_x(vector::x / vec.x);
    set_y(vector::y / vec.y);
    set_z(vector::z / vec.z);
    return *this;
}

void liftoff::vector::print_vector() {
    std::cout << "(" << get_x() << ", " << get_y() << ", " << get_z() << ")" << std::endl;
}
