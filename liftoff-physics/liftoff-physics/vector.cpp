#include <string>
#include <cmath>
#include "vector.h"

namespace liftoff {
    vector::vector() : vector(0) {
    }

    vector::vector(double k) : vector(k, k, k) {
    }

    vector::vector(double vec_x, double vec_y, double vec_z) : x(vec_x), y(vec_y), z(vec_z) {
    }

    double vector::get_x() const {
        return x;
    }

    vector &vector::set_x(double new_x) {
        x = new_x;
        return *this;
    }

    double vector::get_y() const {
        return y;
    }

    vector &vector::set_y(double new_y) {
        y = new_y;
        return *this;
    }

    double vector::get_z() const {
        return z;
    }

    vector &vector::set_z(double new_z) {
        z = new_z;
        return *this;
    }

    vector &vector::set(const vector &vec) {
        set_x(vec.x);
        set_y(vec.y);
        set_z(vec.z);
        return *this;
    }

    vector &vector::add(const vector &vec) {
        set_x(vector::x + vec.x);
        set_y(vector::y + vec.y);
        set_z(vector::z + vec.z);
        return *this;
    }

    vector &vector::sub(const vector &vec) {
        set_x(vector::x - vec.x);
        set_y(vector::y - vec.y);
        set_z(vector::z - vec.z);
        return *this;
    }

    vector &vector::mul(const vector &vec) {
        set_x(vector::x * vec.x);
        set_y(vector::y * vec.y);
        set_z(vector::z * vec.z);
        return *this;
    }

    vector &vector::div(const vector &vec) {
        set_x(vector::x / vec.x);
        set_y(vector::y / vec.y);
        set_z(vector::z / vec.z);
        return *this;
    }

    double vector::magnitude() const {
        return std::sqrt((x * x) + (y * y) + (z * z));
    }

    bool vector::operator==(const vector &rhs) const {
        return x == rhs.x &&
               y == rhs.y &&
               z == rhs.z;
    }

    vector::operator std::string() const {
        std::string result = "vector(";
        result += std::to_string(x);
        result += ",";
        result += std::to_string(y);
        result += ",";
        result += std::to_string(z);
        result += ")";

        return result;
    }
}
