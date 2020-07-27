#include "matrix.h"

// Required precision for floating point values in order to perform the necessary polynomial
// regressions
// (pretty sure this is actually around 80, but 256 just to be safe :>)
static const int REQ_PRECISION = 256;

liftoff::matrix::matrix(size_t size) : liftoff::matrix(size, size) {
}

liftoff::matrix::matrix(size_t rows, size_t columns) {
    cells.reserve(rows);
    for (int r = 0; r < rows; ++r) {
        std::vector<mpf_class> row;
        row.reserve(columns);

        for (int c = 0; c < columns; ++c) {
            row.emplace_back(0, REQ_PRECISION);
        }

        cells.push_back(row);
    }
}

unsigned int liftoff::matrix::rows() const {
    return cells.size();
}

unsigned int liftoff::matrix::columns() const {
    return cells[0].size();
}

std::vector<std::vector<mpf_class>>::reference &liftoff::matrix::operator[](size_t row) {
    return cells[row];
}

const std::vector<mpf_class> &liftoff::matrix::operator[](size_t row) const {
    return cells[row];
}

std::string liftoff::matrix::to_matlab() const {
    unsigned long rows = cells.size();
    std::string result;
    if (rows > 1) {
        result += "[";
    }

    for (int r = 0; r < rows; ++r) {
        result += "[";
        for (int c = 0; c < columns(); ++c) {
            result += std::to_string(cells[r][c].get_d());
            result += " ";
        }
        result += "]";

        if (r != columns() - 1) {
            result += ";";
        }
    }

    if (rows > 1) {
        result += "]";
    }

    return result;
}
