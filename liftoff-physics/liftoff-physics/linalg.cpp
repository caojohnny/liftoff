#include "linalg.h"
#include "matrix.h"

#include <iostream>

// Required precision for floating point values in order to perform the necessary polynomial
// regressions
// (pretty sure this is actually around 80, but 256 just to be safe :>)
static const int REQ_PRECISION = 256;

// Doolittle LU decomposition with partial pivot (L is the identity diagonal)
// Adapted from: https://en.wikipedia.org/wiki/LU_decomposition#C_code_examples
// Adapted from: https://www.codewithc.com/lu-decomposition-algorithm-flowchart/
static int lup(liftoff::matrix &mat, std::vector<int> &perm) {
    int pivot_count = 0;

    // Each element of the permutation vector represents a row of the permutation matrix
    // The integer is the index on that row at which the 1 is located
    for (int i = 0; i < mat.rows(); ++i) {
        perm.push_back(i);
    }

    // Pivoting portion
    for (int row = 0; row < mat.rows(); ++row) {
        int max_row = row;
        mpf_class max_cell{0, REQ_PRECISION};

        // Find the max cell on the diagonal cell for the current row
        for (int r = row; r < mat.rows(); ++r) {
            mpf_class &cell_mag = mat[r][row];
            if (cell_mag < 0) {
                cell_mag = -cell_mag;
            }

            if (max_cell < cell_mag) {
                max_cell = cell_mag;
                max_row = r;
            }
        }

        // max_row has the greatest magnitude, swap into this row
        if (max_row != row) {
            std::swap(mat[row], mat[max_row]);
            std::swap(perm[row], perm[max_row]);

            ++pivot_count;
        }
    }

    // The loop works by going row-by-row and whittling down each
    // term added from multiplying the L and U portions, isolating
    // the value of the cell
    for (int row = 0; row < mat.rows(); ++row) {
        // Before the `row` column: L portion
        for (int col = 0; col < row; ++col) {
            for (int p = 0; p < col; ++p) {
                mat[row][col] -= mat[row][p] * mat[p][col];
            }

            mat[row][col] /= mat[col][col];
        }

        // After the `row` column: U portion
        for (int col = row; col < mat.rows(); ++col) {
            for (int p = 0; p < row; ++p) {
                mat[row][col] -= mat[row][p] * mat[p][col];
            }
        }
    }

    return pivot_count;
}

// Doolittle LU decomposition forward/backward substitution linear solver function
// Adapted from: https://en.wikipedia.org/wiki/LU_decomposition#C_code_examples
static liftoff::polynomial lup_linsolve(const liftoff::matrix &mat, const std::vector<int> &perm,
                                        const liftoff::matrix &b) {
    liftoff::polynomial sol{mat.rows()};

    for (int row = 0; row < mat.rows(); ++row) {
        // sol = P*b
        sol[row] = b[perm[row]][0];

        // Forward substitution
        // col < row so we are taking the L portion
        // sol = P*b = L*y, whittle down sol of terms
        // from the L portion until we can isolate y
        for (int col = 0; col < row; col++) {
            sol[row] -= mat[row][col] * sol[col];
        }
    }

    for (int row = mat.rows() - 1; row >= 0; --row) {
        // Backwards substitution
        // row < col, so we are taking the U portion
        // sol = y = U*x, whittle down sol until we can
        // isolate x -> return
        for (int col = row + 1; col < mat.columns(); col++) {
            sol[row] -= mat[row][col] * sol[col];
        }

        // Since Doolittle gives an identity L portion,
        // divide by the diagonal for the U portion in order
        // to determine the solution
        sol[row] /= mat[row][row];
    }

    return sol;
}

static liftoff::polynomial linsolve(liftoff::matrix &m, const liftoff::matrix &b) {
    std::vector<int> perm;
    lup(m, perm);

    return lup_linsolve(m, perm, b);
}

// https://sameradeeb-new.srv.ualberta.ca/introduction-to-numerical-analysis/polynomial-interpolation/
liftoff::polynomial liftoff::lip(const std::vector<std::pair<double, double>> &forced_points) {
    unsigned int order = forced_points.size() - 1;

    liftoff::matrix m{order + 1};
    for (int r = 0; r <= order; ++r) {
        double x = forced_points[r].first;
        for (int c = 0; c <= order; ++c) {
            mpf_class cell{x, REQ_PRECISION};
            mpf_pow_ui(cell.get_mpf_t(), cell.get_mpf_t(), c);

            m[r][c] = cell;
        }
    }

    liftoff::matrix b{order + 1, 1};
    for (int i = 0; i <= order; ++i) {
        b[i][0] = forced_points[i].second;
    }

    return linsolve(m, b);
}

// Adapted from: https://stackoverflow.com/questions/15191088/how-to-do-a-polynomial-fit-with-fixed-points
liftoff::polynomial liftoff::fit(unsigned int order,
                                 const std::vector<double> &x,
                                 const std::vector<double> &y,
                                 const std::vector<std::pair<double, double>> &forced_points) {
    if (x.size() != y.size()) {
        throw std::invalid_argument("x/y are not the same size");
    }

    liftoff::matrix x_n{2 * order + 1, x.size()};
    for (int r = 0; r < x_n.rows(); ++r) {
        for (int c = 0; c < x.size(); ++c) {
            mpf_class cell{x[c], REQ_PRECISION};
            mpf_pow_ui(cell.get_mpf_t(), cell.get_mpf_t(), r);

            x_n[r][c] = cell;
        }
    }

    liftoff::matrix yx_n{2 * order + 1, 1};
    for (int r = 0; r < order + 1; ++r) {
        mpf_class sum{0, REQ_PRECISION};
        for (int c = 0; c < x_n.columns(); ++c) {
            mpf_class term{x_n[r][c], REQ_PRECISION};
            term *= y[c];

            sum += term;
        }

        yx_n[r][0] = sum;
    }

    liftoff::matrix x_n_sum{x_n.rows(), 1};
    for (int r = 0; r < x_n.rows(); ++r) {
        mpf_class sum{0, REQ_PRECISION};
        for (int c = 0; c < x_n.columns(); ++c) {
            sum += x_n[r][c];
        }

        x_n_sum[r][0] = sum;
    }

    liftoff::matrix xf_n{order + 1};
    for (int r = 0; r < order + 1; ++r) {
        for (int c = 0; c < forced_points.size(); ++c) {
            mpf_class cell{forced_points[c].first, REQ_PRECISION};
            mpf_pow_ui(cell.get_mpf_t(), cell.get_mpf_t(), r);

            xf_n[r][c] = cell;
        }
    }

    unsigned int lsq_bound = order + 1;
    unsigned int m_dim = lsq_bound + forced_points.size();
    liftoff::matrix m{m_dim};
    liftoff::matrix b{m_dim, 1};

    for (int r = 0; r < m_dim; ++r) {
        for (int c = 0; c < m_dim; ++c) {
            if (r < lsq_bound && c < lsq_bound) {
                m[r][c] = x_n_sum[r + c][0];
            } else if (r < lsq_bound && c >= lsq_bound) {
                mpf_class k2{2, REQ_PRECISION};
                m[r][c] = xf_n[r][c - lsq_bound] / k2;
            } else if (r >= lsq_bound && c < lsq_bound) {
                m[r][c] = xf_n[c][r - lsq_bound];
            } else {
                m[r][c] = 0;
            }
        }

        if (r < lsq_bound) {
            b[r][0] = yx_n[r][0];
        } else {
            b[r][0] = forced_points[r - lsq_bound].second;
        }
    }

    liftoff::polynomial sol = linsolve(m, b);

    liftoff::polynomial poly;
    for (int i = 0; i < order + 1; ++i) {
        poly.add_term(sol[i]);
    }

    return poly;
}
