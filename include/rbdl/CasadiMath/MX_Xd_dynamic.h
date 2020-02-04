/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef MX_XD_DYNAMICS_H
#define MX_XD_DYNAMICS_H

#include <vector>
#include <string>
#include <memory>

#include <rbdl/rbdl_config.h>
#include <casadi.hpp>

class MX_Xd_dynamic : public casadi::MX{
public:
    MX_Xd_dynamic(
            unsigned int nrows = 1,
            unsigned int ncols = 1) : casadi::MX(nrows, ncols){
    }

    MX_Xd_dynamic(const casadi::MX& m) :
        casadi::MX(m){
    }

    static MX_Xd_dynamic Zero(unsigned int nrows, unsigned int ncols = 1){
        return casadi::MX::zeros(nrows, ncols);
    }

    void setZero(){
        *this = casadi::MX::zeros(this->rows(), this->cols());
    }

    static MX_Xd_dynamic Identity(unsigned int size){
        return casadi::MX::eye(size);
    }

    MX_Xd_dynamic operator[](unsigned int i) const {
        return (*this)(i, 0);
    }

    MX_Xd_dynamic operator*(const MX_Xd_dynamic& m2){
        return casadi::MX::mtimes(*this, m2);
    }

    unsigned int rows() const {
        return static_cast<unsigned int>(this->casadi::MX::rows());
    }

    unsigned int cols() const {
        return static_cast<unsigned int>(this->casadi::MX::columns());
    }

    unsigned int size() const {
        return rows() * cols();
    }

    template <unsigned int row_count, unsigned int col_count>
    MX_Xd_dynamic block (
            unsigned int row_start,
            unsigned int col_start) const
    {
        return (*this)(
            casadi::Slice(static_cast<casadi_int>(row_start), static_cast<casadi_int>(row_start+row_count)),
            casadi::Slice(static_cast<casadi_int>(col_start), static_cast<casadi_int>(col_start+col_count)));
    }

    MX_Xd_dynamic block (
            unsigned int row_start,
            unsigned int col_start,
            unsigned int row_count,
            unsigned int col_count) const
    {
        return (*this)(
            casadi::Slice(static_cast<casadi_int>(row_start), static_cast<casadi_int>(row_start+row_count)),
            casadi::Slice(static_cast<casadi_int>(col_start), static_cast<casadi_int>(col_start+col_count)));
    }

    MX_Xd_dynamic transpose() const {
        return T();
    }

    MX_Xd_dynamic norm() const {
        return casadi::MX::norm_1(*this);
    }

    MX_Xd_dynamic squaredNorm() const {
        return casadi::MX::norm_2(*this);
    }
};


template <unsigned int nrows, unsigned int ncols>
MX_Xd_dynamic fabs(const MX_Xd_dynamic& m){
    return casadi::MX::abs(m);
}

/* MX_XD_DYNAMICS_H */
#endif

