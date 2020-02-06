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

#include <casadi.hpp>
#include "MX_Xd_static.h"

class MX_Xd_dynamic : public casadi::MX{
public:
    MX_Xd_dynamic(
            unsigned int nrows = 1,
            unsigned int ncols = 1) : casadi::MX(nrows, ncols){
    }

    MX_Xd_dynamic(const casadi::MX& m) :
        casadi::MX(m){
    }

    void conservativeResize (unsigned int nrows, unsigned int ncols = 1) {
        MX_Xd_dynamic result = casadi::MX::zeros(nrows, ncols);

        unsigned int arows = std::min (nrows, rows());
        unsigned int acols = std::min (ncols, cols());

        for (unsigned int i = 0; i < arows; i++) {
            for (unsigned int j = 0; j < acols; j++) {
                result(i,j) = (*this)(i,j);
            }
        }

        *this = result;
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
    MX_Xd_dynamic operator()(unsigned int i, unsigned int j=0) const {
        return this->casadi::MX::operator()(i, j);
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
        return this->casadi::MX::operator()(
            casadi::Slice(static_cast<casadi_int>(row_start), static_cast<casadi_int>(row_start+row_count)),
            casadi::Slice(static_cast<casadi_int>(col_start), static_cast<casadi_int>(col_start+col_count)));
    }

    MX_Xd_dynamic block (
            unsigned int row_start,
            unsigned int col_start,
            unsigned int row_count,
            unsigned int col_count) const
    {
        return this->casadi::MX::operator()(
            casadi::Slice(static_cast<casadi_int>(row_start), static_cast<casadi_int>(row_start+row_count)),
            casadi::Slice(static_cast<casadi_int>(col_start), static_cast<casadi_int>(col_start+col_count)));
    }

    MX_Xd_dynamic transpose() const {
        return T();
    }

    MX_Xd_dynamic inverse() const {
        return inv(*this);
    }

    MX_Xd_static<1, 1> dot(const MX_Xd_dynamic &other_vector) const {
        return casadi::MX::dot(*this, other_vector);
    }

    MX_Xd_dynamic norm() const {
        return casadi::MX::norm_1(*this);
    }

    MX_Xd_dynamic squaredNorm() const {
        return casadi::MX::norm_2(*this);
    }

//    void operator+=(
//            const MX_Xd_dynamic& m2) {
//        this->casadi::MX::operator+=(m2);
//    }

//    void operator-=(
//            const MX_Xd_dynamic& m2) {
//        this->casadi::MX::operator-=(m2);
//    }
    template <unsigned int nrows, unsigned int ncols>
    MX_Xd_dynamic operator*(const MX_Xd_static<nrows, ncols>& other){
        return casadi::MX::mtimes(*this, other);
    }

};

//template <unsigned int nrows1, unsigned int ncols1, unsigned int nrows2, unsigned int ncols2>
//MX_Xd_static<nrows1, ncols2> operator*(
//        const MX_Xd_static<nrows1, ncols1>& m1,
//        const MX_Xd_static<nrows2, ncols2>& m2){
//    return casadi::MX::mtimes(m1, m2);
//}
//inline MX_Xd_static<1, 1> operator*(
//        const casadi::SubMatrix<casadi::MX, unsigned int, unsigned int>& m1,
//        const casadi::SubMatrix<casadi::MX, unsigned int, unsigned int>& m2){
//    return casadi::MX::mtimes(m1, m2);
//}

//template <unsigned int nrows, unsigned int ncols>
//MX_Xd_dynamic operator*(
//        const MX_Xd_static<nrows, ncols>& m1,
//        const MX_Xd_dynamic& m2){
//    return casadi::MX::mtimes(m1, m2);
//}
//template <unsigned int nrows, unsigned int ncols>
//MX_Xd_dynamic operator*(
//        const MX_Xd_static<nrows, ncols>& m1,
//        const casadi::MX& m2){
//    return casadi::MX::mtimes(m1, m2);
//}
//template <unsigned int nrows, unsigned int ncols>
//MX_Xd_dynamic operator*(
//        const MX_Xd_dynamic& m1,
//        const MX_Xd_static<nrows, ncols>& m2){
//    return casadi::MX::mtimes(m1, m2);
//}
//template <unsigned int nrows, unsigned int ncols>
//MX_Xd_dynamic operator*(
//        const casadi::MX& m1,
//        const MX_Xd_static<nrows, ncols>& m2){
//    return casadi::MX::mtimes(m1, m2);
//}
//inline MX_Xd_dynamic operator*(
//        const MX_Xd_dynamic& m1,
//        const casadi::MX& m2){
//    return casadi::MX::mtimes(m1, m2);
//}
//inline MX_Xd_dynamic operator*(
//        const casadi::MX& m1,
//        const MX_Xd_dynamic& m2){
//    return casadi::MX::mtimes(m1, m2);
//}
//inline MX_Xd_dynamic operator*(
//        const casadi::MX& m1,
//        const casadi::MX& m2){
//    return casadi::MX::mtimes(m1, m2);
//}
//inline MX_Xd_dynamic operator*(
//        const MX_Xd_dynamic& m1,
//        const MX_Xd_dynamic& m2){
//    return casadi::MX::mtimes(m1, m2);
//}

//template <unsigned int nrows, unsigned int ncols>
//MX_Xd_dynamic operator/(
//        MX_Xd_static<nrows, ncols> m1,
//        const MX_Xd_dynamic& m2){
//    return m1.casadi::MX::operator/=(m2);
//}
//inline MX_Xd_dynamic operator/(
//        MX_Xd_dynamic m1,
//        const MX_Xd_dynamic& m2){
//    return m1.casadi::MX::operator/=(m2);
//}

//template <unsigned int nrows1, unsigned int ncols1nrows2, unsigned int ncols2>
//MX_Xd_static<nrows1, ncols2> operator*(
//        const MX_Xd_static<nrows1, ncols1nrows2>& m1,
//        const MX_Xd_static<ncols1nrows2, ncols2>& m2){
//    return casadi::MX::mtimes(m1, m2);
//}

//template <unsigned int nrows1, unsigned int ncols1, unsigned int nrows2, unsigned int ncols2>
//inline MX_Xd_static<nrows1, ncols2> operator*(
//        const MX_Xd_static<nrows1, ncols1>& m1,
//        const MX_Xd_static<nrows2, ncols2>& m2){
//    return casadi::MX::mtimes(m1, m2);
//}
//template <unsigned int nrows, unsigned int ncols>
//MX_Xd_static<nrows, ncols> operator*(
//        const MX_Xd_static<nrows, ncols>& m1,
//        const double& m2){
//    return casadi::MX::times(m1, m2);
//}
//inline MX_Xd_dynamic operator*(
//        const casadi::MX& m1,
//        const double& m2){
//    return casadi::MX::times(m1, m2);
//}
//inline MX_Xd_dynamic operator*(
//        const casadi::SubMatrix<casadi::MX, unsigned int, unsigned int>& m1,
//        const casadi::SubMatrix<casadi::MX, unsigned int, unsigned int>& m2){
//    return casadi::MX::times(m1, m2);
//}
//inline MX_Xd_dynamic operator*(
//        const MX_Xd_dynamic& m1,
//        const double& m2){
//    return casadi::MX::times(m1, m2);
//}

//template <unsigned int nrows, unsigned int ncols>
//MX_Xd_static<nrows, ncols> operator*(
//        const double& m1,
//        const MX_Xd_static<nrows, ncols>& m2){
//    return casadi::MX::times(m2, m1);
//}
//inline MX_Xd_dynamic operator*(
//        const double& m1,
//        const casadi::MX& m2){
//    return casadi::MX::times(m2, m1);
//}
//inline MX_Xd_dynamic operator*(
//        const casadi::SubMatrix<casadi::MX, unsigned int, unsigned int>& m1,
//        const MX_Xd_dynamic& m2){
//    return casadi::MX::times(m1, m2);
//}
//inline MX_Xd_dynamic operator*(
//        const double& m1,
//        const MX_Xd_dynamic& m2){
//    return casadi::MX::times(m1, m2);
//}

//inline MX_Xd_dynamic operator+(
//        const MX_Xd_dynamic& m1,
//        const MX_Xd_dynamic& m2) {
//    return static_cast<casadi::MX>(m1).casadi::MX::operator+=(m2);
//}
//inline MX_Xd_dynamic operator-(
//        const MX_Xd_dynamic& m1) {
//    return m1.casadi::MX::operator-();
//}
//inline MX_Xd_dynamic operator-(
//        const MX_Xd_dynamic& m1,
//        const MX_Xd_dynamic& m2) {
//    return static_cast<casadi::MX>(m1).casadi::MX::operator-=(m2);
//}
//template <unsigned int nrows, unsigned int ncols>
//MX_Xd_static<nrows, ncols> operator-(
//        const MX_Xd_static<nrows, ncols>& m1,
//        const MX_Xd_dynamic& m2) {
//    return static_cast<casadi::MX>(m1).casadi::MX::operator-=(m2);
//}
//template <unsigned int nrows, unsigned int ncols>
//MX_Xd_static<nrows, ncols> operator-(
//        const MX_Xd_dynamic& m1,
//        const MX_Xd_static<nrows, ncols>& m2
//       ) {
//    return -static_cast<casadi::MX>(m2).casadi::MX::operator-=(m1);
//}

template <unsigned int nrows, unsigned int ncols>
MX_Xd_dynamic fabs(const MX_Xd_dynamic& m){
    return casadi::MX::abs(m);
}

/* MX_XD_DYNAMICS_H */
#endif

