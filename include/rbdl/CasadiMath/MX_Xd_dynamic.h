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
#include "MX_Xd_scalar.h"

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

    MX_Xd_SubMatrix operator[](unsigned int i) {
        return (*this)(i, 0);
    }
    MX_Xd_scalar operator[](unsigned int i) const {
        return (*this)(i, 0);
    }
    MX_Xd_SubMatrix operator()(unsigned int i, unsigned int j=0) {
        return this->casadi::MX::operator()(
                    casadi::Slice(static_cast<casadi_int>(i), static_cast<casadi_int>(i+1)),
                    casadi::Slice(static_cast<casadi_int>(j), static_cast<casadi_int>(j+1)));
    }
    MX_Xd_scalar operator()(unsigned int i, unsigned int j=0) const {
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
    MX_Xd_SubMatrix block(
            unsigned int row_start,
            unsigned int col_start)
    {
        return this->casadi::MX::operator()(
            casadi::Slice(static_cast<casadi_int>(row_start), static_cast<casadi_int>(row_start+row_count)),
            casadi::Slice(static_cast<casadi_int>(col_start), static_cast<casadi_int>(col_start+col_count)));
    }
    template <unsigned int row_count, unsigned int col_count>
    MX_Xd_dynamic block(
            unsigned int row_start,
            unsigned int col_start) const
    {
        return this->casadi::MX::operator()(
            casadi::Slice(static_cast<casadi_int>(row_start), static_cast<casadi_int>(row_start+row_count)),
            casadi::Slice(static_cast<casadi_int>(col_start), static_cast<casadi_int>(col_start+col_count)));
    }

    MX_Xd_SubMatrix block(
            unsigned int row_start,
            unsigned int col_start,
            unsigned int row_count,
            unsigned int col_count)
    {
        return this->casadi::MX::operator()(
            casadi::Slice(static_cast<casadi_int>(row_start), static_cast<casadi_int>(row_start+row_count)),
            casadi::Slice(static_cast<casadi_int>(col_start), static_cast<casadi_int>(col_start+col_count)));
    }
    MX_Xd_dynamic block(
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

    MX_Xd_scalar dot(const MX_Xd_dynamic &other_vector) const {
        return casadi::MX::dot(*this, other_vector);
    }

    MX_Xd_dynamic norm() const {
        return casadi::MX::norm_1(*this);
    }

    MX_Xd_dynamic squaredNorm() const {
        return casadi::MX::norm_2(*this);
    }



    void operator+=(
            const MX_Xd_dynamic& other) {
        this->casadi::MX::operator+=(other);
    }
    MX_Xd_dynamic operator+(
            const MX_Xd_dynamic& other) const {
        MX_Xd_dynamic out(*this);
        return out.casadi::MX::operator+=(other);
    }
    void operator-=(
            const MX_Xd_dynamic& other) {
        this->casadi::MX::operator-=(other);
    }
    MX_Xd_dynamic operator-(
            const MX_Xd_dynamic& other) const {
        MX_Xd_dynamic out(*this);
        return out.casadi::MX::operator-=(other);
    }
    void operator*=(
            const MX_Xd_dynamic& m2) {
        *this = casadi::MX::mtimes(*this, m2);
    }
    MX_Xd_dynamic operator*(const MX_Xd_dynamic& other) const {
        return casadi::MX::mtimes(*this, other);
    }
    MX_Xd_dynamic operator*(const MX_Xd_scalar& other) const {
        return casadi::MX::mtimes(*this, other);
    }
    MX_Xd_dynamic operator*(const MX_Xd_SubMatrix& other) const {
        return casadi::MX::mtimes(*this, other);
    }
    MX_Xd_dynamic operator*(const double& other) const {
        return casadi::MX::mtimes(*this, other);
    }

    void operator/=(const MX_Xd_scalar &scalar) {
        for (unsigned int i = 0; i < rows() * cols(); i++)
            this->casadi::MX::operator/=(scalar(0, 0));
    }
    MX_Xd_dynamic operator/(const MX_Xd_scalar &scalar) const {
        MX_Xd_dynamic result (*this);
        for (unsigned int i = 0; i < rows() * cols(); i++)
            result[i] /= scalar;
        return result;
    }
    MX_Xd_dynamic operator/(
            const MX_Xd_SubMatrix& scalar) const {
        MX_Xd_dynamic result (*this);
        for (unsigned int i = 0; i < rows() * cols(); i++)
            result[i] /= scalar;
        return result;
    }
};

inline MX_Xd_dynamic operator*(
        const MX_Xd_SubMatrix& me,
        const MX_Xd_scalar& other){
    return MX_Xd_SubMatrix::times(me, other);
}
inline MX_Xd_dynamic operator*(
        const MX_Xd_SubMatrix& me,
        const MX_Xd_SubMatrix& other){
    return MX_Xd_SubMatrix::mtimes(me, other);
}

inline MX_Xd_SubMatrix operator/(
        const MX_Xd_SubMatrix& me,
        const MX_Xd_SubMatrix& scalar) {
    MX_Xd_SubMatrix result (me);
    result.MX_Xd_SubMatrix::operator/=(scalar);
    return result;
}

#include "MX_Xd_static.h"

inline MX_Xd_dynamic operator*(
        const MX_Xd_scalar& m1,
        const MX_Xd_dynamic m2){
    return casadi::MX::mtimes(m1, m2);
}
inline MX_Xd_dynamic operator*(
        const double& m1,
        const MX_Xd_dynamic m2){
    return casadi::MX::mtimes(m1, m2);
}
inline MX_Xd_dynamic operator*(
        const double& m1,
        const MX_Xd_SubMatrix m2){
    return casadi::MX::mtimes(m1, m2);
}
template <unsigned int nrows, unsigned int ncols>
MX_Xd_dynamic operator*(
        const MX_Xd_static<nrows, ncols>& m1,
        const MX_Xd_dynamic m2){
    return casadi::MX::mtimes(m1, m2);
}
template <unsigned int nrows, unsigned int ncols>
MX_Xd_dynamic operator*(
        const MX_Xd_dynamic m1,
        const MX_Xd_static<nrows, ncols>& m2){
    return casadi::MX::mtimes(m1, m2);
}
template <unsigned int nrows, unsigned int ncols>
MX_Xd_dynamic operator*(
        const MX_Xd_SubMatrix& m1,
        const MX_Xd_static<nrows, ncols>& m2){
    return casadi::MX::mtimes(m1, m2);
}
template <unsigned int nrows, unsigned int ncols>
MX_Xd_dynamic operator*(
        const MX_Xd_static<nrows, ncols>& m1,
        const MX_Xd_SubMatrix& m2){
    return casadi::MX::mtimes(m1, m2);
}

inline MX_Xd_dynamic operator/(double scalar, MX_Xd_SubMatrix mat){
    for (unsigned int i=0; i<mat.rows(); ++i){
        for (unsigned int j=0; j<mat.columns(); ++j){
            mat(i, j) = scalar / mat(i, j);
        }
    }
    return mat;
}

template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<nrows, ncols> operator+(
        const MX_Xd_dynamic& me,
        const MX_Xd_static<nrows, ncols>& other
        ) {
    MX_Xd_static<nrows, ncols> out(me);
    return out.casadi::MX::operator+=(other);
}
inline MX_Xd_dynamic operator-(
        const MX_Xd_dynamic& me) {
    return me.casadi::MX::operator-();
}
inline MX_Xd_dynamic operator-(
        const MX_Xd_SubMatrix& me) {
    return me.casadi::MX::operator-();
}
inline MX_Xd_dynamic operator-(
        MX_Xd_SubMatrix me,
        const MX_Xd_scalar& scalar){
    return me.casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>::operator-=(scalar);
}
inline MX_Xd_scalar operator+(
        const MX_Xd_dynamic& me,
        const MX_Xd_scalar& other
        ) {
    MX_Xd_scalar out(me);
    return out.casadi::MX::operator+=(other);
}
inline MX_Xd_scalar operator+(
        const MX_Xd_scalar& me,
        const MX_Xd_dynamic& other
        ) {
    MX_Xd_scalar out(me);
    return out.casadi::MX::operator+=(other);
}
inline MX_Xd_dynamic operator+(
        const MX_Xd_SubMatrix& me,
        const MX_Xd_dynamic& other
        ) {
    MX_Xd_dynamic out(other);
    return out.casadi::MX::operator+=(me);
}
inline MX_Xd_dynamic operator+(
        const MX_Xd_dynamic& me,
        const MX_Xd_SubMatrix& other
        ) {
    MX_Xd_dynamic out(me);
    return out.casadi::MX::operator+=(other);
}

inline MX_Xd_dynamic operator+(
        MX_Xd_SubMatrix me,
        const MX_Xd_SubMatrix& other){
    return me.casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>::operator+=(other);
}

template <unsigned int nrows, unsigned int ncols>
inline MX_Xd_static<nrows, ncols> operator-(
        const MX_Xd_dynamic& me,
        const MX_Xd_static<nrows, ncols>& other
        ) {
    MX_Xd_static<nrows, ncols> out(me);
    return out.casadi::MX::operator-=(other);
}
inline MX_Xd_scalar operator-(
        const MX_Xd_dynamic& me,
        const MX_Xd_scalar& other
        ) {
    MX_Xd_scalar out(me);
    return out.casadi::MX::operator-=(other);
}
inline MX_Xd_scalar operator-(
        const MX_Xd_scalar& me,
        const MX_Xd_dynamic& other
        ) {
    MX_Xd_scalar out(me);
    return out.casadi::MX::operator-=(other);
}
inline MX_Xd_dynamic operator-(
        const MX_Xd_SubMatrix& me,
        const MX_Xd_dynamic& other
        ) {
    MX_Xd_dynamic out(-other);
    return out.casadi::MX::operator+=(me);
}
inline MX_Xd_dynamic operator-(
        const MX_Xd_dynamic& me,
        const MX_Xd_SubMatrix& other
        ) {
    MX_Xd_dynamic out(me);
    return out.casadi::MX::operator-=(other);
}
inline MX_Xd_dynamic operator-(
        const MX_Xd_SubMatrix& me,
        const MX_Xd_SubMatrix& other
        ) {
    MX_Xd_dynamic out(me);
    return out.casadi::MX::operator-=(other);
}

template <unsigned int nrows, unsigned int ncols>
MX_Xd_dynamic fabs(const MX_Xd_dynamic& m){
    return casadi::MX::abs(m);
}

/* MX_XD_DYNAMICS_H */
#endif

