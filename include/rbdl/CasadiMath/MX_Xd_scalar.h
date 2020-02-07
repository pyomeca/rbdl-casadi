/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef MX_XD_SCALAR_H
#define MX_XD_SCALAR_H

#include <vector>
#include <string>
#include <memory>

#include <casadi.hpp>


class MX_Xd_scalar : public casadi::MX{
public:
    MX_Xd_scalar() : casadi::MX(1, 1){

    }

    MX_Xd_scalar(const double val) : casadi::MX(1, 1){
        (*this)(0, 0) = val;
    }

    MX_Xd_scalar(const casadi::MX& m) : casadi::MX(m){

    }

    MX_Xd_scalar(
            const MX_Xd_scalar& v0) :
        casadi::MX(1, 1)
    {
        (*this)(0) = v0(0);
    }

    unsigned int rows() const {
        return 1;
    }

    unsigned int cols() const {
        return 1;
    }

    unsigned int size() const {
        return 1;
    }

    bool operator==(const MX_Xd_scalar& other) const {
        return casadi::MX::is_equal(*this, other);
    }
    bool operator==(const double& val) const {
        return casadi::MX::is_equal(*this, MX_Xd_scalar(val));
    }
    bool operator!=(const MX_Xd_scalar& other) const {
        return !casadi::MX::is_equal(*this, other);
    }
    bool operator!=(const double& val) const {
        return !casadi::MX::is_equal(*this, MX_Xd_scalar(val));
    }

    MX_Xd_scalar operator[](unsigned int i) const{
        return (*this)(i);
    }

    void operator+=(
            const MX_Xd_scalar& other) {
        this->casadi::MX::operator+=(other);
    }
    MX_Xd_scalar operator+(
            const MX_Xd_scalar& other) const {
        MX_Xd_scalar out(*this);
        return out.casadi::MX::operator+=(other);
    }

    void operator-=(
            const MX_Xd_scalar& other) {
        this->casadi::MX::operator-=(other);
    }
    MX_Xd_scalar operator-(
            const MX_Xd_scalar& other) const {
        MX_Xd_scalar out(*this);
        return out.casadi::MX::operator-=(other);
    }

    MX_Xd_scalar operator*(const MX_Xd_scalar& other) const{
        return casadi::MX::times(*this, other);
    }
    MX_Xd_scalar operator*(
            double other) {
        return casadi::MX::times(*this, other);
    }

    void operator/=(
            const MX_Xd_scalar& other
            ){
        this->casadi::MX::operator/=(other);
    }
    MX_Xd_scalar operator/(
            const MX_Xd_scalar& other) const {
        MX_Xd_scalar out(*this);
        return out.casadi::MX::operator/=(other);
    }
};

inline bool operator==(const double& m1, const MX_Xd_scalar& m2){
    return casadi::MX::is_equal(MX_Xd_scalar(m1), m2);
}
inline bool operator!=(const double& m1, const MX_Xd_scalar& m2){
    return !casadi::MX::is_equal(MX_Xd_scalar(m1), m2);
}

inline MX_Xd_scalar operator*(
        double other,
        const MX_Xd_scalar& me
        ) {
    return casadi::MX::times(me, other);
}

inline MX_Xd_scalar operator+(
        double other,
        const MX_Xd_scalar& me) {
    MX_Xd_scalar out(me);
    return out.casadi::MX::operator+=(other);
}

inline MX_Xd_scalar operator-(
        const MX_Xd_scalar& other) {
    return other.casadi::MX::operator-();
}
inline MX_Xd_scalar operator-(
        double other,
        const MX_Xd_scalar& me) {
    MX_Xd_scalar out(me);
    return out.casadi::MX::operator-=(other);
}

inline MX_Xd_scalar fabs(const MX_Xd_scalar& m){
    return casadi::MX::abs(m);
}

inline MX_Xd_scalar sqrt(const MX_Xd_scalar& m){
    return casadi::MX::sqrt(m);
}

inline bool isnan(const MX_Xd_scalar&){
    return false;
}




class MX_Xd_SubMatrix : public casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>{
public:
    MX_Xd_SubMatrix(casadi::MX& mat, const casadi::Slice& i, const casadi::Slice& j) :
        casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>(mat, i, j)
    {

    }
    MX_Xd_SubMatrix(MX_Xd_SubMatrix& mat, const casadi::Slice& i, const casadi::Slice& j) :
        casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>(mat, i, j)
    {

    }
    MX_Xd_SubMatrix(const casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>& me) :
        casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>(me)
    {

    }


    void operator=(const casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>& submat){
        this->casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>::operator=(submat);
    }
    void operator=(const casadi::MX& mat){
        this->casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>::operator =(mat);
    }
    void operator=(const MX_Xd_scalar& mat){
        this->casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>::operator =(mat);
    }
    void operator=(double mat){
        this->casadi::SubMatrix<casadi::MX, casadi::Slice, casadi::Slice>::operator =(mat);
    }
};

/* MX_XD_SCALAR_H */
#endif

