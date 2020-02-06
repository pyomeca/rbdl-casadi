/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2018 Martin Felis <martin@fysx.org>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef MX_XD_STATIC_H
#define MX_XD_STATIC_H

#include <vector>
#include <string>
#include <memory>

#include <casadi.hpp>

template <unsigned int nrows, unsigned int ncols>
class MX_Xd_static : public casadi::MX{
public:
    MX_Xd_static() : casadi::MX(nrows, ncols){

    }

    MX_Xd_static(const double val) : casadi::MX(1, 1){
        (*this)(0, 0) = val;
    }

    MX_Xd_static(const casadi::MX& m) : casadi::MX(m){

    }

    MX_Xd_static(
            const MX_Xd_static<1,1>& v0,
            const MX_Xd_static<1,1>& v1,
            const MX_Xd_static<1,1>& v2) :
        casadi::MX(3, 1)
    {
        (*this)(0) = v0(0);
        (*this)(1) = v1(0);
        (*this)(2) = v2(0);
    }
    MX_Xd_static(const MX_Xd_static<1,1>& v0,
                 const MX_Xd_static<1,1>& v1,
                 const MX_Xd_static<1,1>& v2,
                 const MX_Xd_static<1,1>& v3) :
        casadi::MX(4, 1)
    {
        (*this)(0) = v0(0);
        (*this)(1) = v1(0);
        (*this)(2) = v2(0);
        (*this)(3) = v3(0);
    }
    MX_Xd_static(const MX_Xd_static<1,1>& v0,
                 const MX_Xd_static<1,1>& v1,
                 const MX_Xd_static<1,1>& v2,
                 const MX_Xd_static<1,1>& v3,
                 const MX_Xd_static<1,1>& v4,
                 const MX_Xd_static<1,1>& v5) :
        casadi::MX(6, 1)
    {
        (*this)(0) = v0(0);
        (*this)(1) = v1(0);
        (*this)(2) = v2(0);
        (*this)(3) = v3(0);
        (*this)(4) = v4(0);
        (*this)(5) = v5(0);
    }
    MX_Xd_static(const MX_Xd_static<1,1>& v0,
                 const MX_Xd_static<1,1>& v1,
                 const MX_Xd_static<1,1>& v2,
                 const MX_Xd_static<1,1>& v3,
                 const MX_Xd_static<1,1>& v4,
                 const MX_Xd_static<1,1>& v5,
                 const MX_Xd_static<1,1>& v6,
                 const MX_Xd_static<1,1>& v7) :
        casadi::MX(8, 1)
    {
        (*this)(0) = v0(0);
        (*this)(1) = v1(0);
        (*this)(2) = v2(0);
        (*this)(3) = v3(0);
        (*this)(4) = v4(0);
        (*this)(5) = v5(0);
        (*this)(6) = v6(0);
        (*this)(7) = v7(0);
    }
    MX_Xd_static(const MX_Xd_static<1,1>& v00, const MX_Xd_static<1,1>& v01, const MX_Xd_static<1,1>& v02,
                 const MX_Xd_static<1,1>& v10, const MX_Xd_static<1,1>& v11, const MX_Xd_static<1,1>& v12,
                 const MX_Xd_static<1,1>& v20, const MX_Xd_static<1,1>& v21, const MX_Xd_static<1,1>& v22) :
        casadi::MX(3, 3)
    {
        (*this)(0,0) = v00(0, 0);
        (*this)(0,1) = v01(0, 0);
        (*this)(0,2) = v02(0, 0);
        (*this)(1,0) = v10(0, 0);
        (*this)(1,1) = v11(0, 0);
        (*this)(1,2) = v12(0, 0);
        (*this)(2,0) = v20(0, 0);
        (*this)(2,1) = v21(0, 0);
        (*this)(2,2) = v22(0, 0);
    }
    MX_Xd_static(const MX_Xd_static<1,1>& v00, const MX_Xd_static<1,1>& v01, const MX_Xd_static<1,1>& v02, const MX_Xd_static<1,1>& v03, const MX_Xd_static<1,1>& v04, const MX_Xd_static<1,1>& v05,
                 const MX_Xd_static<1,1>& v10, const MX_Xd_static<1,1>& v11, const MX_Xd_static<1,1>& v12, const MX_Xd_static<1,1>& v13, const MX_Xd_static<1,1>& v14, const MX_Xd_static<1,1>& v15,
                 const MX_Xd_static<1,1>& v20, const MX_Xd_static<1,1>& v21, const MX_Xd_static<1,1>& v22, const MX_Xd_static<1,1>& v23, const MX_Xd_static<1,1>& v24, const MX_Xd_static<1,1>& v25,
                 const MX_Xd_static<1,1>& v30, const MX_Xd_static<1,1>& v31, const MX_Xd_static<1,1>& v32, const MX_Xd_static<1,1>& v33, const MX_Xd_static<1,1>& v34, const MX_Xd_static<1,1>& v35,
                 const MX_Xd_static<1,1>& v40, const MX_Xd_static<1,1>& v41, const MX_Xd_static<1,1>& v42, const MX_Xd_static<1,1>& v43, const MX_Xd_static<1,1>& v44, const MX_Xd_static<1,1>& v45,
                 const MX_Xd_static<1,1>& v50, const MX_Xd_static<1,1>& v51, const MX_Xd_static<1,1>& v52, const MX_Xd_static<1,1>& v53, const MX_Xd_static<1,1>& v54, const MX_Xd_static<1,1>& v55) :
        casadi::MX(6, 6)
    {
        (*this)(0,0) = v00(0, 0);
        (*this)(0,1) = v01(0, 0);
        (*this)(0,2) = v02(0, 0);
        (*this)(0,3) = v03(0, 0);
        (*this)(0,4) = v04(0, 0);
        (*this)(0,5) = v05(0, 0);

        (*this)(1,0) = v10(0, 0);
        (*this)(1,1) = v11(0, 0);
        (*this)(1,2) = v12(0, 0);
        (*this)(1,3) = v13(0, 0);
        (*this)(1,4) = v14(0, 0);
        (*this)(1,5) = v15(0, 0);

        (*this)(2,0) = v20(0, 0);
        (*this)(2,1) = v21(0, 0);
        (*this)(2,2) = v22(0, 0);
        (*this)(2,3) = v23(0, 0);
        (*this)(2,4) = v24(0, 0);
        (*this)(2,5) = v25(0, 0);

        (*this)(3,0) = v30(0, 0);
        (*this)(3,1) = v31(0, 0);
        (*this)(3,2) = v32(0, 0);
        (*this)(3,3) = v33(0, 0);
        (*this)(3,4) = v34(0, 0);
        (*this)(3,5) = v35(0, 0);

        (*this)(4,0) = v40(0, 0);
        (*this)(4,1) = v41(0, 0);
        (*this)(4,2) = v42(0, 0);
        (*this)(4,3) = v43(0, 0);
        (*this)(4,4) = v44(0, 0);
        (*this)(4,5) = v45(0, 0);

        (*this)(5,0) = v50(0, 0);
        (*this)(5,1) = v51(0, 0);
        (*this)(5,2) = v52(0, 0);
        (*this)(5,3) = v53(0, 0);
        (*this)(5,4) = v54(0, 0);
        (*this)(5,5) = v55(0, 0);

    }

    ///
    /// \brief set For 3d Vector
    /// \param v0 X
    /// \param v1 Y
    /// \param v2 Z
    ///
    void set(
            const MX_Xd_static<1,1>& v0,
            const MX_Xd_static<1,1>& v1,
            const MX_Xd_static<1,1>& v2){
        (*this)(0) = v0;
        (*this)(1) = v1;
        (*this)(2) = v2;
    }
    ///
    /// \brief set For Quaternion
    /// \param v0 X
    /// \param v1 Y
    /// \param v2 Z
    /// \param v3 W
    ///
    void set(
            const MX_Xd_static<1,1>& v0,
            const MX_Xd_static<1,1>& v1,
            const MX_Xd_static<1,1>& v2,
            const MX_Xd_static<1,1>& v3){
        (*this)(0) = v0;
        (*this)(1) = v1;
        (*this)(2) = v2;
        (*this)(3) = v3;
    }
    ///
    /// \brief set For SpatialVector
    /// \param v0
    /// \param v1
    /// \param v2
    /// \param v3
    /// \param v4
    /// \param v5
    ///
    void set(
            const MX_Xd_static<1,1>& v0,
            const MX_Xd_static<1,1>& v1,
            const MX_Xd_static<1,1>& v2,
            const MX_Xd_static<1,1>& v3,
            const MX_Xd_static<1,1>& v4,
            const MX_Xd_static<1,1>& v5){
        (*this)(0) = v0;
        (*this)(1) = v1;
        (*this)(2) = v2;
        (*this)(3) = v3;
        (*this)(4) = v4;
        (*this)(5) = v5;
    }

    static MX_Xd_static Identity(){
        MX_Xd_static<nrows, ncols> out;
        for (unsigned int i=0; i<nrows; ++i){
            for (unsigned int j=0; j<ncols; ++j){
                if (i==j){
                    out(i, j) = 1;
                } else {
                    out(i, j) = 0;
                }
            }
        }
        return out;
    }

    static MX_Xd_static Zero(){
        return MX_Xd_static<nrows, ncols>::zeros(nrows, ncols);
    }

    static MX_Xd_static One(){
        return MX_Xd_static<nrows, ncols>::ones(nrows, ncols);
    }

    void setZero(){
        *this = casadi::MX::zeros(this->rows(), this->cols());
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
    MX_Xd_static block (
            unsigned int row_start,
            unsigned int col_start) const
    {
        return (*this)(
            casadi::Slice(static_cast<casadi_int>(row_start), static_cast<casadi_int>(row_start+row_count)),
            casadi::Slice(static_cast<casadi_int>(col_start), static_cast<casadi_int>(col_start+col_count)));
    }

    MX_Xd_static block (
            unsigned int row_start,
            unsigned int col_start,
            unsigned int row_count,
            unsigned int col_count) const
    {
        return (*this)(
            casadi::Slice(static_cast<casadi_int>(row_start), static_cast<casadi_int>(row_start+row_count)),
            casadi::Slice(static_cast<casadi_int>(col_start), static_cast<casadi_int>(col_start+col_count)));
    }

    MX_Xd_static<1,1> operator[](unsigned int i) const{
        return (*this)(i);
    }


    MX_Xd_static<1, 1> dot(const MX_Xd_static<3, 1> &other_vector) const {
        return casadi::MX::dot(*this, other_vector);
    }

    MX_Xd_static<3, 1> cross(const MX_Xd_static<3, 1> &other_vector) const {
            MX_Xd_static<3, 1> result;
            result[0] = (*this)[1] * other_vector[2] - (*this)[2] * other_vector[1];
            result[1] = (*this)[2] * other_vector[0] - (*this)[0] * other_vector[2];
            result[2] = (*this)[0] * other_vector[1] - (*this)[1] * other_vector[0];

            return result;
    }

    MX_Xd_static<ncols, nrows> transpose() const {
        return T();
    }

    MX_Xd_static<1, 1> norm() const{
        return casadi::MX::norm_1(*this);
    }

    MX_Xd_static<1, 1> squaredNorm() const{
        return casadi::MX::norm_2(*this);
    }

    template <unsigned int nrows2, unsigned int ncols2>
    MX_Xd_static<nrows, ncols> operator+=(
            const MX_Xd_static<nrows2, ncols2>& m2) {
        return this->casadi::MX::operator+=(m2);
    }
    template <unsigned int nrows2, unsigned int ncols2>
    MX_Xd_static<nrows, ncols> operator-=(
            const MX_Xd_static<nrows2, ncols2>& m2) {
        return this->casadi::MX::operator-=(m2);
    }
    template <unsigned int nrows2, unsigned int ncols2>
    MX_Xd_static<nrows, ncols> operator*=(
            const MX_Xd_static<nrows2, ncols2>& m2) {
        *this = casadi::MX::mtimes(*this, m2);
        return *this;
    }
};

template <unsigned int nrows, unsigned int ncols>
bool operator==(const MX_Xd_static<nrows, ncols>& m1, const MX_Xd_static<nrows, ncols>& m2){
    return casadi::MX::is_equal(m1, m2);
}

template <unsigned int nrows, unsigned int ncols>
bool operator!=(const MX_Xd_static<nrows, ncols>& m1, const MX_Xd_static<nrows, ncols>& m2){
    return !casadi::MX::is_equal(m1, m2);
}

template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<nrows, ncols> operator+(
        MX_Xd_static<nrows, ncols> m1,
        const MX_Xd_static<nrows, ncols>& m2) {
    return m1.casadi::MX::operator+=(m2);
}

template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<nrows, ncols> operator-(
        MX_Xd_static<nrows, ncols> m1) {
    return m1.casadi::MX::operator-();
}

template <unsigned int nrows1, unsigned int ncols1nrows2, unsigned int ncols2>
MX_Xd_static<nrows1, ncols2> operator*(
        const MX_Xd_static<nrows1, ncols1nrows2>& m1,
        const MX_Xd_static<ncols1nrows2, ncols2>& m2){
    return casadi::MX::mtimes(m1, m2);
}

template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<nrows, ncols> operator*(
        const MX_Xd_static<nrows, ncols>& m1,
        const double& m2){
    return casadi::MX::times(m1, m2);
}
template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<nrows, ncols> operator*(
        const double& m1,
        const MX_Xd_static<nrows, ncols>& m2){
    return casadi::MX::times(m2, m1);
}

template <unsigned int nrows, unsigned int ncols>
MX_Xd_static<1, 1> fabs(const MX_Xd_static<nrows, ncols>& m){
    return casadi::MX::abs(m);
}

template <unsigned int nrows, unsigned int ncols>
bool isnan(const MX_Xd_static<nrows, ncols>&){
    return false;
}

/* MX_XD_STATIC_H */
#endif

