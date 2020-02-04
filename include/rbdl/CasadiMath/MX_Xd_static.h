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

//#include <rbdl/rbdl_config.h>
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
            casadi::MX v0,
            casadi::MX v1,
            casadi::MX v2) :
        casadi::MX(3, 1)
    {
        (*this)(0) = v0(0);
        (*this)(1) = v1(0);
        (*this)(2) = v2(0);
    }
    MX_Xd_static(casadi::MX v0,
                 casadi::MX v1,
                 casadi::MX v2,
                 casadi::MX v3,
                 casadi::MX v4,
                 casadi::MX v5) :
        casadi::MX(6, 1)
    {
        (*this)(0) = v0(0);
        (*this)(1) = v1(0);
        (*this)(2) = v2(0);
        (*this)(3) = v3(0);
        (*this)(4) = v4(0);
        (*this)(5) = v5(0);
    }
    MX_Xd_static(casadi::MX v00, casadi::MX v01, casadi::MX v02,
                 casadi::MX v10, casadi::MX v11, casadi::MX v12,
                 casadi::MX v20, casadi::MX v21, casadi::MX v22) :
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
        MX_Xd_static<nrows, ncols> out;
        for (unsigned int i=0; i<nrows; ++i){
            for (unsigned int j=0; j<ncols; ++j){
                out(i, j) = 0;
            }
        }
        return out;
    }

    unsigned int rows() const {
            return nrows;
    }

    unsigned int cols() const {
            return ncols;
    }

    unsigned int size() const {
            return nrows * ncols;
    }

    template <unsigned int row_count, unsigned int col_count>
    MX_Xd_static block (
            unsigned int row_start,
            unsigned int col_start)
    {
        return (*this)(
            casadi::Slice(static_cast<casadi_int>(row_start), static_cast<casadi_int>(row_start+row_count)),
            casadi::Slice(static_cast<casadi_int>(col_start), static_cast<casadi_int>(col_start+col_count)));
    }

    MX_Xd_static block (
            unsigned int row_start,
            unsigned int col_start,
            unsigned int row_count,
            unsigned int col_count)
    {
        return (*this)(
            casadi::Slice(static_cast<casadi_int>(row_start), static_cast<casadi_int>(row_start+row_count)),
            casadi::Slice(static_cast<casadi_int>(col_start), static_cast<casadi_int>(col_start+col_count)));
    }

    MX_Xd_static<3, 1> cross(const MX_Xd_static<3, 1> &other_vector) const {
            MX_Xd_static<3, 1> result;
            result[0] = (*this)[1] * other_vector[2] - (*this)[2] * other_vector[1];
            result[1] = (*this)[2] * other_vector[0] - (*this)[0] * other_vector[2];
            result[2] = (*this)[0] * other_vector[1] - (*this)[1] * other_vector[0];

            return result;
    }

    MX_Xd_static transpose() const {
        return T();
    }

    casadi::MX operator[](unsigned int i) const{
        return (*this)(i);
    }

//    bool operator==(double val){
//        if (rows() != 1 || cols() != 1){
//            return false;
//        }
//        if ((*this)(0, 0) != val){
//            return false;
//        }
//        return true;
//    }

    bool operator==(const MX_Xd_static& m2){
        if (rows() != m2.rows() || cols() != m2.cols()){
            return false;
        }
        if (this->operator==(static_cast<casadi::MX>(m2))){
            std::cout << "yo" << std::endl;
            return true;
        } else {
            std::cout << "coucou" << std::endl;
            return false;
        }
//        for (unsigned int i=0; i<nrows; ++i){
//            for (unsigned int j=0; j<ncols; ++j){

//                if ((*this)(i, j) != m2(i, j)){
//                    return false;
//                }
//            }
//        }
        return true;
    }

    void set(
            casadi::MX v0,
            casadi::MX v1,
            casadi::MX v2){
        (*this)(0) = v0;
        (*this)(1) = v1;
        (*this)(2) = v2;
    }
    void set(
            casadi::MX v0,
            casadi::MX v1,
            casadi::MX v2,
            casadi::MX v3,
            casadi::MX v4,
            casadi::MX v5){
        (*this)(0) = v0;
        (*this)(1) = v1;
        (*this)(2) = v2;
        (*this)(3) = v3;
        (*this)(4) = v4;
        (*this)(5) = v5;
    }
};

/* MX_XD_STATIC_H */
#endif

