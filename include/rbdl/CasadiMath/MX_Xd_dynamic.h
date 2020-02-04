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

#include <rbdl_casadi/rbdl_casadi_config.h>
#include <casadi.hpp>

class MX_Xd_dynamic : public casadi::MX{
public:
    MX_Xd_dynamic(
            unsigned int nrows,
            unsigned int ncols = 1) : casadi::MX(nrows, ncols){
    }

    MX_Xd_dynamic(const casadi::MX& m) :
        casadi::MX(m){
    }

    static MX_Xd_dynamic Zero(unsigned int nrows, unsigned int ncols = 1){
        return casadi::MX::zeros(nrows, ncols);
    }

    casadi::MX operator[](unsigned int i) const {
        return (*this)(i, 0);
    }

};

/* MX_XD_DYNAMICS_H */
#endif

