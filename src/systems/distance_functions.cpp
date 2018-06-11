/**
 * @file distance_functions.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Original work Copyright (c) 2014, Rutgers the State University of New Jersey, New Brunswick
 * Modified work Copyright 2017 Oleg Y. Sinyavskiy
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Original authors: Zakary Littlefield, Kostas Bekris
 * Modifications by: Oleg Y. Sinyavskiy
 *
 */

#include <assert.h>
#include "systems/distance_functions.h"
#include "systems/two_link_acrobot.hpp"


double euclidean_distance::distance(const double* point1, const double* point2, unsigned int state_dimensions) const {
    double result = 0;
    runtime_assert(state_dimensions == _is_circular_topology.size());
    for (unsigned int i=0; i<state_dimensions; ++i) {
        double weight = _weights[i]*_weights[i];
        if (_is_circular_topology[i]) {
            double val = fabs(point1[i]-point2[i]);
            if(val > M_PI)
                val = 2*M_PI-val;
            result += weight*val*val;
        } else {
            result += weight*(point1[i]-point2[i]) * (point1[i]-point2[i]);
        }
    }
    return std::sqrt(result);
};


double two_link_acrobot_distance::distance(const double* p0, const double* p1, unsigned int state_dimensions) const
{
    return two_link_acrobot_t::distance(p0, p1, state_dimensions);
}


