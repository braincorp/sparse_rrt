/**
 * @file distance_functions.hpp
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

#ifndef SPARSERRT_DISTANCE_FUNCTIONS_H
#define SPARSERRT_DISTANCE_FUNCTIONS_H

#include <functional>
#include <vector>
#include <cmath>
#include "utilities/runtime_assert.hpp"


class distance_t
{
public:
    /**
     * @brief Finds the distance between two states.
     * @details Finds the distance between two states.
     *
     * @param point1 First point.
     * @param point2 Second point.
     * @param state_dimensions dimensionality of the state space
     *
     * @return The distance between point1 and point2.
     */
    virtual double distance(const double* point1, const double* point2, unsigned int state_dimensions) const = 0;
};


/**
 * @brief Computer of weighted euclidean distance between state points
 * @details Computer of weighted euclidean distance between state points. Computes the distance based on the topology of the system
 *
 */
class euclidean_distance: public distance_t
{
public:

    /**
     * @brief euclidean_distance constructor
     * @details euclidean_distance constructor
     *
     * @param is_circular_topology An array that has flags for each dimensions in the state space whether its circular or not
     * @param weights An array of weights for each coordinate
     *
     */
    euclidean_distance(const std::vector<bool>& is_circular_topology,
                       const std::vector<double>& weights)
        : _is_circular_topology(is_circular_topology)
        , _weights(weights)
    { };


	/**
	 * @copydoc distance_t::distance()
	 */
    double distance(const double* point1, const double* point2, unsigned int state_dimensions) const override;

private:
    std::vector<bool> _is_circular_topology;
    std::vector<double> _weights;
};


/**
 * @brief Computer of two joint pole distance
 * @details Computer of two joint pole distance for acrobot robot (speeds up planning)
 *
 */
class two_link_acrobot_distance: public distance_t
{
public:
	/**
	 * @copydoc distance_t::distance()
	 */
    double distance(const double* point1, const double* point2, unsigned int state_dimensions) const override;
};

#endif //SPARSERRT_DISTANCE_FUNCTIONS_H
