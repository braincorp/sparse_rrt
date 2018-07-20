/**
 * @file rrt.cpp
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

#include "motion_planners/rrt.hpp"
#include "nearest_neighbors/graph_nearest_neighbors.hpp"

#include <iostream>
#include <deque>


rrt_node_t::rrt_node_t(double* point, unsigned int state_dimension, rrt_node_t* a_parent, tree_edge_t&& a_parent_edge, double a_cost)
	    : tree_node_t(point, state_dimension, a_parent, std::move(a_parent_edge), a_cost)
{
}


void rrt_t::step(system_interface* system, int min_time_steps, int max_time_steps, double integration_step)
{
    double* sample_state = new double[this->state_dimension];
    double* sample_control = new double[this->control_dimension];

    this->random_state(sample_state);
    this->random_control(sample_control);

    double distance;
    rrt_node_t* nearest = (rrt_node_t*)metric.find_closest(sample_state, &distance)->get_state();
    int num_steps = this->random_generator.uniform_int_random(min_time_steps, max_time_steps);

    double cost = system->propagate(
        nearest->get_point(), this->state_dimension,
        sample_control, this->control_dimension,
        num_steps,
        sample_state, integration_step
    );
    if (!std::isnan(cost))
    {
        //create a new tree node
        double duration = num_steps*integration_step;
        rrt_node_t* new_node = static_cast<rrt_node_t*>(nearest->add_child(new rrt_node_t(
            sample_state, this->state_dimension, nearest,
            tree_edge_t(sample_control, this->control_dimension, duration),
            nearest->get_cost() + cost)
        ));
        metric.add_node(new_node);
        number_of_nodes++;

        if(this->goal_predicate(new_node->get_point(), this->state_dimension)) {
            if (this->best_goal == nullptr || new_node->get_cost() < this->best_goal->get_cost()) {
                this->best_goal = new_node;
            }
        }
    }
    delete sample_state;
    delete sample_control;
}

