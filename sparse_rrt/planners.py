import _sst_module

from sparse_rrt.visualization import render_svg, svg_header, svg_footer


def visualize_wrapper(parent_class):
    '''
    Class factory that wraps visualization function for planners
    :param parent_class: a planner class (RRTWrapper or SSTWrapper)
    :return: a class with visualization functions
    '''
    class VisualizeWrapper(parent_class):
        def visualize_tree(self, system, image_width=500, image_height=500):
            body_string = parent_class.visualize_tree(
                self,
                system,
                image_width=image_width,
                image_height=image_height
            )
            body_string += system.visualize_obstacles(image_width, image_height)
            return render_svg(svg_header(width=image_width, height=image_height) + body_string + svg_footer())

        def visualize_nodes(self, system, image_width=500, image_height=500):
            body_string = parent_class.visualize_nodes(
                self,
                system,
                image_width=image_width,
                image_height=image_height
            )
            body_string += system.visualize_obstacles(image_width, image_height)
            return render_svg(svg_header(width=image_width, height=image_height) + body_string + svg_footer())

    return VisualizeWrapper


class SST(visualize_wrapper(_sst_module.SSTWrapper)):
    '''
    Sparse stable trees planner
    '''
    def __init__(
        self,
        state_bounds,
        control_bounds,
        distance,
        start_state,
        goal_predicate,
        random_seed,
        sst_delta_near,
        sst_delta_drain):
        '''
        Construct SST
        :param state_bounds: numpy array (N x 2) with boundaries of the state space (min and max)
        :param control_bounds: numpy array (N x 2) with boundaries of the control space (min and max)
        :param distance: an instance of IDistance to compute distance between state points
        :param start_state: the start state (numpy array)
        :param goal_predicate: an instance of IGoalPredicate to determine when the system reaches the goal
        :param random_seed: seed for the random generator
        :param sst_delta_near: near distance threshold for SST
        :param sst_delta_drain: drain distance threshold for SST
        '''
        super(SST, self).__init__(
            state_bounds=state_bounds,
            control_bounds=control_bounds,
            distance=distance,
            start_state=start_state,
            goal_predicate=goal_predicate,
            random_seed=random_seed,
            sst_delta_near=sst_delta_near,
            sst_delta_drain=sst_delta_drain
        )

    def get_solution(self):
        '''
        Get current solution of the planning problem
        :return: either None - no solution is found so far or
            a tuple with:
                - state_trajectory: state trajectory for the solution
                - controls: control signals along the trajectory
                - durations:duration of control signals (in seconds! divide by dt to get number of steps)
                - costs: cummulative costs along the path (costs[-1] corresponds to the cost of the whole path)
        '''
        return super(SST, self).get_solution()


class RRT(visualize_wrapper(_sst_module.RRTWrapper)):
    '''
    RRT planner (baseline)
    '''
    def __init__(
        self,
        state_bounds,
        control_bounds,
        distance,
        start_state,
        goal_predicate,
        random_seed):
        '''
        Construct RRT
        :param state_bounds: numpy array (N x 2) with boundaries of the state space (min and max)
        :param control_bounds: numpy array (N x 2) with boundaries of the control space (min and max)
        :param distance: an instance of IDistance to compute distance between state points
        :param start_state: the start state (numpy array)
        :param goal_predicate: an instance of IGoalPredicate to determine when the system reaches the goal
        :param random_seed: seed for the random generator
        '''
        super(RRT, self).__init__(
            state_bounds=state_bounds,
            control_bounds=control_bounds,
            distance=distance,
            start_state=start_state,
            goal_predicate=goal_predicate,
            random_seed=random_seed
        )

    def get_solution(self):
        '''
        Get current solution of the planning problem
        :return: either None - no solution is found so far or
            a tuple with:
                - state_trajectory: state trajectory for the solution
                - controls: control signals along the trajectory
                - durations:duration of control signals (in seconds! divide by dt to get number of steps)
                - costs: cummulative costs along the path (costs[-1] corresponds to the cost of the whole path)
        '''
        return super(RRT, self).get_solution()
