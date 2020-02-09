#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os  # for time functions
from search import *  # for search engines
from sokoban import SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems


def sokoban_goal_state(state):
    '''
  @return: Whether all boxes are stored.
  '''
    for box in state.boxes:
        if box not in state.storage:
            return False
    return True


def heur_manhattan_distance(state):
    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.
    total_distance = 0
    for box in state.boxes:
        min_distance = float("inf")
        for target in state.storage:
            min_distance = min(min_distance, manhattan_distance(box, target))
        total_distance += min_distance
    return total_distance


def manhattan_distance(x, y):
    return abs(x[0] - y[0]) + abs(x[1] - y[1])


# SOKOBAN HEURISTICS
def trivial_heuristic(state):
    '''trivial admissible sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
    count = 0
    for box in state.boxes:
        if box not in state.storage:
            count += 1
    return count


def heur_alternate(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.
    filled_storage = []
    free_boxes = []
    free_storage = []
    for box in state.boxes:
        if box in state.storage:
            filled_storage.append(box)
        else:
            free_boxes.append(box)
    for target in state.storage:
        if target not in filled_storage:
            free_storage.append(target)
    total_distance = 0
    if deadlock_corner(free_boxes, state) or deadlock_wall(free_boxes, free_storage, state):
        return float("inf")
    free_box_copy = free_boxes.copy()
    while free_box_copy:
        closest_box = None
        closest_target = None
        closest_dist = float("inf")
        for box in free_box_copy:
            min_distance = float("inf")
            for target in free_storage:
                dist = manhattan_distance(box, target) + obstacle_on_path(state, box, target) * 2
                if dist < min_distance:
                    min_target = target
                    min_distance = dist
            if min_distance < closest_dist:
                closest_box = box
                closest_target = min_target
                closest_dist = min_distance
        free_box_copy.remove(closest_box)
        free_storage.remove(closest_target)
        total_distance += closest_dist

        min_distance = float("inf")
        for robot in state.robots:
            min_distance = min(min_distance,
                               manhattan_distance(closest_box, robot) - 1)
        total_distance += min_distance

    return total_distance


def obstacle_on_path(state, start, target):
    count = 0
    for obstacle in state.obstacles:
        if obstacle[0] == start[0] and (target[1] <= obstacle[1] <= start[1] or target[1] >= obstacle[1] >= start[1]):
            count += 1
    for box in state.boxes:
        if box[0] == start[0] and (target[1] <= box[1] <= start[1] or target[1] >= box[1] >= start[1]):
            count += 1
    for robot in state.robots:
        if robot[0] == start[0] and (target[1] <= robot[1] <= start[1] or target[1] >= robot[1] >= start[1]):
            count += 1
    count1 = 0
    for obstacle in state.obstacles:
        if obstacle[1] == start[1] and (target[0] <= obstacle[0] <= start[0] or target[0] >= obstacle[0] >= start[0]):
            count1 += 1
    for box in state.boxes:
        if box[1] == start[1] and (target[0] <= box[0] <= start[0] or target[0] >= box[0] >= start[0]):
            count1 += 1
    for robot in state.robots:
        if robot[1] == start[1] and (target[0] <= robot[0] <= start[0] or target[0] >= robot[0] >= start[0]):
            count1 += 1
    return min(count, count1)


def deadlock_corner(free_box, state):
    for box in free_box:
        up = (box[0], box[1] + 1)
        down = (box[0], box[1] - 1)
        left = (box[0] - 1, box[1])
        right = (box[0] + 1, box[1])
        not_up = up in state.obstacles or up[1] == state.height
        not_down = down in state.obstacles or down[1] == -1
        not_left = left in state.obstacles or left[0] == -1
        not_right = right in state.obstacles or right[0] == state.width
        if not_up and (not_left or not_right):
            return True
        if not_down and (not_left or not_right):
            return True
        not_up = not_up or up in state.boxes
        not_down = not_down or down in state.boxes
        not_left = not_left or left in state.boxes
        not_right = not_right or right in state.boxes
        up_left = (box[0] - 1, box[1] + 1)
        down_left = (box[0] - 1, box[1] - 1)
        up_right = (box[0] + 1, box[1] + 1)
        down_right = (box[0] + 1, box[1] - 1)
        not_up_left = up_left in state.boxes or up_left in state.obstacles
        not_up_right = up_right in state.boxes or up_right in state.obstacles
        not_down_left = down_left in state.boxes or down_left in state.obstacles
        not_down_right = down_right in state.boxes or down_right in state.obstacles
        if not_up:
            if not_left and not_up_left:
                return True
            if not_right and not_up_right:
                return True
        if not_down:
            if not_left and not_down_left:
                return True
            if not_right and not_down_right:
                return True
    return False


def deadlock_wall(free_box, free_storage, state):
    for box in free_box:
        if box[0] == 0 or box[0] == state.width - 1:
            if (box[0], box[1] + 1) in state.boxes or (box[0], box[1] - 1) in state.boxes or \
                    (box[0], box[1] + 1) in state.obstacles or (box[0], box[1] - 1) in state.obstacles:
                return True
            dead = True
            for target in free_storage:
                if target[0] == box[0]:
                    dead = False
            if dead:
                return dead
        if box[1] == 0 or box[1] == state.height - 1:
            if (box[0] + 1, box[1]) in state.boxes or (box[0] - 1, box[1]) in state.boxes or \
                    (box[0] + 1, box[1]) in state.obstacles or (box[0] - 1, box[1]) in state.obstacles:
                return True
            dead = True
            for target in free_storage:
                if target[1] == box[1]:
                    dead = False
            if dead:
                return dead
    return False


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0


def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    # Many searches will explore nodes (or states) that are ordered by their f-value.
    # For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    # You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    # The function must return a numeric f-value.
    # The value will determine your state's position on the Frontier list during a 'custom' search.
    # You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + weight * sN.hval


def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound=10):
    # IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''
    start_time = os.times()[0]
    timebound -= 0.05
    search = SearchEngine('custom')
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    search.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_function)
    cost_bound = (float('inf'), float('inf'), float('inf'))
    best_state = False
    time_limit = timebound
    while time_limit > 0:
        cur_state = search.search(time_limit, cost_bound)
        if cur_state is False:
            return best_state
        else:
            cost_bound = (float('inf'), float('inf'), cur_state.gval)
            best_state = cur_state
        time_limit = timebound - (os.times()[0] - start_time)

    return best_state


def anytime_gbfs(initial_state, heur_fn, timebound=10):
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''
    start_time = os.times()[0]
    timebound -= 0.05
    search = SearchEngine(strategy='best_first')
    search.init_search(initial_state, sokoban_goal_state, heur_fn)
    cost_bound = (float('inf'), float('inf'), float('inf'))
    best_state = False
    time_limit = timebound
    while time_limit:
        cur_state = search.search(time_limit, cost_bound)
        if not cur_state:
            return best_state
        else:
            cost_bound = (cur_state.gval, float('inf'), float('inf'))
            best_state = cur_state
        time_limit = timebound - (os.times()[0] - start_time)
    return best_state
