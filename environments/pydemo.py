try:
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import util as ou
    from ompl import base as ob
    from ompl import geometric as og
from time import sleep
from math import fabs

# This is a problem-specific sampler that automatically generates valid
# states; it doesn't need to call SpaceInformation::isValid. This is an
# example of constrained sampling. If you can explicitly describe the set valid
# states and can draw samples from it, then this is typically much more
# efficient than generating random samples from the entire state space and
# checking for validity.
class MyValidStateSampler(ob.ValidStateSampler):
    def __init__(self, si):
        super(MyValidStateSampler, self).__init__(si)
        self.name_ = "my sampler"
        self.rng_ = ou.RNG()

    # Generate a sample in the valid part of the R^3 state space.
    # Valid states satisfy the following constraints:
    # -1<= x,y,z <=1
    # if .25 <= z <= .5, then |x|>.8 and |y|>.8
    def sample(self, state):
        z = self.rng_.uniformReal(-1, 1)

        if z > .25 and z < .5:
            x = self.rng_.uniformReal(0, 1.8)
            y = self.rng_.uniformReal(0, .2)
            i = self.rng_.uniformInt(0, 3)
            if i == 0:
                state[0] = x-1
                state[1] = y-1
            elif i == 1:
                state[0] = x-.8
                state[1] = y+.8
            elif i == 2:
                state[0] = y-1
                state[1] = x-1
            elif i == 3:
                state[0] = y+.8
                state[1] = x-.8
        else:
            state[0] = self.rng_.uniformReal(-1, 1)
            state[1] = self.rng_.uniformReal(-1, 1)
        state[2] = z
        return True



# This function is needed, even when we can write a sampler like the one
# above, because we need to check path segments for validity
def isStateValid(state):
    # Let's pretend that the validity check is computationally relatively
    # expensive to emphasize the benefit of explicitly generating valid
    # samples
    sleep(.001)
    # Valid states satisfy the following constraints:
    # -1<= x,y,z <=1
    # if .25 <= z <= .5, then |x|>.8 and |y|>.8
    return not (fabs(state[0] < .8) and fabs(state[1] < .8) and \
        state[2] > .25 and state[2] < .5)

# return an obstacle-based sampler
def allocOBValidStateSampler(si):
    # we can perform any additional setup / configuration of a sampler here,
    # but there is nothing to tweak in case of the ObstacleBasedValidStateSampler.
    return ob.ObstacleBasedValidStateSampler(si)

# return an instance of my sampler
def allocMyValidStateSampler(si):
    return MyValidStateSampler(si)

def plan(samplerIndex):
    # construct the state space we are planning in
    space = ob.RealVectorStateSpace(3)

    # set the bounds
    bounds = ob.RealVectorBounds(3)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    # define a simple setup class
    ss = og.SimpleSetup(space)

    # set state validity checking for this space
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

    # create a start state
    start = ob.State(space)
    start[0] = 0
    start[1] = 0
    start[2] = 0

    # create a goal state
    goal = ob.State(space)
    goal[0] = 0
    goal[1] = 0
    goal[2] = 1

    # set the start and goal states;
    ss.setStartAndGoalStates(start, goal)

    # set sampler (optional; the default is uniform sampling)
    si = ss.getSpaceInformation()
    if samplerIndex == 1:
        # use obstacle-based sampling
        si.setValidStateSamplerAllocator(ob.ValidStateSamplerAllocator(allocOBValidStateSampler))
    elif samplerIndex == 2:
        # use my sampler
        si.setValidStateSamplerAllocator(ob.ValidStateSamplerAllocator(allocMyValidStateSampler))

    # create a planner for the defined space
    planner = og.FMT(si)
    ss.setPlanner(planner)

    # attempt to solve the problem within ten seconds of planning time
    solved = ss.solve(10.0)
    if solved:
        print("Found solution:")
        # print the path to screen
        print(ss.getSolutionPath())
    else:
        print("No solution found")


if __name__ == '__main__':
    print("Using default uniform sampler:")
    plan(0)
    print("\nUsing obstacle-based sampler:")
    plan(1)
    print("\nUsing my sampler:")
    plan(2)