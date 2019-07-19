#!/usr/bin/env python
# Author: Yubai Di
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


def nominal_pos(space):
    # Neutral position is defined as: [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]
    pos = ob.State(space)
    pos[0] = 0
    pos[1] = 0
    pos[2] = 0
    pos[3] = 0
    pos[4] = 0
    pos[5] = 0
    pos[6] = 0
    return pos

def neutral_position(space):
    # Neutral position is defined as: [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]
    pos = ob.State(space)
    pos[0] = 0
    pos[1] = -0.55
    pos[2] = 0
    pos[3] = 0.75
    pos[4] = 0
    pos[5] = 1.26
    pos[6] = 0
    return pos

NEURTRAL_POSITION = [0.0, -0.55, 0.0, 0.75, 0.0, 1.26, 0.0]
