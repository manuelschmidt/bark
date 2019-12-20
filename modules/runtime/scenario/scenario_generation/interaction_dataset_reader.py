# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

from bark.models.behavior import BehaviorStaticTrajectory
from bark.models.dynamic import StateDefinition
from python.utils import dataset_reader
from python.utils import dict_utils
import numpy as np


def bark_state_from_motion_state(state):
    bark_state = np.zeros(int(StateDefinition.MIN_STATE_SIZE))
    bark_state[int(StateDefinition.TIME_POSITION)] = state.time_stamp_ms / 1000.0
    bark_state[int(StateDefinition.X_POSITION)] = state.x
    bark_state[int(StateDefinition.Y_POSITION)] = state.y
    bark_state[int(StateDefinition.THETA_POSITION)] = state.psi_rad
    bark_state[int(StateDefinition.VEL_POSITION)] = pow(pow(state.vx, 2) + pow(state.vy, 2), 0.5)
    return bark_state.reshape((1, int(StateDefinition.MIN_STATE_SIZE)))


def trajectory_from_trackfile(filename, agent_id, start=0, end=None):
    track_dictionary = dataset_reader.read_tracks(filename)
    track = track_dictionary[agent_id]
    states = list(dict_utils.get_item_iterator(track.motion_states))
    if end is None:
        end = states[-1][0]
    filtered_motion_states = list(filter(lambda s: start <= s[0] <= end, states))
    n = len(filtered_motion_states)
    traj = np.zeros((n, int(StateDefinition.MIN_STATE_SIZE)))
    for i, state in enumerate(filtered_motion_states):
        traj[i, :] = bark_state_from_motion_state(state[1])
    listlistfloat = [list(state) for state in traj]
    return listlistfloat

def behavior_from_trackfile(params):
    fname = params["filename"]
    track_id = params["track_id"]
    start = params["start_offset","The timestamp in ms when to start the trajectory", 0]
    end = params["end_offset", "The timestamp in ms when to end the trajectory", None]
    temp_params = params
    temp_params["static_trajectory"] = trajectory_from_trackfile(fname, track_id, start, end)
    return BehaviorStaticTrajectory(temp_params)
