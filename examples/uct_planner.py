# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


from modules.runtime.scenario.scenario_generation.uniform_vehicle_distribution import UniformVehicleDistribution
from modules.runtime.commons.parameters import ParameterServer
from modules.runtime.viewer.matplotlib_viewer import MPViewer
from modules.runtime.viewer.video_renderer import VideoRenderer
import os
try:
    from bark.models.behavior import BehaviorUCTSingleAgent
except RuntimeError:
    RuntimeError("BehaviorUCTSingleAgent not available, using ConstantVelocityModel")

scenario_param_file ="uct_planner.json" # must be within examples params folder
param_server = ParameterServer(filename= os.path.join("examples/params/",scenario_param_file))

scenario_generation = UniformVehicleDistribution(num_scenarios=1, random_seed=0, params=param_server)


viewer = MPViewer(params=param_server, x_range=[-16,16], y_range=[-2,30], follow_agent_id=True)
sim_step_time = param_server["simulation"]["step_time",
                                        "Step-time used in simulation",
                                        0.2]
sim_real_time_factor = param_server["simulation"]["real_time_factor",
                                                "execution in real-time or faster", 1]
scenario, idx = scenario_generation.get_next_scenario()

world_state = scenario.get_world_state()
world_state.agents[scenario.eval_agent_ids[0]].behavior_model = BehaviorUCTSingleAgent(param_server)
param_server.save("examples/params/mcts_params_written.json")

# world_state.agents[scenario.eval_agent_ids[0]].behavior_model

video_renderer = VideoRenderer(renderer=viewer, world_step_time=sim_step_time, render_intermediate_steps=10)
for _ in range(0, 40): # run scenario for 100 steps
    world_state.do_planning(sim_step_time)
    video_renderer.drawWorld(world_state, scenario.eval_agent_ids)
    world_state.do_execution(sim_step_time)

video_renderer.export_video(filename="examples/scenarios/test_video_intermediate", remove_image_dir=False)
