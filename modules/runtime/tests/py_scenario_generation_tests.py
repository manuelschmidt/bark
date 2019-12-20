# Copyright (c) 2019 fortiss GmbH
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT


import unittest
import os, time
from modules.runtime.scenario.scenario_generation.uniform_vehicle_distribution\
  import UniformVehicleDistribution
from modules.runtime.scenario.scenario_generation.scenario_generation\
  import ScenarioGeneration
from modules.runtime.scenario.scenario_generation.drone_challenge\
  import DroneChallengeScenarioGeneration
from modules.runtime.scenario.scenario_generation.interaction_dataset_reader import BehaviorInteractionDataset
from modules.runtime.commons.parameters import ParameterServer


class ScenarioGenerationTests(unittest.TestCase):
  def assertAlmostEqualList(self, a, b, places):
    for (i_a, i_b) in zip(a, b):
      self.assertAlmostEqual(i_a, i_b, places)

  def test_uniform_vehicle_distribution_default_params(self):
    scenario_generation = UniformVehicleDistribution(num_scenarios=2, random_seed=0)
    scenario_generation.dump_scenario_list("test.scenario")

    scenario_loader = ScenarioGeneration()
    scenario_loader.load_scenario_list("test.scenario")

    self.assertEqual(len(scenario_loader._scenario_list), 2)
    self.assertEqual(len(scenario_loader._scenario_list[0]._agent_list), len(scenario_generation._scenario_list[0]._agent_list))

  def test_drone_challenge_default_params(self):
    scenario_generation = DroneChallengeScenarioGeneration(num_scenarios=2, random_seed=0)
    scenario_generation.dump_scenario_list("test.scenario")

    scenario_loader = ScenarioGeneration()
    scenario_loader.load_scenario_list("test.scenario")

    self.assertEqual(len(scenario_loader._scenario_list), 2)
    self.assertEqual(len(scenario_loader._scenario_list[0]._agent_list), len(scenario_generation._scenario_list[0]._agent_list))

  def test_interaction_dataset_reader(self):
    params = ParameterServer()
    params["filename"] = "external/com_github_interaction-dataset_interaction-dataset/recorded_trackfiles/.TestScenarioForScripts/vehicle_tracks_000.csv"
    params["track_id"] = 1
    behavior = BehaviorInteractionDataset(params)
    traj = behavior.static_trajectory
    self.assertEqual((100, 5), traj.shape)
    self.assertAlmostEqualList([0.1, 1.0, 2.5, 0.0, 10.0], list(traj[0, :]), 5)
    self.assertAlmostEqualList([10.0, 100.0, 2.5, 0.0, 10.0], list(traj[traj.shape[0] - 1, :]), 5)

if __name__ == '__main__':
  unittest.main()

