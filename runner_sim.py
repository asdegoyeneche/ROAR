import logging
import numpy as np
import warnings
from pathlib import Path
from ROAR_Sim.configurations.configuration import Configuration as CarlaConfig
from ROAR_Sim.carla_client.carla_runner import CarlaRunner
from ROAR.agent_module.pure_pursuit_agent import PurePursuitAgent
# from ROAR.agent_module.point_cloud_agent import PointCloudAgent
from ROAR.configurations.configuration import Configuration as AgentConfig
from ROAR.agent_module.special_agents.waypoint_generating_agent import WaypointGeneratigAgent
#from ROAR.agent_module.pid_agent import PIDAgent
from ROAR.agent_module.lqr_agent import LQRAgent


def main():
    agent_config = AgentConfig.parse_file(Path("./ROAR_Sim/configurations/agent_configuration.json"))
    carla_config = CarlaConfig.parse_file(Path("./ROAR_Sim/configurations/configuration.json"))

    carla_runner = CarlaRunner(carla_settings=carla_config,
                               agent_settings=agent_config,
                               npc_agent_class=PurePursuitAgent)

	''' Data collection code. Currently unnecessary
    # make csv file to store some data in
    # we have current position x, y, z, current velocity x, y, z, next waypoint position x, y, z,
    # next waypoint direction relative to the current position of the car x, y, z, steering, and throttle
    csvNotes = "{}\n{}\n".format("we have current car position x, y, z, current car velocity x, y, z, next waypoint position x, y, z,", 
                               "next waypoint direction relative to the current position of the car x, y, z, steering, and throttle")
    csvHeader = "px, py, pz, vx, vy, vz, wpx, wpy, wpz, wvx, wvy, wvz, steering, throttle\n"
    with open("tmp/pid_data.csv", "w") as f:
        f.write(csvNotes)
        f.write(csvHeader)
	'''

    try:
        my_vehicle = carla_runner.set_carla_world()
        #agent = PIDAgent(vehicle=my_vehicle, agent_settings=agent_config)
        agent = LQRAgent(vehicle=my_vehicle, agent_settings=agent_config)
        carla_runner.start_game_loop(agent=agent, use_manual_control=False)
    except Exception as e:
        logging.error(f"Something bad happened during initialization: {e}")
        carla_runner.on_finish()
        logging.error(f"{e}. Might be a good idea to restart Server")


if __name__ == "__main__":
    logging.basicConfig(format='%(levelname)s - %(asctime)s - %(name)s '
                               '- %(message)s',
                        level=logging.DEBUG)
    logging.getLogger("matplotlib").setLevel(logging.WARNING)
    warnings.simplefilter("ignore")
    np.set_printoptions(suppress=True)
    main()
