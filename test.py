from time import sleep

import cosysairsim as airsim
import cv2

from Smav import Waypoint
from obstacle_avoidance import ObstacleAvoidance, ConnectionConfig
from imagery import ImageConfig, ImageStreamsConfig

if __name__ == "__main__":
    avoidance_agent = ObstacleAvoidance(5)
    drone_imagery_config = ImageStreamsConfig()
    drone_imagery_config.add_image_stream(ImageConfig((1080, 1920), 30, 45, 90, "depth", "0"))
    connection_config = ConnectionConfig(
        device="127.0.0.1",
        type="airsim",
        image_streams=drone_imagery_config,
        reset_simulation=True
    )

    avoidance_agent.initialise_drone(connection_config)
    simulator = avoidance_agent.drone().get_simulator()
    sleep(1)
    avoidance_agent.drone().simulator.enableApiControl(True)
    avoidance_agent.drone().takeoff(2)
    avoidance_agent.observation_space.altitude_origin = avoidance_agent.drone().get_location()['alt']

    avoidance_agent.initialise_mission_waypoints([
        Waypoint(10.000122, 10.000005, 3, 0)])

    avoidance_agent.start(20)
