import cv2

from .airsim import collect_image_airsim
from .create_depth_map import compute_stereo_depth_map
# from .estimate_depth_map import estimate_depth_map
from .udp import collect_image_from_udp_stream
from Smav import Drone
from imagery.connection_config import ConnectionConfig
from observation_grid import ObservationGrid

estimate_depth_map = ""

class DroneExtended(Drone):
    def __init__(self, connection_config: ConnectionConfig, origin_grid: ObservationGrid = None,
                 disparity_algorithm: str = "bm"):
        if connection_config.type == "ip":
            if ":" in connection_config.device:
                port = int(connection_config.device.split(":")[1])
            else:
                port = 5760
            super().__init__(connection_config.device, port, simulator_mode=False, simulator_address='')
        elif connection_config.type == "airsim":
            super().__init__(connection_config.device, simulator_mode=True, simulator_address=connection_config.device)

        self.origin_grid = origin_grid
        self.image_streams = connection_config.image_streams
        self.disparity_algorithm = disparity_algorithm

    def set_origin_grid(self, origin_grid):
        self.origin_grid = origin_grid

    def get_origin_grid(self):
        return self.origin_grid

    def get_images(self):
        """
        High level function that generates images from the drone's sensors.
        Returns: [{"config": ImageConfig, "images": [image]}]
        """

        images = []

        if self.sim_mode:
            for stream in self.image_streams.image_configs:
                if stream.img_type == "stereo":
                    if len(stream.endpoint) != 2:
                        raise ValueError("Stereo image stream requires exactly two endpoints")

                    image1 = collect_image_airsim(self.simulator, stream.endpoint[0], stream.img_type)
                    image2 = collect_image_airsim(self.simulator, stream.endpoint[1], stream.img_type)

                    images.append({"config": stream, "images": [image1, image2]})
                else:
                    image = collect_image_airsim(self.simulator, stream.endpoint, stream.img_type)

                    images.append({"config": stream, "images": [image]})
        else:
            for stream in self.image_streams.image_configs:
                if stream.img_type == "stereo":
                    if len(stream.endpoint) != 2:
                        raise ValueError("Stereo image stream requires exactly two endpoints")

                    if "udp" not in stream.endpoint[0] or "udp" not in stream.endpoint[1]:
                        raise ValueError("Stereo image stream requires UDP endpoints")

                    port1 = int(stream.endpoint[0].split(":")[1])
                    port2 = int(stream.endpoint[1].split(":")[1])

                    image1 = collect_image_from_udp_stream(port1, stream.resolution[0], stream.resolution[1])
                    image2 = collect_image_from_udp_stream(port2, stream.resolution[0], stream.resolution[1])

                    images.append({"config": stream, "images": [image1, image2]})

                else:
                    if "udp" not in stream.endpoint:
                        raise ValueError("Image stream requires UDP endpoint")

                    port = int(stream.endpoint.split(":")[1])

                    image = collect_image_from_udp_stream(port, stream.resolution[0], stream.resolution[1])
                    images.append({"config": stream, "images": [image]})

        return images

    def get_depth_images(self):
        """
        High level function that generates depth images from the drone's sensors.
        Returns: [{"config": ImageConfig, "images": [depth_image]}]
        """

        disparity_algorithm = self.disparity_algorithm
        depth_images = []

        for image in self.get_images():
            image_config = image["config"]
            image_data = image["images"]

            if image_config.img_type == "stereo":
                depth_map = self.compute_stereo_depth_map(image, disparity_algorithm)
                depth_images.append({"config": image_config, "images": [depth_map]})
            elif image_config.img_type == "depth":
                depth_images.append({"config": image_config, "images": image_data})
            elif image_config.img_type == "rgb":
                estimated_depth_map = self.estimate_depth_map(image)
                if estimated_depth_map is None:
                    raise ValueError("Error estimating depth map")

                depth_images.append({"config": image_config, "images": [estimated_depth_map]})
            else:
                print(f"Unsupported image type: {image_config.img_type}, skipping...")

        return depth_images

    @staticmethod
    def compute_stereo_depth_map(view, disparity_algorithm):
        """High level depth image generation function that processes the input from the stereo camera onboard the drone"""
        return compute_stereo_depth_map(view["images"][0], view["images"][1], view["config"].lens_distance,
                                        view["config"].focal_length, disparity_algorithm)

    @staticmethod
    def estimate_depth_map(image):
        """High level depth estimation function utilising Apple Depth Pro algorithm"""

        return estimate_depth_map(image["images"][0], image["config"].resolution, image["config"].focal_length)
