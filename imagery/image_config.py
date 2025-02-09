from typing import Tuple, List, Union

class ImageConfig:
    """
    Configuration for a single image stream.

    Attributes:
    - resolution (Tuple[int, int]): The resolution of the image (width, height).
    - fps (float): Frames per second of the image stream.
    - horizontal_fov (float): Horizontal field of view in degrees.
    - vertical_fov (float): Vertical field of view in degrees.
    - img_type (Union[str, List[str]]): Type of image (e.g., 'RGB', 'Depth', 'Stereo').
    - endpoint (Union[str, List[str]]): Endpoint(s) for receiving image data.
    - yaw_pitch_roll (Tuple[float, float, float]): Rotation offset from drone (pitch, yaw, roll). (0, 0, 0) equals zero rotation from drone's forward direction, i.e camera is facing forwards.
    - lens_distance (float): Lens distance for stereo configurations.
    """

    def __init__(
        self,
        resolution: Tuple[int, int],
        fps: float,
        horisontal_fov: float,
        vertical_fov: float,
        img_type: Union[str, List[str]],
        endpoint: Union[str, List[str]],
        rotation: Tuple[float, float, float] = (0, 0, 0),
        lens_distance: float = 0,
        focal_length: float = 0,
    ):
        self.resolution = resolution
        self.fps = fps
        self.horizontal_fov = horisontal_fov
        self.vertical_fov = vertical_fov
        self.img_type = img_type
        self.endpoint = endpoint
        self.rotation = rotation
        self.lens_distance = lens_distance
        self.focal_length = focal_length


class ImageStreamsConfig:
    """
    Configuration for multiple image streams.
    """

    def __init__(self):
        self.image_configs: List[ImageConfig] = []

    def add_image_stream(self, image_config: ImageConfig) -> None:
        """
        Adds a new image stream configuration.

        Args:
        - image_config (ImageConfig): The image stream configuration to add.
        """
        self.image_configs.append(image_config)
