from typing import Optional

from imagery.image_config import ImageStreamsConfig


class ConnectionConfig:
    """
    Configuration for establishing a connection to the drone.

    Attributes:
    - device (str): The device path (e.g., '/dev/ttyUSB0', '192.168.1.12:41451').
    - type (str): The connection type ('serial', 'udp', 'airsim').
    - baud (int): The baud rate for the connection.
    - source_system (int): Source system identifier for the connection.
    - source_component (int): Source component identifier for the connection.
    - image_streams (Optional[ImageStreamsConfig]): Associated image stream configurations.
    """

    def __init__(
            self,
            device: str,
            type: str,
            baud: int = 115200,
            source_system: int = 255,
            source_component: int = 0,
            image_streams: Optional[ImageStreamsConfig] = None
    ):
        self.device = device
        self.type = type
        self.baud = baud
        self.source_system = source_system
        self.source_component = source_component
        self.image_streams = image_streams or ImageStreamsConfig()

    def add_image_stream(self, image_config):
        self.image_streams.add_image_stream(image_config)
