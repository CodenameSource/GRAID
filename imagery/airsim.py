import cosysairsim as airsim
import numpy as np


def collect_image_airsim(client: airsim.MultirotorClient, camera_name, image_type, compress=True):
    """
    Collect an image from the drone.

    Arguments:
    - camera_name (str): The name of the camera to collect the image from.
    - image_type (str): The type of image to collect.

    Returns:
    - np.ndarray: The image data.
    """

    if image_type == "depth":
        image_type = airsim.ImageType.DepthPerspective
        pixels_as_float = True
    elif image_type == "rgb":
        image_type = airsim.ImageType.Scene
        pixels_as_float = False
    else:
        raise ValueError(f"Invalid image type: {image_type}")

    image_request = airsim.ImageRequest(camera_name, image_type, pixels_as_float, compress)
    response = client.simGetImages([image_request])[0]
    if image_type == airsim.ImageType.DepthPerspective:
        image_data = np.array(response.image_data_float, dtype=np.float32)
        image_data = image_data.reshape(response.height, response.width)
        image_data = image_data.clip(0, 20)
    else:
        image_data = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        image_data = image_data.reshape(response.height, response.width, 3)
    return image_data
