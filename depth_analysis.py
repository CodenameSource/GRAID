import numpy as np

from observation import ObservationGraph


# TODO: Account for altitude

def generate_projection_images(depth_data, max_depth=10, fov_horizontal=90, fov_vertical=60):
    """
    Generates projection-level images for given depth data up to max_depth meters.

    Args:
        depth_data (numpy.ndarray): Normalized depth image (depth in meters).
        max_depth (int): Maximum depth to generate projection images for.
        fov_horizontal (float): Horizontal field of view in degrees. - For future improvements
        fov_vertical (float): Vertical field of view in degrees.

    Returns:
        list of numpy.ndarray: Cropped projection images for each depth level.
    """
    depth_data_shape = depth_data.shape

    # Convert FOV to radians
    fov_vertical_radians = np.radians(fov_vertical)

    projection_images = []

    for depth in range(1, max_depth + 1):
        # Perform vertical cropping to keep the center line within 0.75 meters
        vertical_crop_pixels = int(0.75 / (2 * np.tan(fov_vertical_radians / 2) / depth_data_shape[0] * depth))
        center_line = depth_data_shape[0] // 2
        top_crop = max(0, center_line - vertical_crop_pixels)
        bottom_crop = min(depth_data_shape[0], center_line + vertical_crop_pixels)

        # Crop the image vertically
        cropped_image = depth_data[top_crop:bottom_crop, :]
        projection_images.append(cropped_image)

    return projection_images


def apply_kernels(projection_images, kernel_horizontal_size=0.2, kernel_vertical_size=1.5,
                  fov_horizontal=90, fov_vertical=60, percentile=10):
    """
    Applies kernel logic to projection images based on a given percentile.

    Args:
        projection_images (list of numpy.ndarray): Cropped projection images for each depth level.
        kernel_horizontal_size (float): Horizontal kernel size in meters.
        kernel_vertical_size (float): Vertical kernel size in meters.
        fov_horizontal (float): Horizontal field of view in degrees.
        fov_vertical (float): Vertical field of view in degrees.
        percentile (int): Percentile value to determine kernel activation.

    Returns:
        list of dict: List of kernels for each projection image. Each kernel is represented as a dictionary
                      containing kernel metadata and activation status.
    """
    # Convert FOV to radians
    fov_horizontal_radians = np.radians(fov_horizontal)
    fov_vertical_radians = np.radians(fov_vertical)

    # Calculate pixel-to-meter ratio (assume all images have the same dimensions)
    projection_shape = projection_images[0].shape
    pixel_to_meter_ratio_horizontal = 2 * np.tan(fov_horizontal_radians / 2) / projection_shape[1]
    pixel_to_meter_ratio_vertical = 2 * np.tan(fov_vertical_radians / 2) / projection_shape[0]

    kernels_per_projection = []

    for depth, cropped_image in enumerate(projection_images, start=1):
        # Calculate kernel size in pixels for the current depth
        kernel_horizontal_pixels = int(kernel_horizontal_size / (pixel_to_meter_ratio_horizontal * depth))
        kernel_vertical_pixels = int(kernel_vertical_size / (pixel_to_meter_ratio_vertical * depth))

        kernels = []

        for x in range(0, cropped_image.shape[1], kernel_horizontal_pixels):
            for y in range(0, cropped_image.shape[0], kernel_vertical_pixels):
                # Extract the kernel region
                kernel = cropped_image[y:y + kernel_vertical_pixels, x:x + kernel_horizontal_pixels]

                # Check the specified percentile depth within the kernel
                if kernel.size > 0:
                    kernel_percentile = np.percentile(kernel, percentile)
                    kernel_activated = (kernel_percentile < depth) and (kernel_percentile > depth - 1)

                    if not kernel_activated:
                        pass

                    # Save the kernel details
                    kernels.append({
                        "x": x,
                        "y": y,
                        "activated": kernel_activated
                    })

        kernels_per_projection.append({
            "projection_level": cropped_image,
            "depth": depth,
            "horisontal_size": kernel_horizontal_size,
            "vertical_size": kernel_vertical_size,
            "kernel_horisontal_pixels": kernel_horizontal_pixels,
            "kernel_vertical_pixels": kernel_vertical_pixels,
            "kernels": kernels
        })

    return kernels_per_projection


def bundle_kernels_to_larger_verticals(kernels_per_projection, target_vertical_size):
    """
    Bundles kernels into larger vertical sizes by aggregating smaller vertical kernels.

    Args:
        kernels_per_projection (list of dict): List of kernels for each projection image, where each kernel is represented
                                              as a dictionary containing kernel metadata and activation status.
        target_vertical_size (float): Desired vertical size of the bundled kernels (in meters).

    Returns:
        list of dict: Bundled kernels for each projection image with updated vertical sizes and activation status.
    """
    bundled_kernels_per_projection = []

    for projection in kernels_per_projection:
        current_vertical_size = projection["vertical_size"]
        vertical_ratio = round(target_vertical_size / current_vertical_size)

        if vertical_ratio <= 1:
            # No need to bundle if the target size is smaller or equal to the current size
            bundled_kernels_per_projection.append(projection)
            continue

        kernel_dict = {}

        # Group kernels into larger vertical segments
        for kernel in projection["kernels"]:
            vertical_group = kernel["y"] // (vertical_ratio * projection["kernel_vertical_pixels"])
            key = (kernel["x"], vertical_group)
            if key not in kernel_dict:
                kernel_dict[key] = []
            kernel_dict[key].append(kernel)

        # Aggregate masks and activations within each vertical group
        bundled_kernels = []
        for (x, vertical_group), kernels in kernel_dict.items():
            bundled_activation = False

            for kernel in kernels:
                if kernel["activated"]:
                    bundled_activation = True
                    break

            bundled_kernels.append({
                "x": x,
                "y": vertical_group * vertical_ratio * projection["kernel_vertical_pixels"],
                "activated": bundled_activation
            })

        bundled_kernels_per_projection.append({
            "projection_level": projection["projection_level"],
            "depth": projection["depth"],
            "horisontal_size": projection["horisontal_size"],
            "vertical_size": target_vertical_size,
            "kernel_horisontal_pixels": projection["kernel_horisontal_pixels"],
            "kernel_vertical_pixels": vertical_ratio * projection["kernel_vertical_pixels"],
            "kernels": bundled_kernels
        })

    return bundled_kernels_per_projection


def map_obstacles_graph(obs_graph: ObservationGraph, kernels_per_projection, percentile=10):
    """
    Creates an obstacle graph using the kernel activations.

    Args:
        obs_graph (ObservationGraph): Empty obstacle graph to populate.
        kernels_per_projection (list of dict): List of kernels for each projection image.
        percentile (int): Percentile value to determine kernel activation.

    Returns:
        ObsGraph: Graph with obstacles set based on kernel activations.
    """
    for projection in kernels_per_projection:
        depth = projection["depth"]
        kernel_h_pixels = projection["kernel_horisontal_pixels"]
        kernel_v_pixels = projection["kernel_vertical_pixels"]

        for kernel in projection["kernels"]:
            if kernel["activated"]:
                x = kernel["x"]
                y = kernel["y"]

                kernel_x_m = (x / projection["kernel_horisontal_pixels"]) * projection["horisontal_size"]

                # Create a mask for the kernel based on its size
                kernel_percentile_depth = np.percentile(
                    projection["projection_level"][y:y + kernel_v_pixels, x:x + kernel_h_pixels], percentile
                )

                # Set the obstacle in the graph if the percentile depth condition is met
                if depth > kernel_percentile_depth > depth - 1:
                    depth_start, x_start = obs_graph.kernel_to_graph_coord(kernel_percentile_depth, kernel_x_m)
                    _, x_end = obs_graph.kernel_to_graph_coord(kernel_percentile_depth + obs_graph.node_spacing,
                                                               kernel_x_m + projection["horisontal_size"])

                    obs_graph.set_obstacle(depth_start, depth_start, x_start, x_end)

    return obs_graph


def overlay_kernels_on_image(projection, opacity=0.5):
    """
    Overlays kernel activations on the original image with a specified opacity.

    Args:
        projection (dict): Projection image with kernel activations.
        opacity (float): Opacity of the overlay (0 to 1).

    Returns:
        numpy.ndarray: Image with kernel activations overlaid.
    """
    original_image = projection["projection_level"] * 255 / 15
    kernels_width = projection["kernel_horisontal_pixels"]
    kernels_height = projection["kernel_vertical_pixels"]
    kernels_per_projection = projection["kernels"]

    # Convert original image to RGB if necessary
    if len(original_image.shape) == 2 or original_image.shape[2] == 1:
        original_image = np.stack([original_image] * 3, axis=-1)

    # Create a red overlay for activated kernels
    red_overlay = np.zeros_like(original_image, dtype=np.uint8)

    for kernel in kernels_per_projection:
        if kernel["activated"]:
            x = kernel["x"]
            y = kernel["y"]

            # Ensure the overlay dimensions are valid
            overlay_y_end = min(y + kernels_height, red_overlay.shape[0])
            overlay_x_end = min(x + kernels_width, red_overlay.shape[1])
            red_overlay[y:overlay_y_end, x:overlay_x_end, 0] = 255

    # Blend the original image with the red overlay
    overlaid_image = np.clip((1 - opacity) * original_image + opacity * red_overlay, 0, 255).astype(np.uint8)

    return overlaid_image
