import numpy as np
import cv2

from observation import ObservationGraph

from dataclasses import dataclass
from typing import List, Sequence


@dataclass
class Kernel:
    x: int
    y: int
    activated: bool
    depth: float


@dataclass
class ProjectionKernels:
    projection_level: np.ndarray
    depth: int
    kernel_horizontal_size_m: float
    kernel_vertical_size_m: float
    kernel_horizontal_pixels: int
    kernel_vertical_pixels: int
    kernels: List[Kernel]

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


def apply_kernels(
        projection_images: Sequence[np.ndarray],
        kernel_horizontal_size: float = 0.2,
        kernel_vertical_size: float = 1.5,
        fov_horizontal: float = 90.0,
        fov_vertical: float = 60.0,
        percentile: float = 10.0
) -> List[ProjectionKernels]:
    """
    Partition each depth‐level image into rectangular kernels and flag those
    whose given percentile depth lies within [depth-1, depth).

    Parameters
    ----------
    projection_images
        List of 2D depth images (one per depth level), assumed same shape.
    kernel_horizontal_size
        Kernel width in meters.
    kernel_vertical_size
        Kernel height in meters.
    fov_horizontal
        Horizontal field‐of‐view in degrees.
    fov_vertical
        Vertical field‐of‐view in degrees.
    percentile
        Percentile (0–100) used to summarize each kernel.

    Returns
    -------
    List[ProjectionKernels]
    """
    # --- Validate inputs ---
    if not projection_images:
        return []
    if kernel_horizontal_size <= 0 or kernel_vertical_size <= 0:
        raise ValueError("Kernel sizes must be positive.")
    if not (0.0 <= percentile <= 100.0):
        raise ValueError("Percentile must be in [0, 100].")

    # --- Precompute pixel‐to‐meter ratios ---
    h_px, w_px = projection_images[0].shape[:2]
    fh_rad = np.radians(fov_horizontal)
    fv_rad = np.radians(fov_vertical)
    m_per_px_h = (2 * np.tan(fh_rad / 2)) / w_px
    m_per_px_v = (2 * np.tan(fv_rad / 2)) / h_px

    result: List[ProjectionKernels] = []

    for depth_idx, img in enumerate(projection_images, start=1):
        # kernel size in pixels at this depth (ensure ≥1)
        kw = max(1, int(kernel_horizontal_size / (m_per_px_h * depth_idx)))
        kh = max(1, int(kernel_vertical_size / (m_per_px_v * depth_idx)))

        kernels: List[Kernel] = []
        # iterate over grid
        for y in range(0, h_px, kh):
            y_end = min(y + kh, h_px)
            for x in range(0, w_px, kw):
                x_end = min(x + kw, w_px)
                block = img[y:y_end, x:x_end]
                if block.size == 0:
                    continue

                # median or percentile summary
                val = np.percentile(block, percentile)
                # activated if depth-1 ≤ val < depth
                active = (depth_idx - 1) <= val < min(depth_idx, len(projection_images))
                kernels.append(Kernel(x=x, y=y, activated=active, depth=float(val)))

        result.append(ProjectionKernels(
            projection_level=img,
            depth=depth_idx,
            kernel_horizontal_size_m=kernel_horizontal_size,
            kernel_vertical_size_m=kernel_vertical_size,
            kernel_horizontal_pixels=kw,
            kernel_vertical_pixels=kh,
            kernels=kernels
        ))

    return result


def bundle_kernels_to_larger_verticals(
        projections: List[ProjectionKernels],
        target_vertical_size_m: float
) -> List[ProjectionKernels]:
    """
    Aggregate each ProjectionKernels’ small vertical kernels into taller bins
    of size `target_vertical_size_m`, updating activation and choosing the
    minimum depth among members.

    Parameters
    ----------
    projections : List[ProjectionKernels]
        The per‐depth projection + kernel bundles to re‐bin.
    target_vertical_size_m : float
        Desired vertical size (in meters) of each bundled kernel.

    Returns
    -------
    List[ProjectionKernels]
        New list of ProjectionKernels with updated `.kernel_vertical_size_m`,
        `.kernel_vertical_pixels`, and `.kernels` aggregated vertically.
    """
    bundled_projs: List[ProjectionKernels] = []

    for proj in projections:
        # how many small‐kernels stack to reach the target height?
        ratio = round(target_vertical_size_m / proj.kernel_vertical_size_m)
        if ratio <= 1:
            # no up‐binning needed
            bundled_projs.append(proj)
            continue

        # new pixel height of each bundled kernel
        new_kh_px = proj.kernel_vertical_pixels * ratio

        # group by (x position, vertical group index)
        groups: dict[tuple[int, int], List[Kernel]] = {}
        for k in proj.kernels:
            group_idx = k.y // new_kh_px
            key = (k.x, group_idx)
            groups.setdefault(key, []).append(k)

        # build the new, aggregated kernels
        new_kernels: List[Kernel] = []
        for (x, grp), members in groups.items():
            activated = any(k.activated for k in members)
            # pick the smallest‐depth among the group
            depth_val = min(k.depth for k in members)
            y_pos = grp * new_kh_px
            new_kernels.append(Kernel(
                x=x,
                y=y_pos,
                activated=activated,
                depth=depth_val
            ))

        # construct a new ProjectionKernels record
        bundled_projs.append(ProjectionKernels(
            projection_level=proj.projection_level,
            depth=proj.depth,
            kernel_horizontal_size_m=proj.kernel_horizontal_size_m,
            kernel_vertical_size_m=target_vertical_size_m,
            kernel_horizontal_pixels=proj.kernel_horizontal_pixels,
            kernel_vertical_pixels=new_kh_px,
            kernels=new_kernels
        ))

    return bundled_projs


def map_obstacles_graph(
        obs_graph: ObservationGraph,
        projections: List[ProjectionKernels]
) -> ObservationGraph:
    """
    Marks graph nodes as obstacles wherever a kernel is activated.

    Parameters
    ----------
    obs_graph : ObservationGraph
        The empty (or partially populated) graph to annotate.
    projections : List[ProjectionKernels]
        Each projection holds its pixel‐to‐meter sizes and a list of Kernel(x, y, activated, depth).

    Returns
    -------
    ObservationGraph
        The same graph instance, with `occupancy` flags set for each obstacle.
    """
    for proj in projections:
        # pre‐pull to avoid repeated attribute lookups
        kw_px = proj.kernel_horizontal_pixels
        kw_m = proj.kernel_horizontal_size_m
        spacing = obs_graph.node_spacing

        for kern in proj.kernels:
            if not kern.activated:
                continue

            # 1) Compute the kernel's center‐line X in meters
            kernel_x_m = round((kern.x / kw_px) * kw_m, 2)
            # 2) Depth (in meters) is already on the Kernel dataclass
            d0 = kern.depth - .4  # Test for a bugfixing hypothesis
            # 3) Compute graph coords for the front and back faces of this 1‐step‐thick obstacle
            depth_start, x_start = obs_graph.kernel_to_graph_coord(d0, kernel_x_m)
            depth_end, x_end = obs_graph.kernel_to_graph_coord(d0 + spacing, kernel_x_m + kw_m)

            x_start = min(x_start, x_end)
            x_end = max(x_start, x_end)

            # 4) Mark that rectangle in the graph
            obs_graph.set_obstacle(depth_start, depth_end, x_start, x_end)

    return obs_graph


def overlay_kernels_on_image(
        proj: ProjectionKernels,
        opacity: float = 0.5
) -> np.ndarray:
    """
    Overlay activated kernels (red boxes) onto the depth image stored in proj.

    Parameters
    ----------
    proj : ProjectionKernels
        Contains `projection_level` (HxW), pixel‐sized kernels, and their activation flags.
    opacity : float, default=0.5
        Blend factor for the red overlay (0.0–1.0).

    Returns
    -------
    np.ndarray
        RGB image with activated‐kernel regions tinted red.
    """
    alpha = float(np.clip(opacity, 0.0, 1.0))
    img = proj.projection_level * 10

    # ensure RGB
    if img.ndim == 2 or (img.ndim == 3 and img.shape[2] == 1):
        img_rgb = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_GRAY2RGB)
    else:
        img_rgb = img.copy()

    h, w = img_rgb.shape[:2]
    overlay = np.zeros_like(img_rgb)

    kw, kh = proj.kernel_horizontal_pixels, proj.kernel_vertical_pixels

    # draw red rectangles for each activated kernel
    for k in proj.kernels:
        if not k.activated:
            continue
        x0, y0 = k.x, k.y
        x1 = min(x0 + kw, w)
        y1 = min(y0 + kh, h)
        cv2.rectangle(overlay, (x0, y0), (x1, y1), (255, 0, 0), thickness=-1)

    mask = overlay[:, :, 0] > 0
    if not mask.any():
        return img_rgb

    blended = cv2.addWeighted(img_rgb, 1.0 - alpha, overlay, alpha, 0)
    # composite only the masked pixels
    result = img_rgb.copy()
    result[mask] = blended[mask]

    return result
