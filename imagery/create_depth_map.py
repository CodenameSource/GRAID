import cv2
import numpy as np


def compute_stereo_depth_map(left_image, right_image, baseline, focal_length, disparity_algorithm='BM'):
    """
    Compute the depth map from stereo images.

    Args:
        left_image (np.ndarray): The left stereo image (grayscale or color).
        right_image (np.ndarray): The right stereo image (grayscale or color).
        baseline (float): The distance between the two camera lenses (in meters).
        focal_length (float): The focal length of the camera (in pixels).
        disparity_algorithm (str): The algorithm to compute the disparity map, 'BM' (Block Matching)
                                   or 'SGBM' (Semi-Global Block Matching).

    Returns:
        np.ndarray: The depth map as a NumPy array (same resolution as the input images).
    """
    try:
        # Ensure the images are in grayscale
        if len(left_image.shape) == 3:
            left_image = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        if len(right_image.shape) == 3:
            right_image = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

        # Initialize the disparity algorithm
        if disparity_algorithm == 'BM':
            stereo = cv2.StereoBM.create(numDisparities=16 * 8, blockSize=15)
        elif disparity_algorithm == 'SGBM':
            stereo = cv2.StereoSGBM.create(
                minDisparity=0,
                numDisparities=16 * 8,  # Must be divisible by 16
                blockSize=5,
                P1=8 * 3 * 5 ** 2,
                P2=32 * 3 * 5 ** 2,
                disp12MaxDiff=1,
                uniquenessRatio=10,
                speckleWindowSize=100,
                speckleRange=32,
                preFilterCap=63,
                mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
            )
        else:
            raise ValueError("Unsupported disparity algorithm. Choose 'BM' or 'SGBM'.")

        # Compute the disparity map
        disparity = stereo.compute(left_image, right_image).astype(np.float32) / 16.0

        # Avoid division by zero for invalid disparity values
        disparity[disparity <= 0] = np.nan

        # Compute the depth map: Depth = (Baseline * Focal Length) / Disparity
        depth_map = (baseline * focal_length) / disparity

        return depth_map

    except Exception as e:
        print(f"Error computing depth map: {e}")
        return None
