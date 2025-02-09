import cv2


def collect_image_from_udp_stream(port, width=640, height=480):
    """
    Captures frames from a UDP stream using GStreamer and OpenCV.

    Args:
        port (int): The UDP port where the stream is being received.
        width (int): The desired width of the output frames (default: 640).
        height (int): The desired height of the output frames (default: 480).

    Returns:
        np.ndarray or None: The captured frame as a NumPy array, or None if an error occurs.
    """
    try:
        gstreamer_str = (
            f"udpsrc port={port} ! application/x-rtp,payload=96 ! "
            f"rtph264depay ! avdec_h264 discard-corrupted-frames=true ! "
            f"videoconvert ! video/x-raw,width={width},height={height},format=BGR ! appsink"
        )

        cap = cv2.VideoCapture(gstreamer_str, cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            raise RuntimeError("Failed to open video stream with GStreamer.")

        ret, frame = cap.read()

        cap.release()

        if ret:
            return frame
        else:
            raise RuntimeError("Failed to read frame from the UDP stream.")
    except Exception as e:
        print(f"Error capturing frame from GStreamer: {e}")
        return None
