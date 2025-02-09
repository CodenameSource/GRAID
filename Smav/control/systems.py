import math
from time import time, sleep

from pymavlink import mavutil


def request_single_message(master, message_type, message_string):
    # TODO add check if message_type is valid
    # TODO add control for interval
    # TODO add parameters for message, maybe in a dictionary
    # TODO add wait time for response

    wait_time = 6  # in seconds

    message = master.mav.command_long_encode(
        master.target_system,  # Target system ID
        master.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # ID of command to send
        0,
        message_type,  # param1: Message ID to be streamed
        1000000,  # param2: Interval in microseconds
        0,  # param3 (unused)
        0,  # param4 (unused)
        0,  # param5 (unused)
        0,  # param5 (unused)
        0,  # param6 (unused)
    )

    # Send the COMMAND_LONG
    master.mav.send(message)

    start_countdown = time()
    response = None
    while True:
        if time() - start_countdown > wait_time:
            print('Timeout waiting for response')
            break
        sleep(1)
        response = master.recv_match(type=message_string, blocking=False)
        if response is None:
            continue
        break

    return response


def set_param(master, param_id, param_value, type=None):
    """
    Set a parameter on the drone
    :param master: pymavlink object
    :param param_id: string
    :param param_value: float
    :param type: string (optional) - type of the parameter, possible values are:
        - uint8_t
        - int8_t
        - uint16_t
        - int16_t
        - uint32_t
        - int32_t
        - uint64_t
        - int64_t
        - real32_t
        - real64_t
    :return: None
    """

    types = {
        'uint8_t': mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
        'int8_t': mavutil.mavlink.MAV_PARAM_TYPE_INT8,
        'uint16_t': mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
        'int16_t': mavutil.mavlink.MAV_PARAM_TYPE_INT16,
        'uint32_t': mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
        'int32_t': mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        'uint64_t': mavutil.mavlink.MAV_PARAM_TYPE_UINT64,
        'int64_t': mavutil.mavlink.MAV_PARAM_TYPE_INT64,
        'real32_t': mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        'real64_t': mavutil.mavlink.MAV_PARAM_TYPE_REAL64,
    }

    master.param_set_send(param_id, param_value, types[type] if type is not None else None)


def await_armed(master, timeout=10, override=False):
    if override:
        sleep(5)
        return True
    start_time = time()
    while True:
        if time() - start_time > timeout:
            print('Timeout waiting for arming')
            return False
        sleep(1)
        if master.motors_armed():
            return True


def get_location(master):
    raw_response = request_single_message(master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                                          'GLOBAL_POSITION_INT')

    if raw_response is None:
        return None

    response = raw_response.to_dict()

    response['lat'] = response['lat'] / 1e7
    response['lon'] = response['lon'] / 1e7
    response['alt'] = response['alt'] / 1e3
    response['relative_alt'] = response['relative_alt'] / 1e3
    response['vx'] = response['vx'] / 1e2
    response['vy'] = response['vy'] / 1e2
    response['vz'] = response['vz'] / 1e2
    response['hdg'] = response['hdg'] / 1e2

    return response


def get_status(master):
    raw_response = request_single_message(master, mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 'SYS_STATUS')

    if raw_response is None:
        return None

    response = raw_response.to_dict()

    response['voltage_battery'] = response['voltage_battery'] / 1000
    response['current_battery'] = response['current_battery'] / 100
    response['battery_remaining'] = response['battery_remaining']

    return response


def get_heartbeat(master):
    raw_response = request_single_message(master, mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 'HEARTBEAT')

    if raw_response is None:
        return None

    response = raw_response.to_dict()

    return response


def calculate_location_delta(lat1, lon1, lat2, lon2):
    # Adapted from https://stackoverflow.com/questions/639695/how-to-convert-latitude-or-longitude-to-meters
    # TODO rewrite in pythonic way

    earth_radius = 6378.137  # Radius of earth in KM
    delta_lat = lat2 * math.pi / 180 - lat1 * math.pi / 180
    delta_lon = lon2 * math.pi / 180 - lon1 * math.pi / 180

    a = math.sin(delta_lat / 2) * math.sin(delta_lat / 2) + \
        math.cos(lat1 * math.pi / 180) * math.cos(lat2 * math.pi / 180) * \
        math.sin(delta_lon / 2) * math.sin(delta_lon / 2)

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = earth_radius * c
    return d * 1000
