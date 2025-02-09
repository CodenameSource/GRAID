from time import sleep

from pymavlink import mavutil

from .systems import get_location, calculate_location_delta


# TODO Improve documentation & styling

def takeoff(master, altitude: int):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )


def land(master, lat, lon, altitude):
    abort_alt = 5
    land_mode = 0  # (non precision landing)
    yaw_angle = 0
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        abort_alt,
        land_mode,
        0,
        yaw_angle,
        lat,
        lon,
        altitude
    )


def goto(master, lat: int, lon: int, altitude: int, hold_time: int = 2, near_threshold_meters: float = 1.5):
    cont = 3
    coordinate_frame = 0
    yaw_angle = 0
    # lat_int = c_int32(int(lat * 1e7)).value
    # lon_int = c_int32(int(lon * 1e7)).value
    # master.mav.command_long_send(
    #    master.target_system, master.target_component,
    #    mavutil.mavlink.MAV_CMD_OVERRIDE_GOTO,
    #    0,
    #    mavutil.mavlink.MAV_GOTO_DO_HOLD,
    #    mavutil.mavlink.MAV_GOTO_HOLD_AT_SPECIFIED_POSITION,
    #    coordinate_frame,
    #    yaw_angle,
    #    lat,
    #    lon,
    #    altitude,
    # )

    master.mav.mission_item_send(master.target_system, master.target_component, 0, coordinate_frame,
                                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                 2, 0,
                                 0, 0, 0,
                                 yaw_angle, lat, lon, altitude)

    near_waypoint = False

    while not near_waypoint:
        current_location = get_location(master)

        distance_to_waypoint = calculate_location_delta(current_location['lat'], current_location['lon'],
                                                        lat, lon)

        if distance_to_waypoint < near_threshold_meters:
            break

        print("Distance to waypoint: ", distance_to_waypoint)

        sleep(2)

    sleep(hold_time)
