import cosysairsim as airsim
from pymavlink import mavutil

from Smav.control import set_mode, takeoff, land, goto, get_location, set_param, await_armed, get_status, get_heartbeat
from Smav.util import Waypoint


class Drone:
    def __init__(self, host, port=5760, tcp=True, simulator_mode=False, simulator_address=''):
        if not tcp:
            raise NotImplementedError("Only TCP is supported")

        self.host = host
        self.port = port
        self.sim_mode = simulator_mode
        self.simulator_address = simulator_address
        self.simulator = None

        self.master = self.connect()

    def connect(self):
        if self.sim_mode and self.simulator_address:
            self.simulator = airsim.MultirotorClient(self.simulator_address)
            self.simulator.confirmConnection()
            self.simulator.enableApiControl(True)
            return None
        master = mavutil.mavlink_connection(f'tcp:{self.host}:{self.port}')
        heartbeat = master.wait_heartbeat(timeout=30)

        if heartbeat is None:
            raise ConnectionError("No heartbeat received")

        return master

    def disconnect(self):
        self.master.close()

    def reconnect(self):
        self.disconnect()
        self.master = self.connect()

    def arm(self, tries=3, timeout=10):
        if self.sim_mode and self.simulator:
            self.simulator.armDisarm(True)
            return

        for i in range(tries):
            self.master.arducopter_arm()
            armed = await_armed(self.master, timeout=timeout,
                                override=self.sim_mode)
            if armed:
                return
            else:
                print(f"Attempt {i + 1} to arm failed")

    def set_mode(self, mode: str):
        set_mode(self.master, mode)

    def takeoff(self, alt: int):
        if self.sim_mode and self.simulator:
            self.simulator.armDisarm(True)
            self.simulator.takeoffAsync().join()

            current_location = self.get_location()
            self.simulator.moveToGPSAsync(current_location['lat'], current_location['lon'],
                                          current_location['alt'] + alt, 1).join()
            return

        takeoff(self.master, alt)

    def land_bellow(self):
        if self.sim_mode and self.simulator:
            self.simulator.landAsync().join()
            return

        current_location = self.get_location()
        lat = current_location['lat']
        lon = current_location['lon']
        landing_alt = current_location['alt'] - current_location['relative_alt'] + .05

        land(self.master, lat, lon, landing_alt)

    def goto(self, waypoint: Waypoint, hold_time: int = 2, velocity: int = 1):
        if self.sim_mode and self.simulator:
            self.simulator.moveToGPSAsync(waypoint.lat, waypoint.lon, waypoint.alt, velocity)
            return
        goto(self.master, waypoint.lat, waypoint.lon, waypoint.alt, hold_time=hold_time)

    def get_location(self):
        if self.sim_mode and self.simulator:
            vehicle_state = self.simulator.getMultirotorState()
            gps_data = vehicle_state.gps_location
            hdg = vehicle_state.rc_data.yaw
            relative_alt = vehicle_state.kinematics_estimated.position.z_val
            linear_velocity = vehicle_state.kinematics_estimated.linear_velocity

            return {'lat': gps_data.latitude, 'lon': gps_data.longitude, 'alt': gps_data.altitude,
                    'relative_alt': relative_alt, 'vx': linear_velocity.x_val, 'vy': linear_velocity.y_val,
                    'vz': linear_velocity.z_val, 'hdg': hdg}

        return get_location(self.master)

    def get_status(self):
        return get_status(self.master)

    def get_heartbeat(self):
        return get_heartbeat(self.master)

    def remove_arming_check(self):
        set_param(self.master, 'ARMING_CHECK', 0, 'uint8_t')
