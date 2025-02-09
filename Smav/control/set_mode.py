from time import time

from pymavlink import mavutil


def get_mode_id(master, mode_name: str) -> int:
    # Check if mode is available
    if mode_name not in master.mode_mapping():
        print('Unknown mode : {}'.format(mode_name))
        print('Try:', list(master.mode_mapping().keys()))
        return -1

    return master.mode_mapping()[mode_name]


def set_mode(master, mode_name: str) -> None:
    mode_id = get_mode_id(master, mode_name)
    if mode_id == -1:
        return

    # Set new mode
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    ack_wait_start = time()
    while True:
        if time() - ack_wait_start > 10:  # TODO add timeout as parameter
            print('Timeout setting mode or no response from autopilot')
            break

        # Wait for ACK command
        # Would be good to add mechanism to avoid endlessly blocking
        # if the autopilot sends a NACK or never receives the message
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=False)
        if ack_msg is None:
            continue
        ack_msg = ack_msg.to_dict()

        # Continue waiting if the acknowledged command is not `set_mode`
        if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue

        # Print the ACK result !
        print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break
