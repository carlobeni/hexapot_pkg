import os

# ================= ROS DOMAIN =================
ROS_DOMAIN_ID = 42

def check_domain_id(logger=None):
    env = os.getenv("ROS_DOMAIN_ID")
    msg = (
        f"ROS_DOMAIN_ID={env} (expected {ROS_DOMAIN_ID})"
        if env else
        f"ROS_DOMAIN_ID NOT set (recommended {ROS_DOMAIN_ID})"
    )
    if logger:
        logger.warning(msg)
    else:
        print(msg)

# ================= TOPICS (FROM PI5) =================
# READY TO USE
TOPIC_IMU_GIR_ACC = "/pi/sensor/imu_data"
TOPIC_IMU_MAG = "/pi/sensor/imu_mag"
# IN DEVELOPMENT
TOPIC_GPS = "/pi/sensor/gps_fix"
TOPIC_CAMERA = "/pi/camera/image_raw"
TOPIC_IR = "/pi/sensor/IR_measure"
TOPIC_ULTRASONIC = "/pi/sensor/ultrasonic_read"

# ================= TOPICS (just for PC) =================
TOPIC_MONITOR = "/pc/system/monitor"  
TOPIC_HEADING_COMPASS    = "/pc/internal/heading_mag"     # heading solo magnetómetro
TOPIC_HEADING_COMPASS_KALMAN = "/pc/internal/heading_kalman"  # heading fusionado (PC)

# ================= COMMAND =================
TOPIC_CMD_SERIAL_MEGA = "/cmd_serial"



# ================= COMMAND =================
TOPIC_CMD_VEL_ROBOT = "/cmd_vel_robot"
TOPIC_CMD_SERIAL = "/cmd_serial"

