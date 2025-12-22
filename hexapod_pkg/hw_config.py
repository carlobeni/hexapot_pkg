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

# ================= DDS TOPICS =================
# TOPICS (From Pi)
TOPIC_PI_IMU_GIR_ACC = "/pi/sensor/imu_data" 
TOPIC_PI_IMU_MAG = "/pi/sensor/imu_mag"
TOPIC_PI_GPS = "/pi/sensor/gps_fix"
TOPIC_PI_CAMERA = "/pi/camera/image_raw"
TOPIC_PI_IR1 = "/pi/sensor/IR_measure1"
TOPIC_PI_IR2 = "/pi/sensor/IR_measure2"
TOPIC_PI_ULTRASONIC = "/pi/sensor/ultrasonic_read"

TOPIC_PI_PHONE_HEADING = "/pi/phone/sensor/heading" # Topic type: Float32
TOPIC_PI_PHONE_CAMERA = "/pi/phone/camera/image_raw" # Topic type: Image
TOPIC_PI_PHONE_GPS = "/pi/phone/sensor/gps_fix" # Topic type: NavSatFix
# In development
TOPIC_PI_PHONE_IMU_GIR_ACC = "/pi/phone/sensor/imu_data" # Topic type: Imu
TOPIC_PI_PHONE_MAG = "/pi/phone/sensor/imu_mag" # Topic type: MagneticField

# TOPICS (From Gazebo)
TOPIC_GZ_IMU_GIR_ACC = "/gz/sensor/imu_data"
TOPIC_GZ_IMU_MAG = "/gz/sensor/imu_mag"
TOPIC_GZ_GPS = "/gz/sensor/gps_fix"
TOPIC_GZ_CAMERA = "/gz/camera/image_raw"
TOPIC_GZ_IR1 = "/gz/sensor/IR_measure1"
TOPIC_GZ_IR2 = "/gz/sensor/IR_measure2"
TOPIC_GZ_ULTRASONIC = "/gz/sensor/ultrasonic_read"

# ================= LOCALIZATION NODES =================
TOPIC_GPS_TO_XY = "/localization/gps_to_xy"
TOPIC_ESTIMATE_XY = "/localization/estimate_xy"
TOPIC_ESTIMATE_HEADING = "/localization/heading"

# ================= ULTRASONIC MANAGER =================
TOPIC_ULTRASONIC_RANGE = "/ultrasonic_range"

# ================= IMAGE RECOGNITION =================


# ================= TOPICS (just for PC) =================
TOPIC_MONITOR = "/pc/system/monitor"  

TOPIC_XY_BY_GPS_CURRENT_POSITION = "/pc/internal/xy_gps_current_position"
TOPIC_XY_ODM_CURRENT_POSITION = "/pc/internal/xy_odom_current_position"
TOPIC_HEADING_COMPASS    = "/pc/internal/heading_mag"     # heading solo magnetómetro
TOPIC_HEADING_COMPASS_KALMAN = "/pc/internal/heading_kalman"  # heading fusionado (PC)

# ================= COMMAND =================
TOPIC_CMD_REAL_ROBOT = "/hl_cmd" 
TOPIC_CMD_GZ_ROBOT = "/hl_cmd"
# TOPIC CMD SERIAL DE SALIDA
TOPIC_CMD_SERIAL = "/cmd_serial"

