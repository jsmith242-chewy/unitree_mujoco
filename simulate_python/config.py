ROBOT = "g1" # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1" 
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene.xml" # Robot scene
DOMAIN_ID = 1 # Domain id
INTERFACE = "lo" # Interface 

USE_JOYSTICK = 1 # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

PRINT_SCENE_INFORMATION = True # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = True # Virtual spring band, used for lifting h1

SIMULATE_DT = 0.005  # Need to be larger than the runtime of viewer.sync()
VIEWER_DT = 0.02  # 50 fps for viewer

# Camera settings
HEAD_CAMERA_ENABLE = True # Enable head camera
HEAD_CAMERA_WIDTH = 640 # Head camera width
HEAD_CAMERA_HEIGHT = 480 # Head camera height
HEAD_CAMERA_FPS = 30 # Head camera fps
HEAD_CAMERA_IMAGE_SERVER = True # Enable head camera image server (to publish ZMQ message for use with XR Teleoperate)
