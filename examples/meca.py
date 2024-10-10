from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import carb
import numpy as np
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.objects import FixedCuboid
from omni.isaac.core.physics_context import PhysicsContext
from common_examples import cal_path

# Global variables
_command = [0.0, 0.0, 0.0]

def _sub_keyboard_event(event, *args, **kwargs):
    global _command
    key_to_command = {
        carb.input.KeyboardInput.W: [0.5, 0.0, 0.0],
        carb.input.KeyboardInput.S: [-0.5, 0.0, 0.0],
        carb.input.KeyboardInput.A: [0.0, 0.0, -0.5],
        carb.input.KeyboardInput.D: [0.0, 0.0, 0.5],
        carb.input.KeyboardInput.U: [0.0, 0.5, 0.0],
        carb.input.KeyboardInput.O: [0.0, -0.5, 0.0]
    }
    if event.type in [carb.input.KeyboardEventType.KEY_PRESS, carb.input.KeyboardEventType.KEY_REPEAT]:
        _command = key_to_command.get(event.input, [0.0, 0.0, 0.0])
    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        _command = [0.0, 0.0, 0.0]

# Setup world and robot
my_world = World(stage_units_in_meters=1.0)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
jetbot_asset_path = "omniverse://localhost/Projects/RBROS2/WheeledRobot/summit_xl_original.usd"
add_reference_to_stage(usd_path=jetbot_asset_path, prim_path="/World/Summit")

# Robot configuration
wheel_radius = np.array([0.127] * 4)
wheel_positions = np.array([
    [0.229, 0.235, 0.11],
    [0.229, -0.235, 0.11],
    [-0.229, 0.235, 0.11],
    [-0.229, -0.235, 0.11]
])
wheel_orientations = np.array([[0.7071068, 0, 0, 0.7071068]] * 4)
mecanum_angles = np.array([-135.0, -45.0, -45.0, -135.0])

my_jetbot = my_world.scene.add(
    WheeledRobot(
        prim_path="/World/Summit/summit_xl_base_link",
        name="my_jetbot",
        wheel_dof_names=[
            "summit_xl_front_left_wheel_joint",
            "summit_xl_front_right_wheel_joint",
            "summit_xl_back_left_wheel_joint",
            "summit_xl_back_right_wheel_joint",
        ],
        create_robot=True,
        usd_path=jetbot_asset_path,
        position=np.array([10.0, 5.0, 2.0]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    )
)

my_world.scene.add_default_ground_plane()
my_controller = HolonomicController(
    name="holonomic_controller",
    wheel_radius=wheel_radius,
    wheel_positions=wheel_positions,
    wheel_orientations=wheel_orientations,
    mecanum_angles=mecanum_angles,
    wheel_axis=np.array([1, 0, 0]),
    up_axis=np.array([0, 0, 1])
)

# Set physics properties
scene = PhysicsContext()
scene.set_physics_dt(1 / 30.0)

# Obstacle setup
wall_color = np.array([1.0, 0.0, 0.0])
obstacle_points = set()
obstacle_points.update({(2, i) for i in range(16)})
obstacle_points.update({(i, 10) for i in range(10, 16)})
obstacle_points.update({(i, 15) for i in range(10, 16)})
obstacle_points.update({(i, i) for i in range(16)})
obstacle_points.add((0, 0))
start_point, goal_point = (6, 2), (3, 20)

def create_obstacles(my_world, wall_color, obstacle_points, wall_length=40):
    def add_obstacle(name, position, scale, angle=0):
        orientation = (np.cos(angle / 2), 0, 0, np.sin(angle / 2))
        obstacle = FixedCuboid(
            prim_path=f"/World/Xform/{name}",
            name=name,
            position=position,
            scale=scale,
            orientation=orientation,
            color=wall_color
        )
        my_world.scene.add(obstacle)

    wall_thickness, wall_height = 0.1, 1.5
    obstacle_size = 0.7

    # Create walls
    for wall in ["Bottom", "Left", "Top", "Right"]:
        if wall in ["Bottom", "Top"]:
            position = [wall_length / 2, wall_length if wall == "Top" else 0, 0]
            scale = [wall_length, wall_thickness, wall_height]
        else:
            position = [wall_length if wall == "Right" else 0, wall_length / 2, 0]
            scale = [wall_thickness, wall_length, wall_height]
        add_obstacle(f"Wall_{wall}", position, scale)

    # Create obstacles
    for idx, (x, y) in enumerate(obstacle_points):
        add_obstacle(f"Obstacle_{idx}", [x + obstacle_size / 2, y + obstacle_size / 2, 0], [obstacle_size] * 2 + [wall_height])

create_obstacles(my_world, wall_color, obstacle_points)

# Robot movement functions
def quaternion_to_yaw(q):
    return np.degrees(np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2]**2 + q[3]**2)))

def move_robot(end_x, end_y, my_jetbot, my_controller, angle_threshold=5, distance_threshold=0.5):
    jetbot_position, jetbot_orientation = my_jetbot.get_world_pose()
    dx, dy = end_x - jetbot_position[0], end_y - jetbot_position[1]
    distance = np.sqrt(dx**2 + dy**2)

    if distance < distance_threshold:
        my_jetbot.apply_wheel_actions(my_controller.forward(command=[0, 0, 0]))
        print(f"Reached target point: ({end_x}, {end_y})")
        return True

    target_angle = np.degrees(np.arctan2(dy, dx))
    current_angle = quaternion_to_yaw(jetbot_orientation)
    angle_difference = target_angle - current_angle

    if abs(angle_difference) >= angle_threshold:
        rotation_speed = angle_difference / 10.0 * 0.5
        my_jetbot.apply_wheel_actions(my_controller.forward(command=[1, 0.0, -rotation_speed * 0.3]))
        print(f"Rotating: Current angle = {current_angle}, Target angle = {target_angle}, Difference = {angle_difference}")
    else:
        my_jetbot.apply_wheel_actions(my_controller.forward(command=[10, 0, 0]))
        print(f"Moving forward, Distance: {distance}")
    return False

def follow_path(path, my_jetbot, my_controller):
    current_target_index = 0
    while current_target_index < len(path):
        target = path[current_target_index]
        if move_robot(target[0], target[1], my_jetbot, my_controller):
            print(f"Reached point {current_target_index + 1}: {target}")
            current_target_index += 1
        yield

# Main simulation loop
path = cal_path(40, 40, start=start_point, goal=goal_point, obstacles=obstacle_points)
print("Calculated path:", path)
path_follower = follow_path(path, my_jetbot, my_controller)

# Subscribe to keyboard events
appwindow = omni.appwindow.get_default_app_window()
input_interface = carb.input.acquire_input_interface()
input_interface.subscribe_to_keyboard_events(appwindow.get_keyboard(), _sub_keyboard_event)

my_world.reset()

while simulation_app.is_running():
    my_world.step(render=True)
    my_jetbot.apply_wheel_actions(my_controller.forward(command=_command))
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
            path_follower = follow_path(path, my_jetbot, my_controller)
        try:
            next(path_follower)
        except StopIteration:
            print("Path completed")

simulation_app.close()