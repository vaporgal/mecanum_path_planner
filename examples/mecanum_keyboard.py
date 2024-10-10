

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})
from common_examples import cal_path
import omni
import carb

#from omni.isaac.jetbot import Jetbot
from omni.isaac.core import World
#from omni.isaac.jetbot.controllers import DifferentialController
import numpy as np


_command = [0.0, 0.0,0.0]

def _sub_keyboard_event(event, *args, **kwargs):
    global _command
    if (event.type == carb.input.KeyboardEventType.KEY_PRESS
        or event.type == carb.input.KeyboardEventType.KEY_REPEAT):
        if event.input == carb.input.KeyboardInput.W:
            _command = [0.5, 0.0, 0.0]
        if event.input == carb.input.KeyboardInput.S:
            _command = [-0.5,0, 0.0]
        if event.input == carb.input.KeyboardInput.A:
            _command = [0.0, 0.0, -0.5]
        if event.input == carb.input.KeyboardInput.D:
            _command = [0.0, 0.0, 0.5]
        if event.input == carb.input.KeyboardInput.U:
            _command = [0, 0.5, 0]
        if event.input == carb.input.KeyboardInput.O:
            _command = [0, -0.5, 0]
            # x,y,rotation
    if event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        _command = [0.0, 0.0, 0.0]

# subscribe to keyboard events
appwindow = omni.appwindow.get_default_app_window()
input = carb.input.acquire_input_interface()
input.subscribe_to_keyboard_events(appwindow.get_keyboard(), _sub_keyboard_event)

#my_world = World(stage_units_in_meters=0.01)
#---------------------------
import carb
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.physics_context.physics_context import PhysicsContext
from omni.isaac.core.objects import VisualCuboid, FixedCuboid
my_world = World(stage_units_in_meters=1.0)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
jetbot_asset_path = "omniverse://localhost/Projects/RBROS2/WheeledRobot/summit_xl_original.usd"
#jetbot_asset_path = "omniverse://localhost/Library/saver.usd"
add_reference_to_stage(usd_path=jetbot_asset_path, prim_path="/World/Summit")
#--------------------------------------------\
wheel_radius = np.array([ 0.127, 0.127, 0.127, 0.127 ])
wheel_positions = np.array([
            [0.229, 0.235, 0.11],
            [0.229, -0.235, 0.11],
            [-0.229, 0.235, 0.11],
            [-0.229, -0.235, 0.11],
])

wheel_orientations = np.array([
            # w, x, y, z
            [0.7071068, 0, 0, 0.7071068],
            [0.7071068, 0, 0, 0.7071068],
            [0.7071068, 0, 0, 0.7071068],
            [0.7071068, 0, 0, 0.7071068],
        ])
mecanum_angles = np.array([
            -135.0,
            -45.0,
            -45.0,
            -135.0,
        ])
wheel_axis = np.array([1, 0, 0])
up_axis = np.array([0, 0, 1])
#--------------------------------------------
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
        position=np.array([10.0, 5.0, 0.0]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
    )

)
scene = PhysicsContext()
scene.set_physics_dt(1 / 30.0)
print("111111111111111111111111111111111111111111",omni.isaac.core.__file__)



#--------------------------------墙壁
import numpy as np
wall_color = np.array([1.0, 0.0, 0.0])  # 红色
obstacle_points = set()
obstacle_points.update({(2, i) for i in range(16)})
obstacle_points.update({(i, 10) for i in range(10,16)})
obstacle_points.update({(i, i) for i in range(16)})
obstacle_points.update({(0,0)})
start_point = (6,2)
goal_point = (3,20)

def create_obstacles(my_world, wall_color,obstacle_points,wall_length):
    obstacles = []

    def add_obstacle(name, position, scale, angle=0, enabled=True):
        if enabled:
            # 将方位角转换为四元数，这里假设绕Z轴旋转

            qw = np.cos(angle / 2)
            qx = 0
            qy = 0
            qz = np.sin(angle / 2)
            orientation = (qw, qx, qy, qz)

            obstacle = FixedCuboid(
                prim_path=f"/World/Xform/{name}",
                name=name,
                position=position,
                scale=scale,
                orientation=orientation,
                color=wall_color
            )
            my_world.scene.add(obstacle)
            obstacles.append(obstacle)

    # 围墙尺寸
    wall_length = wall_length
    wall_thickness = 0.1
    wall_height = 1.5

    obstacle_size = 0.7  # 每个障碍物的尺寸，可以根据需要调整
    # 围墙
    # 底部
    add_obstacle("Wall_Bottom", [wall_length / 2, 0, 0], [wall_length, wall_thickness, wall_height])
    # 左侧
    add_obstacle("Wall_Left", [0, wall_length / 2, 0], [wall_thickness, wall_length, wall_height])
    # 顶部
    add_obstacle("Wall_Top", [wall_length / 2, wall_length, 0], [wall_length, wall_thickness, wall_height])
    # 右侧
    add_obstacle("Wall_Right", [wall_length, wall_length / 2, 0], [wall_thickness, wall_length, wall_height])
    #创建障碍物
    for idx, (x, y) in enumerate(obstacle_points):
        # 障碍物的中心位置需要根据障碍物尺寸进行调整
        position = [x + obstacle_size / 2, y + obstacle_size / 2, 0]
        scale = [obstacle_size, obstacle_size, wall_height]
        add_obstacle(f"Obstacle_{idx}", position, scale)

    return obstacles

# 使用示例

create_obstacles(my_world, wall_color, obstacle_points,wall_length=40)


#---------------------------
#my_jetbot = my_world.scene.add(Jetbot(prim_path="/World/Jetbot", name="my_jetbot", position=np.array([0, 0.0, 2.0])))
my_world.scene.add_default_ground_plane()
#my_controller = DifferentialController(name="simple_control", wheel_radius=0.03, wheel_base=0.1125)
my_controller = HolonomicController(
    name="holonomic_controller",
    wheel_radius=wheel_radius,
    wheel_positions=wheel_positions,
    wheel_orientations=wheel_orientations,
    mecanum_angles=mecanum_angles,
    wheel_axis=wheel_axis,
    up_axis=up_axis,
)
my_world.reset()
import math
import time

def quaternion_to_yaw(q):
    """
    将四元数转换为在xy平面上的yaw角度
    :param q: 四元数 [q_w, q_x, q_y, q_z]
    :return: xy平面上的yaw角度（以度为单位）
    """
    q_w, q_x, q_y, q_z = q
    # 计算yaw (航向角)
    yaw = math.atan2(2 * (q_w * q_z), 1 - 2 * (q_z**2))
    # 将弧度转换为度
    return math.degrees(yaw)


import math


def move_robot(start_x, start_y, start_z, end_x, end_y, end_z, my_jetbot, my_controller, angle_threshold=5, distance_threshold=0.5):
    # 确保 z 坐标为 0
    if start_z != 0 or end_z != 0:
        print("警告：z 坐标不为 0，将被忽略。")
    jetbot_position, jetbot_orientation = my_jetbot.get_world_pose()
    print(jetbot_position)
    dx = end_x - jetbot_position[0]
    dy = end_y - jetbot_position[1]
    distance = math.sqrt(dx ** 2 + dy ** 2)

    # 检查是否到达目标点
    if distance < distance_threshold:
        print(f"到达目标点: ({end_x}, {end_y}), 停止移动。")
        _command = [0, 0, 0]  # 可以根据需要调整速度
        my_jetbot.apply_wheel_actions(my_controller.forward(command=_command))
        return True  # 表示到达目标点

    target_angle = math.atan2(dy, dx)
    target_angle_degrees = math.degrees(target_angle)

    current_angle_degrees = quaternion_to_yaw(jetbot_orientation)
    angle_difference = target_angle_degrees - current_angle_degrees

    if abs(angle_difference) >= angle_threshold:
        # 旋转到目标角度
        rotation_speed = angle_difference / 10.0 * 0.5  # 将角度归一化并设置转速
        _command = [0.0, 0.0, -rotation_speed * 0.6]
        my_jetbot.apply_wheel_actions(my_controller.forward(command=_command))
        print(f"旋转中: 当前角度 = {current_angle_degrees}, 目标角度 = {target_angle_degrees}, 差异 = {angle_difference}")
        return False  # 继续旋转
    else:
        # 向前移动
        _command = [5, 0, 0]  # 可以根据需要调整速度
        my_jetbot.apply_wheel_actions(my_controller.forward(command=_command))
        print(f"向前移动，距离: {distance}")
        return False  # 继续前进

# 使用示例


def follow_path(path, my_jetbot, my_controller):
    current_target_index = 0

    while current_target_index < len(path):
        target = path[current_target_index]
        jetbot_position, jetbot_orientation = my_jetbot.get_world_pose()
        print("jetbot_position",jetbot_position)
        print('target_position',target)
        # 调用 move_robot 函数移动到当前目标点
        reached = move_robot(jetbot_position[0], jetbot_position[1], jetbot_position[2],
                             target[0], target[1], target[2],
                             my_jetbot, my_controller)

        if reached:
            print(f"到达第 {current_target_index + 1} 个目标点: {target}")
            current_target_index += 1

        yield  # 允许在每次迭代后返回控制权给主循环


# 定义路径点--------------------------------------------------------------
#original_points = [(110, 80), (110, 79), (109, 78), (109, 77), (108, 76), (108, 75), (108, 74), (108, 73), (107, 72), (106, 71), (106, 70), (106, 69), (106, 68), (106, 67), (106, 66), (106, 65), (106, 64), (106, 63), (106, 62), (105, 61), (104, 60), (103, 59), (102, 58), (101, 57), (100, 56), (99, 55), (98, 54), (97, 53), (96, 52), (95, 51), (94, 50), (93, 49), (92, 48), (91, 47), (90, 47), (89, 47), (88, 47), (87, 47), (86, 47), (85, 47), (84, 48), (83, 49), (82, 49), (81, 49), (80, 50), (79, 50), (78, 50), (77, 50), (76, 50), (75, 50), (74, 50), (73, 50), (72, 50), (71, 50), (70, 50), (69, 50), (68, 50), (67, 50), (66, 50), (65, 50), (64, 50), (63, 50), (62, 50), (61, 50), (60, 50)]

path = cal_path(40,40,start=start_point,goal=goal_point,obstacles=obstacle_points)

# 倒序输出


# 打印结果
print(path)



#-----------------------------------------------------------------------
# 创建路径跟随器
path_follower = follow_path(path, my_jetbot, my_controller)

# 主循环
while simulation_app.is_running():
    my_world.step(render=True)
    my_jetbot.apply_wheel_actions(my_controller.forward(command=_command))
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
            path_follower = follow_path(path, my_jetbot, my_controller)  # 重置路径跟随器

        try:
            next(path_follower)  # 执行路径跟随的下一步
        except StopIteration:
            print("路径已完成")
            # 可以在这里添加路径完成后的操作，比如停止模拟或重新开始


simulation_app.close()