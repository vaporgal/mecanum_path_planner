from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.wheeled_robots.controllers.holonomic_controller import HolonomicController
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.core.objects import FixedCuboid
import omni.appwindow
import carb
import numpy as np
import math

class SimulationEnvironment:
    def __init__(self):

        self.world = World(stage_units_in_meters=1.0)
        self.setup_scene()


    def setup_scene(self):
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
        jetbot_asset_path = "omniverse://localhost/Projects/RBROS2/WheeledRobot/summit_xl_original.usd"
        add_reference_to_stage(usd_path=jetbot_asset_path, prim_path="/World/Summit")
        self.world.scene.add_default_ground_plane()

class Robot:
    def __init__(self, world):
        self.world = world
        self.setup_robot()

    def setup_robot(self):
        wheel_radius = np.array([0.127, 0.127, 0.127, 0.127])
        wheel_positions = np.array([
            [0.229, 0.235, 0.11],
            [0.229, -0.235, 0.11],
            [-0.229, 0.235, 0.11],
            [-0.229, -0.235, 0.11],
        ])
        wheel_orientations = np.array([
            [0.7071068, 0, 0, 0.7071068],
            [0.7071068, 0, 0, 0.7071068],
            [0.7071068, 0, 0, 0.7071068],
            [0.7071068, 0, 0, 0.7071068],
        ])
        mecanum_angles = np.array([-135.0, -45.0, -45.0, -135.0])
        wheel_axis = np.array([1, 0, 0])
        up_axis = np.array([0, 0, 1])

        self.robot = self.world.scene.add(
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
                usd_path="omniverse://localhost/Projects/RBROS2/WheeledRobot/summit_xl_original.usd",
                position=np.array([10.0, 5.0, 0.0]),
                orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            )
        )

        self.controller = HolonomicController(
            name="holonomic_controller",
            wheel_radius=wheel_radius,
            wheel_positions=wheel_positions,
            wheel_orientations=wheel_orientations,
            mecanum_angles=mecanum_angles,
            wheel_axis=wheel_axis,
            up_axis=up_axis,
        )

    def move(self, command):
        self.robot.apply_wheel_actions(self.controller.forward(command=command))

    def get_pose(self):
        return self.robot.get_world_pose()

class ObstacleManager:
    def __init__(self, world):
        self.world = world
        self.wall_color = np.array([1.0, 0.0, 0.0])
        self.obstacle_points = set()
        self.setup_obstacles()

    def setup_obstacles(self):
        self.obstacle_points.update({(2, i) for i in range(16)})
        self.obstacle_points.update({(i, 10) for i in range(10, 16)})
        self.obstacle_points.update({(i, i) for i in range(16)})
        self.obstacle_points.update({(0, 0)})
        self.create_obstacles(40)

    def create_obstacles(self, wall_length):
        wall_thickness = 0.1
        wall_height = 1.5
        obstacle_size = 0.7

        def add_obstacle(name, position, scale, angle=0):
            qw = np.cos(angle / 2)
            qz = np.sin(angle / 2)
            orientation = (qw, 0, 0, qz)
            obstacle = FixedCuboid(
                prim_path=f"/World/Xform/{name}",
                name=name,
                position=position,
                scale=scale,
                orientation=orientation,
                color=self.wall_color
            )
            self.world.scene.add(obstacle)

        # Add walls
        add_obstacle("Wall_Bottom", [wall_length / 2, 0, 0], [wall_length, wall_thickness, wall_height])
        add_obstacle("Wall_Left", [0, wall_length / 2, 0], [wall_thickness, wall_length, wall_height])
        add_obstacle("Wall_Top", [wall_length / 2, wall_length, 0], [wall_length, wall_thickness, wall_height])
        add_obstacle("Wall_Right", [wall_length, wall_length / 2, 0], [wall_thickness, wall_length, wall_height])

        # Add obstacles
        for idx, (x, y) in enumerate(self.obstacle_points):
            position = [x + obstacle_size / 2, y + obstacle_size / 2, 0]
            scale = [obstacle_size, obstacle_size, wall_height]
            add_obstacle(f"Obstacle_{idx}", position, scale)

class PathPlanner:
    def __init__(self, start_point, goal_point, obstacle_points):
        self.start_point = start_point
        self.goal_point = goal_point
        self.obstacle_points = obstacle_points

    def plan_path(self):
        # Placeholder for path planning algorithm
        # For now, we'll just return a simple path
        return [self.start_point, (5, 5), (10, 10), self.goal_point]

class Simulation:
    def __init__(self):
        self.env = SimulationEnvironment()
        self.robot = Robot(self.env.world)
        self.obstacle_manager = ObstacleManager(self.env.world)
        self.path_planner = PathPlanner((6, 2), (3, 20), self.obstacle_manager.obstacle_points)
        self.path = self.path_planner.plan_path()
        self.current_target_index = 0

    def run(self):
        while simulation_app.is_running():
            self.env.world.step(render=True)

            if self.env.world.is_playing():
                if self.env.world.current_time_step_index == 0:
                    self.env.world.reset()
                    self.robot.controller.reset()
                    self.current_target_index = 0

                if self.current_target_index < len(self.path):
                    target = self.path[self.current_target_index]
                    reached = self.move_robot_to_target(target)
                    if reached:
                        print(f"Reached target {self.current_target_index + 1}: {target}")
                        self.current_target_index += 1
                else:
                    print("Path completed")

        self.env.simulation_app.close()

    def move_robot_to_target(self, target, angle_threshold=5, distance_threshold=0.5):
        robot_position, robot_orientation = self.robot.get_pose()
        dx = target[0] - robot_position[0]
        dy = target[1] - robot_position[1]
        distance = math.sqrt(dx ** 2 + dy ** 2)

        if distance < distance_threshold:
            self.robot.move([0, 0, 0])
            return True

        target_angle = math.degrees(math.atan2(dy, dx))
        current_angle = self.quaternion_to_yaw(robot_orientation)
        angle_difference = target_angle - current_angle

        if abs(angle_difference) >= angle_threshold:
            rotation_speed = angle_difference / 10.0 * 0.5
            self.robot.move([0.0, 0.0, -rotation_speed * 0.6])
            return False
        else:
            self.robot.move([5, 0, 0])
            return False

    @staticmethod
    def quaternion_to_yaw(q):
        return math.degrees(math.atan2(2 * (q[0] * q[3]), 1 - 2 * (q[3]**2)))

if __name__ == "__main__":
    simulation = Simulation()
    simulation.run()