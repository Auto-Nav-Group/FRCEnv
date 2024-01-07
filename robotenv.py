from utils import Quaternion
from frc_map import Map
import torch
import pybullet as p
import numpy as np
import time
import math

TIME_DELTA = 0.1 # Time setup in simulation
GUI = False # GUI flag
GOAL_REACHED_DIST = 1 # Distance to goal to be considered reached
MIN_START_DIST = 4 # Minimum distance from start to goal
MAX_SPEED = 5 # Maximum speed of the robot
MAX_ANGULAR_SPEED = math.pi # Maximum angular speed of the robot
TIP_ANGLE = 30

LIDAR_RANGE = 10
LIDAR_ANGLE = 2*np.pi
LIDAR_POINTS = 100

DEBUG_CIRCLE_COUNT = 100
DEBUG_CIRCLE_INCREMENT = np.pi*2/DEBUG_CIRCLE_COUNT

GRAVITY = 0

SPAWN_BORDER = 2

class RobotVEnv:
    def __init__(self, field : Map, assets_path):
        self.basis = field
        self.robot = None
        self.goal = None
        self.ray_debug_id = []
        self.goal_x = 5
        self.goal_y = 1.5
        self.start_x = -5
        self.start_y = -1.5
        self.start_angle = 0#np.pi/2
        self.x = 0
        self.y = 0
        self.init_x = 0
        self.init_y = 0
        self.obstacles = []
        self.lidar_dists = []
        self.pybullet_instance = p
        self.debug_start_angle = 0
        if GUI:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(assets_path)
        p.setGravity(0, 0, -GRAVITY)
        p.setTimeStep(0.01)
        p.setTimeStep(TIME_DELTA)
        self.environment_ids = []
        self.environment_dim = 0
        self.robotid = 0
        self.target = [0,0,0,0,0,0,0,1,0,0]
        self.max_mes = math.sqrt(self.basis.size.width**2+ self.basis.size.height**2)

    def step(self, action):
        target = False
        done = False
        achieved_goal = False

        p.stepSimulation()

        if GUI:
            time.sleep(TIME_DELTA)

        position, quaternion = p.getBasePositionAndOrientation(self.robotid)
        if position[2] < -1:
            p.resetBasePositionAndOrientation(self.robotid, [position[0], position[1], 0], quaternion)
        if position[0] < -self.basis.size.width / 2 or position[0] > self.basis.size.width / 2 or position[
            1] < -self.basis.size.height / 2 or position[1] > self.basis.size.height / 2:
            p.resetBasePositionAndOrientation(self.robotid, [0, 0, position[2]], quaternion)
            done = True
        if math.isnan(position[0]) or math.isnan(position[1]) or math.isnan(position[2]):
            p.resetBasePositionAndOrientation(self.robotid, [0, 0, position[2]], quaternion)
            done = True
        p.resetBasePositionAndOrientation(self.robotid, [position[0], position[1], 0.25], quaternion)

        dist_traveled = math.sqrt((position[0] - self.x) ** 2 + (position[1] - self.y) ** 2)

        self.x = position[0]
        self.y = position[1]
        distance = math.sqrt((self.goal_x - self.x) ** 2 + (self.goal_y - self.y) ** 2)
        q = Quaternion()
        q.define(quaternion[3], quaternion[0], quaternion[1], quaternion[2])
        quaternion = q
        roll, pitch, yaw = quaternion.to_euler()

        if type(action) == torch.Tensor:
            action = action.cpu().detach().numpy()

        rotation_yaw = action[0] * float(MAX_ANGULAR_SPEED)

        action_x = action[1] * MAX_SPEED * math.cos(yaw)
        action_y = action[1] * MAX_SPEED * math.sin(yaw)

        p.resetBaseVelocity(self.robotid, [action_x, action_y, 0], [0, 0, rotation_yaw])

        collision = self._get_collisions()

        if roll > math.radians(TIP_ANGLE) or roll < math.radians(-TIP_ANGLE) or pitch > math.radians(
                TIP_ANGLE) or pitch < math.radians(-TIP_ANGLE):
            collision = True

        try:
            angle = round(yaw)
        except Exception as e:
            angle = 0

        distance = np.linalg.norm(
            [self.x - self.goal_x, self.y - self.goal_y]
        )

        # Calculate the relative angle between the robots heading and heading toward the goal
        skew_x = self.goal_x - self.x
        skew_y = self.goal_y - self.y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        try:
            beta = math.acos(dot / (mag1 * mag2))
        except:
            print("Divide by zero error in beta calculation")
            beta = 0
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - angle
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        # Detect if the goal has been reached and give a large positive reward
        if distance < GOAL_REACHED_DIST:
            achieved_goal = True
            done = True
        if collision is True:
            done = True
        robot_state = [theta, distance, self.x, self.y, self.goal_x, self.goal_y, action[0], action[1], self.init_x, self.init_y]
        # reward = self.get_reward(target, collision, action)
        # return robot_state, reward, done, target
        return robot_state, collision, done, achieved_goal, dist_traveled, self._run_lidar()

    def reset(self, reload=False, angle=-1, load_state=None): # Create a new environment
        p.resetSimulation()
        p.setGravity(0, 0, -GRAVITY)
        p.setTimeStep(0.01)
        p.setTimeStep(TIME_DELTA)
        self.environment_ids = []
        self.environment_dim = 0

        self.floor = p.loadURDF("floor.urdf")
        self.environment_ids.append(self.floor)
        for i in range(4):
            self.environment_ids.append(p.loadURDF("bound"+str(i+1)+".urdf"))
        for i in range(len(self.basis.obstacles)):
            self.obstacles.append(p.loadURDF("obs_"+str(i+1)+".urdf"))
            self.environment_ids.append(self.obstacles[i])
        for i in range(len(self.environment_ids)):
            self.environment_dim += p.getNumJoints(self.environment_ids[i])

        if load_state is not None:
            self.goal_x = load_state[4]
            self.goal_y = load_state[5]
            self.x = load_state[2]
            self.y = load_state[3]
            self.start_angle = load_state[0]
            self._reset_situation(np.pi, new_goal=False, new_angle=False, new_start=False)
        elif reload:
            if angle != -1:
                self._reset_situation(np.pi, new_goal=False, new_angle=True, new_start=False, set_angle=angle)
            else:
                self._reset_situation(np.pi, new_goal=False, new_angle=False, new_start=False)
        else:
            self._reset_situation(np.pi)



        quaternion = Quaternion.from_euler(0, 0, self.start_angle)

        position = [self.x, self.y, 0.25]
        orientation = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

        self.robotid = p.loadURDF("robot.urdf", position, orientation)


        if GUI:
            time.sleep(TIME_DELTA)

        skew_x = self.goal_x - self.x
        skew_y = self.goal_y - self.y

        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))

        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - self.start_angle

        distance = math.sqrt((self.goal_x-self.x)**2+(self.goal_y-self.y)**2)
        self.init_x = self.x
        self.init_y = self.y
        robot_state = [theta, distance, self.x, self.y, self.goal_x, self.goal_y, 0, 0, self.init_x, self.init_y]
        return robot_state, distance, self._run_lidar()

    def debug_circle_reset(self):
        global state, distance, min_dist
        if self.debug_start_angle < 2*np.pi:
            state, distance, min_dist = self.reset(reload=True, angle=self.debug_start_angle+DEBUG_CIRCLE_INCREMENT)
            self.debug_start_angle+= DEBUG_CIRCLE_INCREMENT
        if self.debug_start_angle >= 2*np.pi:
            return [0,0,0,0,0,0,0,0], 0, 0, True
        return state, distance, min_dist, False


    def _get_collisions(self):
        points = p.getContactPoints(self.robotid)
        filtered_points = []
        for point in points:
            if point[1] != self.floor and point[2] != self.floor:
                filtered_points.append(point)
        if len(filtered_points) > 0:
            return True
        else:
            return False

    def _run_lidar(self):
        for did in self.ray_debug_id:
            p.removeUserDebugItem(did)
        self.ray_debug_id.clear()
        start_positions = []
        end_positions = []
        for i in range(LIDAR_POINTS):
            start_positions.append([self.x, self.y, 0.5])
            end_positions.append([self.x + LIDAR_RANGE * math.cos(i * 2 * math.pi / LIDAR_POINTS - LIDAR_ANGLE/2),
                                  self.y + LIDAR_RANGE * math.sin(i * 2 * math.pi / LIDAR_POINTS - LIDAR_ANGLE/2), 0.5])
        res = p.rayTestBatch(start_positions, end_positions)

        distances = []
        non_goal = []

        for i in range(len(res)):
            result = res[i]
            if result[0] > -1 and result[0] != self.goal:
                hit_position = result[3]  # Get the collision point
                start_point = start_positions[i]
                distance_to_collision = math.sqrt(math.pow(hit_position[0] - start_point[0], 2) + math.pow(hit_position[1] - start_point[1], 2))
                distances.append(distance_to_collision)
                if result[0] != self.goal:
                    non_goal.append(distance_to_collision)
                #debug_ray = p.addUserDebugLine(start_point, hit_position, [1, 0, 0], 1, 0.01)
                #self.ray_debug_id.append(debug_ray)
            #else:
                #debug_ray = p.addUserDebugLine(start_positions[i], end_positions[i], [0, 1, 0], 1, 0.01)
                #self.ray_debug_id.append(debug_ray)

        self.lidar_dists = distances

        if len(non_goal) == 0:
            return 0
        return min(non_goal)

    def _new_goal(self, new_pos=True):
        if new_pos:
            self.goal_x = np.random.uniform(SPAWN_BORDER-self.basis.size.width/2, self.basis.size.width/2-SPAWN_BORDER)
            self.goal_y = np.random.uniform(SPAWN_BORDER-self.basis.size.height/2, self.basis.size.height/2-SPAWN_BORDER)
            goal_fine = False
            distance = 0
            while not goal_fine:
                goal_fine = True
                distance = math.sqrt((self.goal_x-self.x)**2+(self.goal_y-self.y)**2)
                if distance<MIN_START_DIST:
                    goal_fine=False
                    self.goal_x = np.random.uniform(SPAWN_BORDER - self.basis.size.width / 2,
                                                    self.basis.size.width / 2 - SPAWN_BORDER)
                    self.goal_y = np.random.uniform(SPAWN_BORDER - self.basis.size.height / 2,
                                                    self.basis.size.height / 2 - SPAWN_BORDER)
                if goal_fine is True:
                    for i in range(len(self.basis.obstacles)):
                        if self.basis.obstacles[i].Loc.x-self.basis.obstacles[i].Size.width/2<self.goal_x<self.basis.obstacles[i].Loc.x+self.basis.obstacles[i].Size.width/2 and self.basis.obstacles[i].Loc.y-self.basis.obstacles[i].Size.height/2<self.goal_y<self.basis.obstacles[i].Loc.y+self.basis.obstacles[i].Size.height/2:
                            goal_fine = False
                            self.goal_x = np.random.uniform(SPAWN_BORDER - self.basis.size.width / 2,
                                                            self.basis.size.width / 2 - SPAWN_BORDER)
                            self.goal_y = np.random.uniform(SPAWN_BORDER - self.basis.size.height / 2,
                                                            self.basis.size.height / 2 - SPAWN_BORDER)
                            break
        self.goal = p.loadURDF("goal.urdf", [self.goal_x, self.goal_y, 0.1])

    def _reset_situation(self, ideal_angle, new_goal=True, new_start=True, new_angle=True, set_angle=-1):
        if new_start:
            x = 0
            y = 0
            position_fine = False
            while not position_fine:
                x = np.random.uniform(SPAWN_BORDER, self.basis.size.width-SPAWN_BORDER)
                y = np.random.uniform(SPAWN_BORDER, self.basis.size.height-SPAWN_BORDER)
                is_fine = True
                for i in range(len(self.basis.obstacles)):
                    if self.basis.vobstacles[i].Loc.x < x < self.basis.vobstacles[i].Loc.x + self.basis.vobstacles[i].Size.width and self.basis.vobstacles[i].Loc.y < y < self.basis.vobstacles[i].Loc.y + self.basis.vobstacles[i].Size.height:
                        is_fine = False
                        break
                position_fine = is_fine
                if position_fine:
                    break
            x=x-self.basis.size.width/2
            y=y-self.basis.size.height/2
            self.x = x
            self.y = y
            self.start_x = x
            self.start_y = y
        else:
            self.x = self.start_x
            self.y = self.start_y
        self._new_goal(new_pos=new_goal)
        angle_to_goal = math.atan2(self.goal_y-self.y, self.goal_x-self.x)
        if new_angle:
            if set_angle == -1: self.start_angle = np.random.uniform(angle_to_goal-ideal_angle, angle_to_goal+ideal_angle)
            else: self.start_angle=set_angle