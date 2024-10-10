"""
@file: env.py
@breif: 2-dimension environment
@author: Winter
@update: 2023.1.13
"""
from math import sqrt
from abc import ABC, abstractmethod
from scipy.spatial import cKDTree
import numpy as np

from .node import Node

class Env(ABC):
    """
    Class for building 2-d workspace of robots.

    Parameters:
        x_range (int): x-axis range of enviroment
        y_range (int): y-axis range of environmet
        eps (float): tolerance for float comparison

    Examples:
        >>> from python_motion_planning.utils import Env
        >>> env = Env(30, 40)
    """
    def __init__(self, x_range: int, y_range: int, eps: float = 1e-6) -> None:
        # size of environment
        self.x_range = x_range  
        self.y_range = y_range
        self.eps = eps

    @property
    def grid_map(self) -> set:
        return {(i, j) for i in range(self.x_range) for j in range(self.y_range)}

    @abstractmethod
    def init(self) -> None:
        pass

class Grid(Env):
    """
    Class for discrete 2-d grid map.
    """
    def __init__(self, x_range: int, y_range: int, obstacles=None) -> None:
        """
        Initialize a grid with specified x and y range, and optional obstacles.

        :param x_range: Horizontal size of the grid.
        :param y_range: Vertical size of the grid.
        :param obstacles: An iterable of tuples (x, y) representing obstacle positions.
        """
        super().__init__(x_range, y_range)
        self.motions = [Node((-1, 0), None, 1, None), Node((-1, 1),  None, sqrt(2), None),
                        Node((0, 1),  None, 1, None), Node((1, 1),   None, sqrt(2), None),
                        Node((1, 0),  None, 1, None), Node((1, -1),  None, sqrt(2), None),
                        Node((0, -1), None, 1, None), Node((-1, -1), None, sqrt(2), None)]
        self.obstacles = set(obstacles) if obstacles is not None else set()
        self.init()

    def init(self) -> None:
        """
        Initialize or re-initialize the grid map.
        """
        x, y = self.x_range, self.y_range

        # Add boundary obstacles if not already present in the user-defined obstacles
        for i in range(x):
            self.obstacles.add((i, 0))
            self.obstacles.add((i, y - 1))
        for i in range(1, y - 1):  # Avoid duplicating corners
            self.obstacles.add((0, i))
            self.obstacles.add((x - 1, i))

        self.obstacles_tree = cKDTree(np.array(list(self.obstacles)))

    def update(self, obstacles):
        """
        Update the grid map with new obstacles.

        :param obstacles: An iterable of tuples (x, y) representing new obstacle positions.
        """
        self.obstacles.update(obstacles)
        self.init()



class Map(Env):
    """
    Class for continuous 2-d map.
    """
    def __init__(self, x_range: int, y_range: int) -> None:
        super().__init__(x_range, y_range)
        self.boundary = None
        self.obs_circ = None
        self.obs_rect = None
        self.init()

    def init(self):
        """
        Initialize map.
        """
        x, y = self.x_range, self.y_range

        # boundary of environment
        #设置边界
        self.boundary = [
            [0, 0, 1, y],
            [0, y, x, 1],
            [1, 0, x, 1],
            [x, 1, 1, y]
        ]

        # user-defined obstacles
        self.obs_rect = [
            [14, 12, 8, 2],
            [18, 22, 8, 3],
            [26, 7, 2, 12],
            [32, 14, 10, 2],
            [32, 3, 10, 2]
        ]

        self.obs_circ = [
            [7, 12, 3],
            [46, 20, 2],
            [15, 5, 2],
            [37, 7, 3],
            [37, 23, 3]
        ]

    def update(self, boundary, obs_circ, obs_rect):
        self.boundary = boundary if boundary else self.boundary
        self.obs_circ = obs_circ if obs_circ else self.obs_circ
        self.obs_rect = obs_rect if obs_rect else self.obs_rect
