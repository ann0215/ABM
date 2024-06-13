import re
import toml
import logging
import numpy as np
from time import time
from numba import njit
from functools import wraps
from typing import Dict, List, Tuple
from abc import ABC, abstractmethod
from contextlib import contextmanager
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Polygon
from matplotlib.collections import PatchCollection


### fieldofview

class FieldOfView(object):
    """Compute field of view prefactors.

    The field of view angle twophi is given in degrees.
    out_of_view_factor is C in the paper.
    """

    def __init__(self, phi=None, out_of_view_factor=None):
        phi = phi or 100.0
        out_of_view_factor = out_of_view_factor or 0.5
        self.cosphi = np.cos(phi / 180.0 * np.pi)
        self.out_of_view_factor = out_of_view_factor

    def __call__(self, desired_direction, forces_direction):
        """Weighting factor for field of view.

        desired_direction : e, rank 2 and normalized in the last index.
        forces_direction : f, rank 3 tensor.
        """
        in_sight = (
            np.einsum("aj,abj->ab", desired_direction, forces_direction)
            > np.linalg.norm(forces_direction, axis=-1) * self.cosphi
        )
        out = self.out_of_view_factor * np.ones_like(in_sight)
        out[in_sight] = 1.0
        np.fill_diagonal(out, 0.0)
        return out


### scene

class PedState:
    """Tracks the state of pedstrains and social groups"""

    def __init__(self, state, groups, config):
        self.default_tau = config("tau", 0.5)
        self.step_width = config("step_width", 0.4)
        self.agent_radius = config("agent_radius", 0.35)
        self.max_speed_multiplier = config("max_speed_multiplier", 1.5)

        self.max_speeds = None
        self.initial_speeds = None

        self.ped_states = []
        self.group_states = []

        self.update(state, groups)

    def update(self, state, groups):
        self.state = state
        self.groups = groups

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, state):
        tau = self.default_tau * np.ones(state.shape[0])
        if state.shape[1] < 7:
            self._state = np.concatenate((state, np.expand_dims(tau, -1)), axis=-1)
        else:
            self._state = state
        if self.initial_speeds is None:
            self.initial_speeds = self.speeds()
        self.max_speeds = self.max_speed_multiplier * self.initial_speeds
        self.ped_states.append(self._state.copy())

    def get_states(self):
        return np.stack(self.ped_states), self.group_states

    def size(self) -> int:
        return self.state.shape[0]

    def pos(self) -> np.ndarray:
        return self.state[:, 0:2]

    def vel(self) -> np.ndarray:
        return self.state[:, 2:4]

    def goal(self) -> np.ndarray:
        return self.state[:, 4:6]

    def tau(self):
        return self.state[:, 6:7]

    def speeds(self):
        """Return the speeds corresponding to a given state."""
        return speeds(self.state)

    def step(self, force, groups=None):
        """Move peds according to forces"""
        # desired velocity
        desired_velocity = self.vel() + self.step_width * force
        desired_velocity = self.capped_velocity(desired_velocity, self.max_speeds)
        # stop when arrived
        desired_velocity[desired_directions(self.state)[1] < 0.5] = [0, 0]

        # update state
        next_state = self.state
        next_state[:, 0:2] += desired_velocity * self.step_width
        next_state[:, 2:4] = desired_velocity
        next_groups = self.groups
        if groups is not None:
            next_groups = groups
        self.update(next_state, next_groups)

    def desired_directions(self):
        return desired_directions(self.state)[0]

    @staticmethod
    def capped_velocity(desired_velocity, max_velocity):
        """Scale down a desired velocity to its capped speed."""
        desired_speeds = np.linalg.norm(desired_velocity, axis=-1)
        factor = np.minimum(1.0, max_velocity / desired_speeds)
        factor[desired_speeds == 0] = 0.0
        return desired_velocity * np.expand_dims(factor, -1)

    @property
    def groups(self) -> List[List]:
        return self._groups

    @groups.setter
    def groups(self, groups: List[List]):
        if groups is None:
            self._groups = []
        else:
            self._groups = groups
        self.group_states.append(self._groups.copy())

    def has_group(self):
        return self.groups is not None

    # def get_group_by_idx(self, index: int) -> np.ndarray:
    #     return self.state[self.groups[index], :]

    def which_group(self, index: int) -> int:
        """find group index from ped index"""
        for i, group in enumerate(self.groups):
            if index in group:
                return i
        return -1


class EnvState:
    """State of the environment obstacles"""

    def __init__(self, obstacles, resolution=10):
        self.resolution = resolution
        self.obstacles = obstacles

    @property
    def obstacles(self) -> List[np.ndarray]:
        """obstacles is a list of np.ndarray"""
        return self._obstacles

    @obstacles.setter
    def obstacles(self, obstacles):
        """Input an list of (startx, endx, starty, endy) as start and end of a line"""
        if obstacles is None:
            self._obstacles = []
        else:
            self._obstacles = []
            for startx, endx, starty, endy in obstacles:
                samples = int(np.linalg.norm((startx - endx, starty - endy)) * self.resolution)
                line = np.array(
                    list(
                        zip(np.linspace(startx, endx, samples), np.linspace(starty, endy, samples))
                    )
                )
                self._obstacles.append(line)



### potentials

class PedPedPotential(object):
    """Ped-ped interaction potential.

    v0 is in m^2 / s^2.
    sigma is in m.
    """

    def __init__(self, delta_t, v0=None, sigma=None):
        self.delta_t = delta_t
        self.v0 = v0 or 2.1
        self.sigma = sigma or 0.3

    def b(self, r_ab, speeds, desired_directions):
        """Calculate b.
        b denotes the semi-minor axis of the ellipse and is given by
        e: desired direction
        2b=sqrt((r_ab+(r_ab-v*delta_t*e_b))
        """
        speeds_b = np.expand_dims(speeds, axis=0)
        speeds_b_abc = np.expand_dims(speeds_b, axis=2)  # abc = alpha, beta, coordinates
        e_b = np.expand_dims(desired_directions, axis=0)

        in_sqrt = (
            np.linalg.norm(r_ab, axis=-1)
            + np.linalg.norm(r_ab - self.delta_t * speeds_b_abc * e_b, axis=-1)
        ) ** 2 - (self.delta_t * speeds_b) ** 2
        np.fill_diagonal(in_sqrt, 0.0)

        return 0.5 * np.sqrt(in_sqrt)

    def value_r_ab(self, r_ab, speeds, desired_directions):
        """Value of potential explicitly parametrized with r_ab."""
        return self.v0 * np.exp(-self.b(r_ab, speeds, desired_directions) / self.sigma)

    @staticmethod
    def r_ab(state):
        """r_ab
        r_ab := r_a - r_b.
        """
        return vec_diff(state[:, :2])

    def __call__(self, state):
        speeds = speeds(state)
        return self.value_r_ab(self.r_ab(state), speeds, desired_directions(state))

    def grad_r_ab(self, state, delta=1e-3):
        """Compute gradient wrt r_ab using finite difference differentiation."""
        r_ab = self.r_ab(state)
        speeds = speeds(state)
        desired_directions = desired_directions(state)

        dx = np.array([[[delta, 0.0]]])
        dy = np.array([[[0.0, delta]]])

        v = self.value_r_ab(r_ab, speeds, desired_directions)
        dvdx = (self.value_r_ab(r_ab + dx, speeds, desired_directions) - v) / delta
        dvdy = (self.value_r_ab(r_ab + dy, speeds, desired_directions) - v) / delta

        # remove gradients from self-intereactions
        np.fill_diagonal(dvdx, 0.0)
        np.fill_diagonal(dvdy, 0.0)

        return np.stack((dvdx, dvdy), axis=-1)


class PedSpacePotential(object):
    """Pedestrian-obstacles interaction potential.

    obstacles is a list of numpy arrays containing points of boundaries.

    u0 is in m^2 / s^2.
    r is in m
    """

    def __init__(self, obstacles, u0=None, r=None):
        self.obstacles = obstacles or []
        self.u0 = u0 or 10
        self.r = r or 0.2

    def value_r_aB(self, r_aB):
        """Compute value parametrized with r_aB."""
        return self.u0 * np.exp(-1.0 * np.linalg.norm(r_aB, axis=-1) / self.r)

    def r_aB(self, state):
        """r_aB"""
        if not self.obstacles:
            return np.zeros((state.shape[0], 0, 2))

        r_a = np.expand_dims(state[:, 0:2], 1)
        closest_i = [
            np.argmin(np.linalg.norm(r_a - np.expand_dims(B, 0), axis=-1), axis=1)
            for B in self.obstacles
        ]
        closest_points = np.swapaxes(
            np.stack([B[i] for B, i in zip(self.obstacles, closest_i)]), 0, 1
        )  # index order: pedestrian, boundary, coordinates
        return r_a - closest_points

    def __call__(self, state):
        return self.value_r_aB(self.r_aB(state))

    def grad_r_aB(self, state, delta=1e-3):
        """Compute gradient wrt r_aB using finite difference differentiation."""
        r_aB = self.r_aB(state)

        dx = np.array([[[delta, 0.0]]])
        dy = np.array([[[0.0, delta]]])

        v = self.value_r_aB(r_aB)
        dvdx = (self.value_r_aB(r_aB + dx) - v) / delta
        dvdy = (self.value_r_aB(r_aB + dy) - v) / delta

        return np.stack((dvdx, dvdy), axis=-1)



### forces

def camel_to_snake(camel_case_string):
    """Convert CamelCase to snake_case"""

    return re.sub(r"(?<!^)(?=[A-Z])", "_", camel_case_string).lower()


class Force(ABC):
    """Force base class"""

    def __init__(self):
        super().__init__()
        self.scene = None
        self.peds = None
        self.factor = 1.0
        self.config = Config()

    def init(self, scene, config):
        """Load config and scene"""
        # load the sub field corresponding to the force name from global confgi file
        self.config = config.sub_config(camel_to_snake(type(self).__name__))
        if self.config:
            self.factor = self.config("factor", 1.0)

        self.scene = scene
        self.peds = self.scene.peds

    @abstractmethod
    def _get_force(self) -> np.ndarray:
        """Abstract class to get social forces
            return: an array of force vectors for each pedestrians
        """
        raise NotImplementedError

    def get_force(self, debug=False):
        force = self._get_force()
        if debug:
            logger.debug(f"{camel_to_snake(type(self).__name__)}:\n {repr(force)}")
        return force


class GoalAttractiveForce(Force):
    """accelerate to desired velocity"""

    def _get_force(self):
        F0 = (
            1.0
            / self.peds.tau()
            * (
                np.expand_dims(self.peds.initial_speeds, -1) * self.peds.desired_directions()
                - self.peds.vel()
            )
        )
        return F0 * self.factor


class PedRepulsiveForce(Force):
    """Ped to ped repulsive force"""

    def _get_force(self):
        potential_func = PedPedPotential(
            self.peds.step_width, v0=self.config("v0"), sigma=self.config("sigma"),
        )
        f_ab = -1.0 * potential_func.grad_r_ab(self.peds.state)

        fov = FieldOfView(phi=self.config("fov_phi"), out_of_view_factor=self.config("fov_factor"),)
        w = np.expand_dims(fov(self.peds.desired_directions(), -f_ab), -1)
        F_ab = w * f_ab
        return np.sum(F_ab, axis=1) * self.factor


class SpaceRepulsiveForce(Force):
    """obstacles to ped repulsive force"""

    def _get_force(self):
        if self.scene.get_obstacles() is None:
            F_aB = np.zeros((self.peds.size(), 0, 2))
        else:
            potential_func = PedSpacePotential(
                self.scene.get_obstacles(), u0=self.config("u0"), r=self.config("r")
            )
            F_aB = -1.0 * potential_func.grad_r_aB(self.peds.state)
        return np.sum(F_aB, axis=1) * self.factor


class GroupCoherenceForce(Force):
    """ Alternative group coherence force as specified in pedsim_ros"""

    def _get_force(self):
        forces = np.zeros((self.peds.size(), 2))
        if self.peds.has_group():
            for group in self.peds.groups:
                threshold = (len(group) - 1) / 2
                member_pos = self.peds.pos()[group, :]
                com = center_of_mass(member_pos)
                force_vec = com - member_pos
                norms = speeds(force_vec)
                softened_factor = (np.tanh(norms - threshold) + 1) / 2
                forces[group, :] += (force_vec.T * softened_factor).T
        return forces * self.factor


class GroupRepulsiveForce(Force):
    """Group repulsive force"""

    def _get_force(self):
        threshold = self.config("threshold", 0.1)
        forces = np.zeros((self.peds.size(), 2))
        if self.peds.has_group():
            for group in self.peds.groups:
                size = len(group)
                member_pos = self.peds.pos()[group, :]
                diff = each_diff(member_pos)  # others - self
                _, norms = normalize(diff)
                diff[norms > threshold, :] = 0
                forces[group, :] += np.sum(diff.reshape((size, -1, 2)), axis=1)

        return forces * self.factor



# class GroupGazeForce(Force):
#     """Group gaze force"""

#     def _get_force(self):
#         forces = np.zeros((self.peds.size(), 2))
#         directions, dist = desired_directions(self.peds.state)
#         if self.peds.has_group():
#             for group in self.peds.groups:
#                 group_size = len(group)
#                 # 1-agent groups don't need to compute this
#                 if group_size <= 1:
#                     continue
#                 member_pos = self.peds.pos()[group, :]
#                 member_directions = directions[group, :]
#                 member_dist = dist[group]
#                 # use center of mass without the current agent
#                 relative_com = np.array(
#                     [
#                         center_of_mass(member_pos[np.arange(group_size) != i, :2])
#                         - member_pos[i, :]
#                         for i in range(group_size)
#                     ]
#                 )

#                 com_directions, com_dist = normalize(relative_com)
#                 # angle between walking direction and center of mass
#                 element_prod = np.array(
#                     [np.dot(d, c) for d, c in zip(member_directions, com_directions)]
#                 )
#                 force = (
#                     com_dist.reshape(-1, 1)
#                     * element_prod.reshape(-1, 1)
#                     / member_dist.reshape(-1, 1)
#                     * member_directions
#                 )
#                 forces[group, :] += force

#         return forces * self.factor


class DesiredForce(Force):
    """Calculates the force between this agent and the next assigned waypoint.
    If the waypoint has been reached, the next waypoint in the list will be
    selected.
    :return: the calculated force
    """

    def _get_force(self):
        relaxation_time = self.config("relaxation_time", 0.5)
        # goal_threshold = self.config("goal_threshold", 0)
        pos = self.peds.pos()
        vel = self.peds.vel()
        goal = self.peds.goal()
        direction, dist = normalize(goal - pos)
        force = np.zeros((self.peds.size(), 2))
        # force[dist > goal_threshold] = (
        #     direction * self.peds.max_speeds.reshape((-1, 1)) - vel.reshape((-1, 2))
        # )[dist > goal_threshold, :]
        # force[dist <= goal_threshold] = -1.0 * vel[dist <= goal_threshold]
        # force /= relexation_time
        desired_speed = direction * self.peds.max_speeds.reshape((-1, 1))
        force = (desired_speed - vel.reshape((-1, 2))) / relaxation_time
        return force * self.factor


class SocialForce(Force):
    """Calculates the social force between this agent and all the other agents
    belonging to the same scene.
    It iterates over all agents inside the scene, has therefore the complexity
    O(N^2). A better
    agent storing structure in Tscene would fix this. But for small (less than
    10000 agents) scenarios, this is just
    fine.
    :return:  nx2 ndarray the calculated force
    """

    def _get_force(self):
        lambda_importance = self.config("lambda_importance", 2.0)
        gamma = self.config("gamma", 0.35)
        n = self.config("n", 2)
        n_prime = self.config("n_prime", 3)

        pos_diff = each_diff(self.peds.pos())  # n*(n-1)x2 other - self
        diff_direction, diff_length = normalize(pos_diff)
        vel_diff = -1.0 * each_diff(self.peds.vel())  # n*(n-1)x2 self - other

        # compute interaction direction t_ij
        interaction_vec = lambda_importance * vel_diff + diff_direction
        interaction_direction, interaction_length = normalize(interaction_vec)

        # compute angle theta (between interaction and position difference vector)
        theta = vector_angles(interaction_direction) - vector_angles(
            diff_direction
        )
        # compute model parameter B = gamma * ||D||
        B = gamma * interaction_length

        force_velocity_amount = np.exp(-1.0 * diff_length / B - np.square(n_prime * B * theta))
        force_angle_amount = -np.sign(theta) * np.exp(
            -1.0 * diff_length / B - np.square(n * B * theta)
        )
        force_velocity = force_velocity_amount.reshape(-1, 1) * interaction_direction
        force_angle = force_angle_amount.reshape(-1, 1) * left_normal(
            interaction_direction
        )

        force = force_velocity + force_angle  # n*(n-1) x 2
        force = np.sum(force.reshape((self.peds.size(), -1, 2)), axis=1)
        return force * self.factor


class ObstacleForce(Force):
    """Calculates the force between this agent and the nearest obstacle in this
    scene.
    :return:  the calculated force
    """

    def _get_force(self):
        sigma = self.config("sigma", 0.2)
        threshold = self.config("threshold", 0.2) + self.peds.agent_radius
        force = np.zeros((self.peds.size(), 2))
        if len(self.scene.get_obstacles()) == 0:
            return force
        obstacles = np.vstack(self.scene.get_obstacles())
        pos = self.peds.pos()

        for i, p in enumerate(pos):
            diff = p - obstacles
            directions, dist = normalize(diff)
            dist = dist - self.peds.agent_radius
            if np.all(dist >= threshold):
                continue
            dist_mask = dist < threshold
            directions[dist_mask] *= np.exp(-dist[dist_mask].reshape(-1, 1) / sigma)
            force[i] = np.sum(directions[dist_mask], axis=0)

        return force * self.factor
    


### config

class Config:
    """Config loading and updating
    Attribute
    -------------
    config: dict

    Methods
    -------------
    from_dict: update from a dict
    load_config: update from file
    sub_config: return a sub dict wrapped in Config()
    """

    def __init__(self, config=None) -> None:
        self.config = {}
        if config:
            self.config = config

    def from_dict(self, config: Dict) -> None:
        """Update from dict"""
        self.config.update(config)

    def load_config(self, filename: str) -> None:
        """update from file"""
        user_config = toml.load(filename)
        self.from_dict(user_config)

    def sub_config(self, field_name: str) -> "Config":
        """return a sub dict wrapped in Config()"""
        sub_dict = self.config.get(field_name)
        if isinstance(sub_dict, dict):
            return Config(sub_dict)
        return Config()

    def __call__(self, entry: str, default=None):
        return self.config.get(entry) or default


class DefaultConfig(Config):
    """Default configs"""

    CONFIG = """
    title = "Social Force Default Config File"

    [scene]
    enable_group = true
    agent_radius = 0.35
    step_width = 1.0
    max_speed_multiplier = 1.5
    tau = 0.5
    resolution = 10

    [goal_attractive_force]
    factor = 1

    [ped_repulsive_force]
    factor = 1.5
    v0 = 2.1
    sigma = 0.3
    # fov params
    fov_phi = 100.0
    fov_factor = 0.5 # out of view factor

    [space_repulsive_force]
    factor = 1
    u0 = 10
    r = 0.2

    [group_coherence_force]
    factor = 3.0

    [group_repulsive_force]
    factor = 1.0
    threshold = 0.55

    [group_gaze_force]
    factor = 4.0
    # fov params
    fov_phi = 90.0

    [desired_force]
    factor = 1.0
    relaxation_time = 0.5
    goal_threshold = 0.2

    [social_force]
    factor = 5.1
    lambda_importance = 2.0
    gamma = 0.35
    n = 2
    n_prime = 3

    [obstacle_force]
    factor = 10.0
    sigma = 0.2
    threshold = 3.0

    [along_wall_force]
    """

    def __init__(self):
        # config_dir = Path(__file__).resolve().parent.parent.joinpath("/config")
        # super().__init__(toml.load(config_dir.joinpath(default_config)))
        super().__init__(toml.loads(self.CONFIG))




### plot

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as mpl_animation
    from matplotlib.patches import Circle, Polygon
    from matplotlib.collections import PatchCollection
except ImportError:
    plt = None
    mpl_animation = None


@contextmanager
def canvas(image_file=None, **kwargs):
    """Generic matplotlib context."""
    fig, ax = plt.subplots(**kwargs)
    ax.grid(linestyle="dotted")
    ax.set_aspect(1.0, "datalim")
    ax.set_axisbelow(True)

    yield ax

    fig.set_tight_layout(True)
    if image_file:
        fig.savefig(image_file, dpi=300)
    # fig.show()
    plt.close(fig)


@contextmanager
def animation(length: int, movie_file=None, writer=None, **kwargs):
    """Context for animations."""
    fig, ax = plt.subplots(**kwargs)
    fig.set_tight_layout(True)
    ax.grid(linestyle="dotted")
    ax.set_aspect(1.0, "datalim")
    ax.set_axisbelow(True)

    context = {"ax": ax, "update_function": None, "init_function": None}
    yield context

    ani = mpl_animation.FuncAnimation(
        fig,
        init_func=context["init_function"],
        func=context["update_function"],
        frames=length,
        blit=True,
    )
    if movie_file:
        ani.save(movie_file, writer=writer)
    plt.close(fig)




logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class SceneVisualizer:
    """Context for social nav visualization"""

    def __init__(
        self, scene, output=None, writer="imagemagick", cmap="viridis", agent_colors=None, **kwargs
    ):
        self.scene = scene
        self.states, self.group_states = self.scene.get_states()
        self.cmap = cmap
        self.agent_colors = agent_colors
        self.frames = self.scene.get_length()
        self.output = output
        self.writer = writer

        self.fig, self.ax = plt.subplots(**kwargs)

        self.ani = None

        self.group_actors = None
        self.group_collection = PatchCollection([])
        self.group_collection.set(
            animated=True,
            alpha=0.2,
            cmap=self.cmap,
            facecolors="none",
            edgecolors="purple",
            linewidth=2,
            clip_on=True,
        )

        self.human_actors = None
        self.human_collection = PatchCollection([])
        self.human_collection.set(animated=True, alpha=0.6, cmap=self.cmap, clip_on=True)

    def plot(self):
        """Main method for create plot"""
        self.plot_obstacles()
        groups = self.group_states[0]  # static group for now
        if not groups:
            for ped in range(self.scene.peds.size()):
                x = self.states[:, ped, 0]
                y = self.states[:, ped, 1]
                self.ax.plot(x, y, "-o", label=f"ped {ped}", markersize=2.5)
        else:
            colors = plt.cm.rainbow(np.linspace(0, 1, len(groups)))
            for i, group in enumerate(groups):
                for ped in group:
                    x = self.states[:, ped, 0]
                    y = self.states[:, ped, 1]
                    self.ax.plot(x, y, "-o", label=f"ped {ped}", markersize=2.5, color=colors[i])
        self.ax.legend()
        return self.fig

    def animate(self):
        """Main method to create animation"""
        self.ani = FuncAnimation(
            self.fig,
            init_func=self.animation_init,
            func=self.animation_update,
            frames=self.frames,
            blit=True,
        )
        return self.ani

    def __enter__(self):
        logger.info("Start plotting.")
        self.fig.set_tight_layout(True)
        self.ax.grid(linestyle="dotted")
        self.ax.set_aspect("equal")
        self.ax.margins(2.0)
        self.ax.set_axisbelow(True)
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")

        plt.rcParams["animation.html"] = "jshtml"

        # x, y limit from states, only for animation
        margin = 2.0
        xy_limits = np.array(
            [minmax(state) for state in self.states]
        )  # (x_min, y_min, x_max, y_max)
        xy_min = np.min(xy_limits[:, :2], axis=0) - margin
        xy_max = np.max(xy_limits[:, 2:4], axis=0) + margin
        self.ax.set(xlim=(xy_min[0], xy_max[0]), ylim=(xy_min[1], xy_max[1]))

        return self

    def __exit__(self, exception_type, exception_value, traceback):
        if exception_type:
            logger.error(
                f"Exception type: {exception_type}; Exception value: {exception_value}; Traceback: {traceback}"
            )
        logger.info("Plotting ends.")
        if self.output:
            if self.ani:
                output = self.output + ".gif"
                logger.info(f"Saving animation as {output}")
                self.ani.save(output, writer=self.writer)
            else:
                output = self.output + ".png"
                logger.info(f"Saving plot as {output}")
                self.fig.savefig(output, dpi=300)
        plt.close(self.fig)

    def plot_human(self, step=-1):
        """Generate patches for human
        :param step: index of state, default is the latest
        :return: list of patches
        """
        states, _ = self.scene.get_states()
        current_state = states[step]
        # radius = 0.35
        # if self.human_actors:
        #     for i, human in enumerate(self.human_actors):
        #         human.center = current_state[i, :2]
        #         human.set_radius(radius)
        # else:
        #     self.human_actors = [
        #         Circle(pos, radius) for pos in current_state[:, :2]
        #     ]
        radius = [0.2] * current_state.shape[0]
        if self.human_actors:
            for i, human in enumerate(self.human_actors):
                human.center = current_state[i, :2]
                human.set_radius(0.2)
        else:
            self.human_actors = [
                Circle(pos, r) for pos, r in zip(current_state[:, :2], radius)
            ]
        self.human_collection.set_paths(self.human_actors)
        if not self.agent_colors:
            self.human_collection.set_array(np.arange(current_state.shape[0]))
        else:
            assert len(self.human_actors) == len(
                self.agent_colors
            ), "agent_colors must be the same length as the agents"
            self.human_collection.set_facecolor(self.agent_colors)

    def plot_groups(self, step=-1):
        """Generate patches for groups
        :param step: index of state, default is the latest
        :return: list of patches
        """
        states, group_states = self.scene.get_states()
        current_state = states[step]
        current_groups = group_states[step]
        if self.group_actors:  # update patches, else create
            points = [current_state[g, :2] for g in current_groups]
            for i, p in enumerate(points):
                self.group_actors[i].set_xy(p)
        else:
            self.group_actors = [Polygon(current_state[g, :2]) for g in current_groups]
        self.group_collection.set_paths(self.group_actors)

    def plot_obstacles(self):
        for s in self.scene.get_obstacles():
            self.ax.plot(s[:, 0], s[:, 1], "-o", color="black", markersize=2.5)

    def animation_init(self):
        self.plot_obstacles()
        self.ax.add_collection(self.group_collection)
        self.ax.add_collection(self.human_collection)
        return (self.group_collection, self.human_collection)

    def animation_update(self, i):
        self.plot_groups(i)
        self.plot_human(i)
        return (self.group_collection, self.human_collection)

    def minmax(state):
        x_min = np.min(state[:, 0])
        y_min = np.min(state[:, 1])
        x_max = np.max(state[:, 0])
        y_max = np.max(state[:, 1])
        return x_min, y_min, x_max, y_max



### stateutils

@njit
def vector_angles(vecs: np.ndarray) -> np.ndarray:
    """Calculate angles for an array of vectors
    :param vecs: nx2 ndarray
    :return: nx1 ndarray
    """
    ang = np.arctan2(vecs[:, 1], vecs[:, 0])  # atan2(y, x)
    return ang


@njit
def left_normal(vecs: np.ndarray) -> np.ndarray:
    vecs = np.fliplr(vecs) * np.array([-1.0, 1.0])
    return vecs


@njit
def right_normal(vecs: np.ndarray) -> np.ndarray:
    vecs = np.fliplr(vecs) * np.array([1.0, -1.0])
    return vecs


@njit
def normalize(vecs: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Normalize nx2 array along the second axis
    input: [n,2] ndarray
    output: (normalized vectors, norm factors)
    """
    norm_factors = []
    for line in vecs:
        norm_factors.append(np.linalg.norm(line))
    norm_factors = np.array(norm_factors)
    normalized = vecs / np.expand_dims(norm_factors, -1)
    # get rid of nans
    for i in range(norm_factors.shape[0]):
        if norm_factors[i] == 0:
            normalized[i] = np.zeros(vecs.shape[1])
    return normalized, norm_factors


@njit
def desired_directions(state: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Given the current state and destination, compute desired direction."""
    destination_vectors = state[:, 4:6] - state[:, 0:2]
    directions, dist = normalize(destination_vectors)
    return directions, dist


@njit
def vec_diff(vecs: np.ndarray) -> np.ndarray:
    """r_ab
    r_ab := r_a − r_b.
    """
    diff = np.expand_dims(vecs, 1) - np.expand_dims(vecs, 0)
    return diff


def each_diff(vecs: np.ndarray, keepdims=False) -> np.ndarray:
    """
    :param vecs: nx2 array
    :return: diff with diagonal elements removed
    """
    diff = vec_diff(vecs)
    # diff = diff[np.any(diff, axis=-1), :]  # get rid of zero vectors
    diff = diff[
        ~np.eye(diff.shape[0], dtype=bool), :
    ]  # get rif of diagonal elements in the diff matrix
    if keepdims:
        diff = diff.reshape(vecs.shape[0], -1, vecs.shape[1])

    return diff


@njit
def speeds(state: np.ndarray) -> np.ndarray:
    """Return the speeds corresponding to a given state."""
    state = state.astype(np.float64)
    speed_vecs = state[:, 2:4]
    speeds_array = np.empty(state.shape[0], dtype=np.float64)
    for i in range(state.shape[0]):
        speeds_array[i] = np.linalg.norm(speed_vecs[i])
    return speeds_array


@njit
def center_of_mass(vecs: np.ndarray) -> np.ndarray:
    """Center-of-mass of a given group"""
    return np.sum(vecs, axis=0) / vecs.shape[0]


@njit
def minmax(vecs: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    x_min = np.min(vecs[:, 0])
    y_min = np.min(vecs[:, 1])
    x_max = np.max(vecs[:, 0])
    y_max = np.max(vecs[:, 1])
    return (x_min, y_min, x_max, y_max)


### log

# Create a custom logger
logger = logging.getLogger("root")
logger.setLevel(logging.DEBUG)
FORMAT = "%(levelname)s:[%(filename)s:%(lineno)s %(funcName)20s() ] %(message)s"

# Create handlers
c_handler = logging.StreamHandler()
f_handler = logging.FileHandler("file.log")
c_handler.setLevel(logging.DEBUG)
f_handler.setLevel(logging.WARNING)

# Create formatters and add it to handlers
c_format = logging.Formatter(FORMAT)
f_format = logging.Formatter("%(asctime)s|" + FORMAT)
c_handler.setFormatter(c_format)
f_handler.setFormatter(f_format)

# Add handlers to the logger
logger.addHandler(c_handler)
logger.addHandler(f_handler)


def timeit(f):
    @wraps(f)
    def wrap(*args, **kw):
        ts = time()
        result = f(*args, **kw)
        te = time()
        logger.debug(f"Timeit: {f.__name__}({args}, {kw}), took: {te-ts:2.4f} sec")
        return result

    return wrap



# simulator


class Simulator:
    """Simulate social force model.

    ...

    Attributes
    ----------
    state : np.ndarray [n, 6] or [n, 7]
       Each entry represents a pedestrian state, (x, y, v_x, v_y, d_x, d_y, [tau])
    obstacles : np.ndarray
        Environmental obstacles
    groups : List of Lists
        Group members are denoted by their indices in the state
    config : Dict
        Loaded from a toml config file
    max_speeds : np.ndarray
        Maximum speed of pedestrians
    forces : List
        Forces to factor in during navigation

    Methods
    ---------
    capped_velocity(desired_velcity)
        Scale down a desired velocity to its capped speed
    step()
        Make one step
    """

    def __init__(self, state, groups=None, obstacles=None, config_file=None):
        self.config = DefaultConfig()
        if config_file:
            self.config.load_config(config_file)
        # TODO: load obstacles from config
        self.scene_config = self.config.sub_config("scene")
        # initiate obstacles
        self.env = EnvState(obstacles, self.config("resolution", 10.0))

        # initiate agents
        self.peds = PedState(state, groups, self.config)

        # construct forces
        self.forces = self.make_forces(self.config)

    def make_forces(self, force_configs):
        """Construct forces"""
        force_list = [
            DesiredForce(),
            SocialForce(),
            ObstacleForce(),
            # forces.PedRepulsiveForce(),
            # forces.SpaceRepulsiveForce(),
        ]
        group_forces = [
            GroupCoherenceForce(),
            # GroupRepulsiveForce(),
            # GroupGazeForce(),
        ]
        if self.scene_config("enable_group"):
            force_list += group_forces

        # initiate forces
        for force in force_list:
            force.init(self, force_configs)

        return force_list

    def compute_forces(self):
        """compute forces"""
        return sum(map(lambda x: x.get_force(), self.forces))

    def get_states(self):
        """Expose whole state"""
        return self.peds.get_states()

    def get_length(self):
        """Get simulation length"""
        return len(self.get_states()[0])

    def get_obstacles(self):
        return self.env.obstacles

    def step_once(self):
        """step once"""
        self.peds.step(self.compute_forces())

    def step(self, n=1):
        """Step n time"""
        for _ in range(n):
            self.step_once()
        return self
