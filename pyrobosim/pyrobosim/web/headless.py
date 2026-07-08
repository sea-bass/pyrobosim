"""
A minimal stand-in for the Qt GUI, used by the web frontend.

This is implemented purely for compatibility with the Qt GUI, and should be
removed in the next major release of PyRoboSim when we fully switch over to
the web frontend for visualization.

The web app renders by polling the world directly, so it does not need the Qt
canvas. However, the core world model calls ``world.gui.canvas.*`` to refresh
the Qt canvas (guarded by ``if world.gui is not None``), and some examples wait
in ``while world.gui is None`` before proceeding. ``HeadlessGui`` mirrors
exactly the (small) interface that ``pyrobosim.core`` accesses on the GUI
object:

* Structural-change hooks (pick/place/open/close/detect/plan) bump a shared
  ``change_count`` so the web engine can cheaply tell when a discrete change
  happened without re-scanning world state every frame.
* ``navigate_signal`` stays *functional*: core delegates navigation to the GUI
  (the Qt ``NavRunner`` thread), so task plans would not move the robot without
  it.
* ``show_planner_and_path_signal`` stashes the path (and graph visibility) per
  robot. This matters for paths supplied ready-made by an action (e.g. a
  PDDLStream ``navigate`` action whose path came from a stream): the path
  planner never records those, so the figure cannot recover them from
  ``get_latest_path``.
* ``obj_patches`` stores nothing, so core never tries to remove a Matplotlib
  artist that was never added to a real axes.
"""

import threading
from typing import Any, Callable


class _Signal:
    """Stand-in for a Qt signal: calls ``fn`` (if given) when emitted."""

    def __init__(self, fn: Callable[..., None] | None = None) -> None:
        self._fn = fn

    def emit(self, *args: Any, **kwargs: Any) -> None:
        if self._fn is not None:
            self._fn(*args, **kwargs)


def _navigate_on_thread(
    robot: Any, goal: Any, path: Any = None, realtime_factor: float = 1.0
) -> None:
    """Runs a robot navigation on a background thread, like the Qt ``NavRunner``."""
    threading.Thread(
        target=lambda: robot.navigate(
            goal=goal, path=path, realtime_factor=realtime_factor
        ),
        daemon=True,
    ).start()


class _NoPatchList:
    """
    Stand-in for the canvas object-patch list that stores nothing.

    In web mode object visualization patches are never added to a real
    Matplotlib axes, so core must not try to remove them (it would raise
    "cannot remove artist"). Reporting ``False`` for membership keeps core out
    of that code path.
    """

    def append(self, item: Any) -> None:
        pass

    def remove(self, item: Any) -> None:
        pass

    def __contains__(self, item: Any) -> bool:
        return False


class _Axes:
    """No-op stand-in for a Matplotlib axes."""

    def add_patch(self, *args: Any, **kwargs: Any) -> None:
        pass


class _HeadlessCanvas:
    """No-op stand-in for the Qt ``WorldCanvas``."""

    def __init__(self, bump: Callable[..., None]) -> None:
        self.axes = _Axes()
        self.obj_patches = _NoPatchList()
        # Latest path handed to the canvas per robot (e.g. PDDLStream-supplied
        # paths that the path planner never recorded), read by the web figure.
        self.displayed_paths: dict[str, Any] = {}
        # Whether to show planner graphs per robot: True when the planner planned
        # during the action, False when a path was supplied ready-made.
        self.graphs_visible: dict[str, bool] = {}
        self._bump = bump
        # Frequent generic redraw hook: ignored (motion is handled by polling).
        self.draw_signal = _Signal()
        # Functional: actually performs navigation.
        self.navigate_signal = _Signal(_navigate_on_thread)
        # Structural-change hooks: bump the shared change counter.
        self.show_hallways_signal = _Signal(bump)
        self.show_locations_signal = _Signal(bump)
        self.show_objects_signal = _Signal(bump)
        self.show_robots_signal = _Signal(bump)
        # Functional: stashes the supplied path / graph visibility, bumps counter.
        self.show_planner_and_path_signal = _Signal(self._show_planner_and_path)

    def _show_planner_and_path(
        self, robot: Any, show_graphs: bool = True, path: Any = None
    ) -> None:
        if robot is not None:
            self.displayed_paths[robot.name] = path
            self.graphs_visible[robot.name] = show_graphs
        self._bump()

    def show(self) -> None:
        # Called on world reset; drop stale state so a reset world starts clean
        # (robots are rebuilt, so their old supplied paths no longer apply).
        self.displayed_paths.clear()
        self.graphs_visible.clear()

    def show_objects(self) -> None:
        pass

    def show_world_state(self, robot: Any = None) -> None:
        pass

    def update_object_plot(self, obj: Any) -> None:
        pass


class HeadlessGui:
    """Minimal ``world.gui`` stand-in so core's GUI hooks are no-ops in web mode."""

    def __init__(self) -> None:
        #: Number of structural-change hooks fired so far.
        self.change_count = 0
        self.canvas = _HeadlessCanvas(self._bump)
        self.update_buttons_signal = _Signal(self._bump)

    def _bump(self, *args: Any, **kwargs: Any) -> None:
        self.change_count += 1

    def set_buttons_during_action(self, state: bool) -> None:
        pass
