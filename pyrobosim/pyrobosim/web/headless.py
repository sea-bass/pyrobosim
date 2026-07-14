"""
A minimal stand-in for the Qt GUI, used by the web frontend.

The web app renders by polling the world directly, but the core world model
still calls a small set of GUI hooks (``world.gui.canvas.*``), and some
examples wait for ``world.gui`` to be set. :class:`HeadlessGui` implements
exactly the interface that ``pyrobosim.core`` accesses on the GUI object:

* Structural-change hooks (pick/place/open/close/detect/plan) bump a shared
  ``change_count``, so the web engine can cheaply detect discrete changes.
* ``navigate_signal`` performs navigation on a background thread, mirroring
  the Qt GUI's navigation runner, since core delegates navigation to the GUI.
* ``show_planner_and_path_signal`` stashes the supplied path and a snapshot
  of the planner graphs per robot, which is what the web figure renders.

This module exists purely for compatibility with the Qt GUI, and should be
removed when PyRoboSim fully switches over to the web frontend.
"""

from typing import Any, Callable

from .commands import run_async
from ..core.robot import Robot
from ..utils.path import Path
from ..utils.search_graph import SearchGraph


class _Signal:
    """Stand-in for a Qt signal: calls ``fn`` (if given) when emitted."""

    def __init__(self, fn: Callable[..., None] | None = None) -> None:
        """
        Creates a signal stand-in.

        :param fn: The callable to invoke on emit. If None, emitting is a no-op.
        """
        self._fn = fn

    def emit(self, *args: Any, **kwargs: Any) -> None:
        """
        Emits the signal, forwarding all arguments to the wrapped callable.

        :param args: Positional arguments for the wrapped callable.
        :param kwargs: Keyword arguments for the wrapped callable.
        """
        if self._fn is not None:
            self._fn(*args, **kwargs)


def _navigate_on_thread(
    robot: Robot, goal: Any, path: Path | None = None, realtime_factor: float = 1.0
) -> None:
    """
    Runs a robot navigation on a background thread, like the Qt GUI does.

    :param robot: The robot to navigate.
    :param goal: The navigation goal (an entity or entity query).
    :param path: An optional ready-made path to follow instead of planning.
    :param realtime_factor: The real-time factor for the motion animation.
    """
    run_async(
        lambda: robot.navigate(goal=goal, path=path, realtime_factor=realtime_factor)
    )


class _NoPatchList:
    """
    Stand-in for the canvas object-patch list that stores nothing.

    In web mode, object visualization patches are never added to real
    Matplotlib axes, so core must not try to remove them. Reporting ``False``
    for membership keeps core out of that code path.
    """

    def append(self, item: Any) -> None:
        """
        Discards the given item.

        :param item: The item to (not) store.
        """

    def remove(self, item: Any) -> None:
        """
        Does nothing, as no items are ever stored.

        :param item: The item to (not) remove.
        """

    def __contains__(self, item: Any) -> bool:
        """
        Reports that no item is in the list.

        :param item: The item to check.
        :return: Always False.
        """
        return False


class _Axes:
    """No-op stand-in for a Matplotlib axes."""

    def add_patch(self, *args: Any, **kwargs: Any) -> None:
        """
        Does nothing; provided for Matplotlib axes compatibility.

        :param args: Ignored.
        :param kwargs: Ignored.
        """


class HeadlessCanvas:
    """No-op stand-in for the Qt ``WorldCanvas``."""

    def __init__(self, bump: Callable[..., None]) -> None:
        """
        Creates a headless canvas.

        :param bump: Callback invoked whenever a structural world change occurs.
        """
        self.axes = _Axes()
        self.obj_patches = _NoPatchList()
        #: Per-robot display state, written by ``show_planner_and_path_signal``
        #: and read by the web figure.
        self.displayed_paths: dict[str, Path | None] = {}
        self.displayed_graphs: dict[str, list[SearchGraph]] = {}
        self._bump = bump
        # Generic redraw hook: ignored, since motion is handled by polling.
        self.draw_signal = _Signal()
        # Functional: actually performs navigation.
        self.navigate_signal = _Signal(_navigate_on_thread)
        # Structural-change hooks: bump the shared change counter.
        self.show_hallways_signal = _Signal(bump)
        self.show_locations_signal = _Signal(bump)
        self.show_objects_signal = _Signal(bump)
        self.show_robots_signal = _Signal(bump)
        # Functional: stashes the supplied path / graph snapshot, bumps counter.
        self.show_planner_and_path_signal = _Signal(self._show_planner_and_path)

    def _show_planner_and_path(
        self, robot: Robot | None, show_graphs: bool = True, path: Path | None = None
    ) -> None:
        """
        Stashes the supplied path and a snapshot of the planner's graphs.

        The graphs are snapshotted (rather than read live at render time)
        because some planners mutate their state continuously while replanning.

        :param robot: The robot the path/graphs belong to. If None, only bumps
            the change counter.
        :param show_graphs: If True, snapshots the planner's current graphs;
            otherwise clears them.
        :param path: The path to display for the robot, if any.
        """
        if robot is not None:
            planner = robot.path_planner
            self.displayed_paths[robot.name] = path
            self.displayed_graphs[robot.name] = (
                list(planner.get_graphs())
                if (show_graphs and planner is not None)
                else []
            )
        self._bump()

    def show(self) -> None:
        """
        Clears the stashed per-robot display state.

        Core calls this on world reset, which rebuilds the robots, so their
        old paths and graphs no longer apply.
        """
        self.displayed_paths.clear()
        self.displayed_graphs.clear()

    def show_objects(self) -> None:
        """Does nothing; provided for Qt canvas compatibility."""

    def show_world_state(self, robot: Any = None) -> None:
        """Does nothing; provided for Qt canvas compatibility."""

    def update_object_plot(self, obj: Any) -> None:
        """Does nothing; provided for Qt canvas compatibility."""


class HeadlessGui:
    """Minimal ``world.gui`` stand-in so core's GUI hooks work in web mode."""

    def __init__(self) -> None:
        """Creates a headless GUI with a fresh canvas and change counter."""
        #: Number of structural-change hooks fired so far.
        self.change_count = 0
        self.canvas = HeadlessCanvas(self._bump)
        self.update_buttons_signal = _Signal(self._bump)

    def _bump(self, *args: Any, **kwargs: Any) -> None:
        """
        Increments the structural-change counter.

        :param args: Ignored, so this can back any signal.
        :param kwargs: Ignored, so this can back any signal.
        """
        self.change_count += 1

    def set_buttons_during_action(self, state: bool) -> None:
        """Does nothing; provided for Qt GUI compatibility."""
