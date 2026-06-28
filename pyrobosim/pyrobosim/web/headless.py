"""
A minimal stand-in for the Qt GUI, used by the web frontend.

This is implemented purely for compatibility with the Qt GUI, and should be
removed in the next major release of PyRoboSim when we fully switch over to
the web frontend for visualization.

The web app renders by polling the world directly, so it does not need the Qt
canvas. However, the core world model and some examples treat ``world.gui`` as a
signal that "a GUI is attached / ready": core methods call
``world.gui.canvas.*`` to refresh the Qt canvas (all guarded by
``if world.gui is not None``), and examples wait in ``while world.gui is None``
before proceeding.

``HeadlessGui`` satisfies both. It mirrors exactly the (small) interface that
``pyrobosim.core`` accesses on the GUI object, and:

* turns canvas-refresh hooks into a shared **change counter** so the web engine
  can cheaply tell when a discrete change happened (pick/place/open/close/
  detect/plan) without re-scanning world state every frame;
* keeps ``navigate_signal`` *functional* (core delegates navigation to it);
* stores no object patches, so core never tries to remove a Matplotlib artist
  that was never added to a real axes.
"""

import threading
from typing import Any


class _Counter:
    """A simple shared integer counter."""

    def __init__(self) -> None:
        self.value = 0


class _Signal:
    """No-op stand-in for a Qt signal (used for the frequent ``draw_signal``)."""

    def emit(self, *args: Any, **kwargs: Any) -> None:
        pass


class _CounterSignal:
    """A signal stand-in that bumps a shared counter whenever it is emitted."""

    def __init__(self, counter: _Counter) -> None:
        self._counter = counter

    def emit(self, *args: Any, **kwargs: Any) -> None:
        self._counter.value += 1


class _ShowPathSignal:
    """
    Functional stand-in for the canvas ``show_planner_and_path_signal``.

    Core emits this whenever a path should be displayed: when a planner plans on
    the spot (carrying the freshly planned path), and when an action supplies its
    own path -- e.g. a PDDLStream ``navigate`` action whose path is produced by a
    stream rather than planned at navigation time. In the latter case the path
    planner never records the path, so the web figure cannot recover it from
    ``get_latest_path``. We therefore stash the path here (per robot) for the
    figure to render, and bump the change counter so the engine does a full
    refresh.

    The ``show_graphs`` flag is stashed too: graphs are only meaningful when the
    planner actually planned during the action (``True``), not when a path was
    supplied ready-made (``False``), so the figure suppresses them in the latter
    case -- mirroring the Qt canvas.
    """

    def __init__(
        self,
        counter: _Counter,
        paths: dict[str, Any],
        graphs_visible: dict[str, bool],
    ) -> None:
        self._counter = counter
        self._paths = paths
        self._graphs_visible = graphs_visible

    def emit(self, robot: Any, show_graphs: bool = True, path: Any = None) -> None:
        if robot is not None:
            self._paths[robot.name] = path
            self._graphs_visible[robot.name] = show_graphs
        self._counter.value += 1


class _NavigateSignal:
    """
    Functional stand-in for the Qt canvas ``navigate_signal``.

    Core's ``Robot.execute_action`` does not navigate inline when a GUI is
    attached; it emits this signal and waits for ``executing_nav`` to clear,
    expecting the GUI to run the navigation on a thread (the Qt ``NavRunner``).
    This replicates that so task plans actually move the robot in web mode.
    """

    def emit(
        self,
        robot: Any,
        goal: Any,
        path: Any = None,
        realtime_factor: float = 1.0,
    ) -> None:
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

    def __init__(self, counter: _Counter) -> None:
        self.axes = _Axes()
        self.obj_patches = _NoPatchList()
        # Latest path handed to the canvas per robot (e.g. PDDLStream-supplied
        # paths that the path planner never recorded), read by the web figure.
        self.displayed_paths: dict[str, Any] = {}
        # Whether to show planner graphs per robot: True when the planner planned
        # during the action, False when a path was supplied ready-made.
        self.graphs_visible: dict[str, bool] = {}
        # Frequent generic redraw hook: ignored (motion is handled by polling).
        self.draw_signal = _Signal()
        # Functional: actually performs navigation.
        self.navigate_signal = _NavigateSignal()
        # Structural-change hooks: bump the shared counter.
        self.show_hallways_signal = _CounterSignal(counter)
        self.show_locations_signal = _CounterSignal(counter)
        self.show_objects_signal = _CounterSignal(counter)
        self.show_robots_signal = _CounterSignal(counter)
        # Functional: stashes the supplied path / graph visibility, bumps counter.
        self.show_planner_and_path_signal = _ShowPathSignal(
            counter, self.displayed_paths, self.graphs_visible
        )

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
        self._counter = _Counter()
        self.canvas = _HeadlessCanvas(self._counter)
        self.update_buttons_signal = _CounterSignal(self._counter)

    @property
    def change_count(self) -> int:
        """Number of structural-change hooks fired so far."""
        return self._counter.value

    def set_buttons_during_action(self, state: bool) -> None:
        pass
