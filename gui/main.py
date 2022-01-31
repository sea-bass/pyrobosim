import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from shapely.geometry import Polygon
from descartes.patch import PolygonPatch
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg

from .world import WorldGUI

class PyRoboSim(QtWidgets.QApplication):
    def __init__(self, world, args):
        super(PyRoboSim, self).__init__(args)
        self.set_world(world)

    def set_world(self, world):
        self.ww = WorldWidget(world)
        self.ww.createPoly([(0, 0), (1, 1), (1, 0)])
        self.ww.createPoly([(1, 2), (2, 3), (2, 2)])
        self.ww.show()

class WorldWidget(QtWidgets.QMainWindow):
    def __init__(self, world, *args, **kwargs):
        super(WorldWidget, self).__init__(*args, **kwargs)
        self.setWindowTitle("pyrobosim")

        # Matplotlib stuff
        self.wg = WorldGUI(world, width=5, height=4, dpi=100)

        # Push button
        self.push_button = QtWidgets.QPushButton("Randomize position")
        self.push_button.clicked.connect(self.on_click)

        layout = QtWidgets.QVBoxLayout()
        layout.addWidget(self.push_button)
        layout.addWidget(self.wg)
        widget = QtWidgets.QWidget()
        widget.setLayout(layout)
        self.setCentralWidget(widget)

        self.polygons = []

    def on_click(self):
        L = 0.2
        x = 2.0 * np.random.random()
        y = 3.0 * np.random.random()
        yaw = 2.0 * np.pi * np.random.random()
        self.wg.robot_body.set_xdata(x)
        self.wg.robot_body.set_ydata(y)
        self.wg.robot_dir.set_xdata(x + np.array([0, L*np.cos(yaw)]))
        self.wg.robot_dir.set_ydata(y + np.array([0, L*np.sin(yaw)]))
        self.wg.axes.autoscale()
        self.wg.draw()

    def createPoly(self, coords):
        polygon = Polygon(coords)
        self.polygons.append(polygon)
        patch = PolygonPatch(polygon, fc=[1,0,0], ec=[0,0,1], lw=2, zorder=2)
        self.wg.axes.add_patch(patch)
        self.wg.axes.autoscale()
