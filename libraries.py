# -*- coding: utf-8 -*-
"""
Created on Thu Jun 23 17:11:12 2022

@author: EliasC
"""

import numpy as np
import matplotlib.pyplot as plt
import math
import sys

from PyQt5 import QtWidgets
from PyQt5 import QtCore
from PyQt5.QtCore import QThread
from PyQt5.QtCore import QObject
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

from mpl_toolkits.mplot3d import Axes3D


from GUI import Ui_MainWindow 

#Variables globales


RAD_TO_DEF = 180/math.pi
DEG_TO_RAD = math.pi/180
