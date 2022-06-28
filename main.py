# -*- coding: utf-8 -*-
"""
Created on Fri Jun 24 10:22:59 2022

@author: EliasC
"""

from libraries import *
from mainwindow import *
   
if __name__ == '__main__':
 
    app = QtWidgets.QApplication.instance() # checks if QApplication already exists 
    
    if not app: # create QApplication if it doesnt exist 
        app = QtWidgets.QApplication([])
    
    
    app.setStyle('Fusion')

 
    simulator = MainWindow()
    # simulator.setWindowIcon(QIcon('images\Logo_mini.png'))
    simulator.show()
    
    app.exec_()