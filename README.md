# hBN_TRNG
A Python application to run a live TRNG relying on a 2D-material based memristor (or any other high impedance device), an Arduino development board and a COTS analog front-end.

"""
Created on Wed Jan 19 10:52:10 2022

@author: Sebastián Matías Pazos
"""

Required components:
1) An entropy source device (high impedance) and means to connect it to a transimpedance amplifier.
2) A custom transimpedance amplifier (refer to paper for detailed schematics) or a commercial amplifier.
3) An Arduino NANO development board (or similar) with Standard Pyfirmata sketch loaded. 
NOTE: The Arduino board must be connected to the PC in order for the code to run, even in "Simulation Mode".

NOTICE: Please install Python libraries (see following list) when missing. This code was tested on a standard Anaconda environment under Windows 10 OS.

#importing libraries
import matplotlib.pyplot as plt
from multiprocessing import Process, Pipe
import numpy as np
import pyfirmata
import time
import sys, traceback
from PIL import Image
import scipy.io
from datetime import datetime
import os

