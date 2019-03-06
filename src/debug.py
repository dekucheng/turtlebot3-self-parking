#!/usr/bin/env python
class me():
    def __init__(self):
        self.a = 3
    def poly(self):
        self.img = 3
        self.get()
    def get(self):
        print(3)

a = me()
a.poly()
from scipy.cluster.hierarchy import dendrogram, linkage
import numpy as np
# some setting for this notebook to actually show the graphs inline
# you probably won't need this
np.set_printoptions(precision=5, suppress=True)  # suppress scientific float notation
