# Curve fit for the force sensor experiment
x = [5,10,20,50,100,200,500]
y22 = [0.26,1.55,1.49,1.30,2.13,3.83,3.38]
def f(x,a,b,c):
    return a * x + b * x^2 + c

import numpy as np
import matplotlib.pyplot as plt
import scipy
#pop, _ = scipy.curve_fit(f,x,y22)
z=np.polyfit(x,y22,2)
plt.plot(z)