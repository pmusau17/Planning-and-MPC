from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import numpy as np

### There are several general interpolation facilities available in SciPy, for data in 1, 2, and higher dimensions:


# one-dimensional case

# define linearly spaced out points
x = np.linspace(0, 10, num=11, endpoint=True)

# apply some function to them
y = np.cos(-x**2/9.0)

# Try to model that function using a linear interpolation 
f = interp1d(x, y)

# Try doing it with cubic splines
f2 = interp1d(x, y, kind='cubic')


# create more finely spaced out points
xnew = np.linspace(0, 10, num=41, endpoint=True)

# do the plotting
plt.plot(x, y, 'o', xnew, f(xnew), '-', xnew, f2(xnew), '--')
plt.legend(['data', 'linear', 'cubic'], loc='best')
plt.show()

## way more cool things can be found here: https://docs.scipy.org/doc/scipy/reference/tutorial/interpolate.html