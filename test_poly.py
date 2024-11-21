#from roblib import *
import numpy as np

# Genere les coefs d'un polynome d'ordre 3
def coef_polynoms_order3(t0, t1, x0, x1, dx0, dx1): 
    A = np.array([[t0**3, t0**2, t0, 1], [t1**3, t1**2, t1, 1], [3*t0**2, 2*t0, 1, 0], [3*t1**2, 2*t1, 1, 0]])
    B = np.array([x0, x1, dx0, dx1])

    return np.linalg.solve(A, B)

# Genere les coefs d'un polynome d'ordre 4 (Implique de définir des accélérations au WP)
# def coef_polynoms_order4(t0, t1, x0, x1, dx0, dx1, ddx0, ddx1): 
#     A = np.array([[t0**4, t0**3, t0**2, t0, 1], [t1**4, t1**3, t1**2, t1, 1], [4*t0**3, 3*t0**2, 2*t0, 1, 0],
#                 [4*t1**3, 3*t1**2, 2*t1, 1, 0], [12*t0**2, 6*t0, 0, 0, 0], [12*t1**2, 6*t1, 0, 0, 0]])
#     B = np.array([x0, x1, dx0, dx1, ddx0, ddx1])

#     return np.linalg.solve(A, B)

# Genere les coefs d'un polynome d'ordre 5 (Implique de définir des accélérations et des jerk au WP)
def coef_polynoms_order5(t0, t1, x0, x1, dx0, dx1, ddx0, ddx1): 
    A = np.array([[t0**5, t0**4, t0**3, t0**2, t0, 1], [t1**5, t1**4, t1**3, t1**2, t1, 1], 
                    [5*t0**4, 4*t0**3, 3*t0**2, 2*t0, 1, 0], [5*t1**4, 4*t1**3, 3*t1**2, 2*t1, 1, 0], 
                    [20*t0**3, 12*t0**2, 6*t0, 2, 0, 0], [20*t1**3, 12*t1**2, 6*t1, 2, 0, 0]])
    B = np.array([x0, x1, dx0, dx1, ddx0, ddx1])

    return np.linalg.solve(A, B)

# Définitions des WPs
# timing, position, vitesse 
t0, x0, y0, dx0, dy0 = 0, 0, 0, 3, 3
t1, x1, y1, dx1, dy1 = 10, 10, 10, 1, 0

# Calcul les coefs des polynomes axe par axe
ax, bx, cx, dx = coef_polynoms_order3(t0, t1, x0, x1, dx0, dx1)
ay, by, cy, dy = coef_polynoms_order3(t0, t1, y0, y1, dy0, dy1)

# Propage les polynomes sur le temps
t = linspace(t0, t1, 100)
x = ax*t**3 + bx*t**2 + cx*t + dx
dotx = 3*ax*t**2 + 2*bx*t + cx

y = ay*t**3 + by*t**2 + cy*t + dy
doty = 3*ay*t**2 + 2*by*t + cy
figure()
plot(x, y)
plot([x0, x1], [y0, y1], 'ro')

figure()
plot(t, dotx)
plot(t, doty)
show()