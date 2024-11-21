from roblib import *


def f_default(boat): 
    return 0

def f_dubins(boat): 
    _,_, psi = boat.X.flatten()
    u1, u2 = boat.U.flatten()
    
    return np.array([[u1*cos(psi), u1*sin(psi), u2]]).T

def f_sailboat(boat): 
    x, y, theta, v, w = boat.X.flatten()
    delta_r, delta_smax = boat.U.flatten()
    p0,p1,p2,p3,p4,p5,p6,p7,p8,p9 = 0.1,50,6000,1000,2000,1,1,2,300,10000 # SB parameters

    w_ap = np.array([[boat.a_wind*cos(boat.psi_wind-theta) - v],[boat.a_wind*sin(boat.psi_wind-theta)]])
    boat.psi_wind_ap = angle(w_ap)
    a_ap=norm(w_ap)
    sigma = cos(boat.psi_wind_ap) + cos(delta_smax)
    if sigma < 0 :
        boat.delta_s = pi + boat.psi_wind_ap
    else :
        boat.delta_s = -sign(sin(boat.psi_wind_ap))*delta_smax
    fr = p4*v*sin(delta_r)
    fs = p3*(a_ap**2)* sin(boat.delta_s-boat.psi_wind_ap)
    dx=v*cos(theta) + p0*boat.a_wind*cos(boat.psi_wind)
    dy=v*sin(theta) + p0*boat.a_wind*sin(boat.psi_wind)
    dv=(fs*sin(boat.delta_s)-fr*sin(delta_r)-p1*v**2)/p8
    dw=(fs*(p5-p6*cos(boat.delta_s)) - p7*fr*cos(delta_r) - p2*w*v)/p9
    xdot=array([[dx],[dy],[w],[dv],[dw]])
    return xdot

def f_float(boat):

    ρ,g,cx,β=boat.rho, boat.g, boat.cx, boat.β
    V = boat.V
    L = boat.L
    z, dz, b = boat.X.flatten()
    u = boat.U[0]

    ddz = g - (g*max(0,L+min(z,0)) + dz*abs(dz)*cx/2)/((1+β*b)*L) # update avec model ordre 3 / Model ordre 2 + equation de mb à la place de juste b ?

    return np.array([[dz, ddz, u]]).T

def f_float_2(boat):
    rho,g,cx,β=boat.rho, boat.g, boat.cx, boat.β
    V, L, m0 = boat.V, boat.L, boat.m0

    z, dz, mb = boat.X.flatten()
    u = boat.U[0]

    # ddz = g - (g*max(0,L+min(z,0)) + dz*abs(dz)*cx/2)/((1+β*b)*L) # update avec model ordre 3 / Model ordre 2 + equation de mb à la place de juste b ?

    ddz = ((m0+mb)*g - rho*V*g - cx*dz*abs(dz))/(m0+mb)
    return np.array([[dz, ddz, rho*u]]).T

def f_float_3(boat):
    rho,g,cx,β=boat.rho, boat.g, boat.cx, boat.β
    V, L, m0 = boat.V, boat.L, boat.m0

    z, dz, ddz = boat.X.flatten()
    u = boat.U[0]

    d3z = (g-ddz)*(rho*(g-ddz)*u - 2*cx*ddz*abs(dz))/(rho*V*g + cx*dz*abs(dz))

    return np.array([[dz, ddz, d3z]]).T