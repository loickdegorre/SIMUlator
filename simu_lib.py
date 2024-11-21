from roblib import *

def init_USV(boat): 
    boat.X = np.array([[0, 0, 0, 0, 0, 0]]).T
    boat.dX = np.array([[0, 0, 0, 0, 0, 0]]).T
    boat.U = np.array([[0, 0, 0, 0]]).T

def init_default(boat):
    boat.X = np.array([[0]]).T
    boat.dX = np.array([[0]]).T
    boat.U = np.array([[0]]).T

def init_dubins(boat): 
    boat.X = np.array([[1, 0, 0]]).T
    boat.dX = np.array([[1, 0, 0]]).T
    boat.U = np.array([[0, 0]]).T

def init_sailboat(boat):
    boat.X = np.array([[0, 0, 0, 0, 0]]).T
    boat.dX = np.array([[0, 0, 0, 0, 0]]).T
    boat.U = np.array([[0, 0]]).T

def init_float(boat):
    boat.X = np.array([[1, 0, 0]]).T # z, dz, b
    boat.dX = np.array([[0, 0, 0]]).T 
    boat.U = np.array([[0]]).T

def init_float_2(boat):
    boat.X = np.array([[1, 0, 0]]).T # z, dz, mb
    boat.dX = np.array([[0, 0, 0]]).T 
    boat.U = np.array([[0]]).T

def init_float_3(boat):
    boat.X = np.array([[1, 0, 0]]).T # z, dz, ddz
    boat.dX = np.array([[0, 0, 0]]).T 
    boat.U = np.array([[0]]).T


def init_mission_default(boat): 
    pass

def init_mission_setpoint(boat): 
    boat.Xd = np.array([[3, 0, 0]]).T
    boat.dXd = np.array([[0, 0, 0]]).T
    boat.ddXd = np.array([[0, 0, 0]]).T

def init_mission_slope(boat): 
    boat.Xd = np.array([[3, 0.5, 0]]).T
    boat.dXd = np.array([[0, 0, 0]]).T
    boat.ddXd = np.array([[0, 0, 0]]).T

def update_mission_slope(boat):
    zd, dzd, _ = boat.Xd.flatten()

    if boat.t == 10 or boat.t == 20:
        dzd = -dzd
    zd = zd + dzd*boat.dt
    boat.Xd = np.array([[zd, dzd, 0]]).T



def init_mission_sine(boat): 
    boat.Xd = np.array([[3, 0, 0]]).T
    boat.dXd = np.array([[0, 0, 0]]).T
    boat.ddXd = np.array([[0, 0, 0]]).T

def update_mission_sine(boat): 
    zd, dzd, _ = boat.Xd.flatten()
    
    zd = 3 + sin(0.5*boat.t)
    dzd = 0.5*cos(0.5*boat.t)
    ddzd = -0.25*sin(0.5*boat.t)
    d3zd = -0.125*cos(0.5*boat.t)

    boat.Xd = np.array([[zd, dzd, ddzd]]).T
    boat.dXd = np.array([[dzd, ddzd, d3zd]]).T
    boat.ddXd = np.array([[ddzd, d3zd, 0]]).T

def init_mission_line(boat): 
    if len(boat.X) == 3 or len(boat.X) == 5: # surface vessel or sailboat
        boat.A = np.array([[0, 0]]).T
        boat.B = np.array([[100, 100]]).T
    elif len(boat.X) == 6 : # 3D vessel
        boat.A = np.array([[0, 0, 0]]).T
        boat.B = np.array([[10, 10, 10]]).T
    else: 
        print("Mission line ill-defined")
        pass

def init_mission_traj1(boat): 
    # First order - straight line
    boat.A = np.array([[0 ,0]]).T
    boat.B = np.array([[15, 15]]).T
    boat.store_Xd = np.linspace(boat.A, boat.B, int((boat.tf-boat.t0)/boat.dt))

    boat.Xd = boat.store_Xd[0]

def init_mission_traj2(boat):
    boat.waypoints = np.array([[boat.t0, 0, 0, 0, 0], [20, 20, 20, 2, 0], [boat.tf, 30, 10, 0, 0]]) #t, x, y, dx, dy
    boat.Xd = boat.waypoints[0][1:3]
    for i in range(len(boat.waypoints)-1):
        t0, x0, y0, dx0, dy0 = boat.waypoints[i]
        t1, x1, y1, dx1, dy1 = boat.waypoints[i+1]

        ax, bx, cx, dx = coef_polynoms_order3(t0, t1, x0, x1, dx0, dx1)
        ay, by, cy, dy = coef_polynoms_order3(t0, t1, y0, y1, dy0, dy1)

        t = np.linspace(t0, t1, int((t1-t0)/boat.dt))
        x = ax*t**3 + bx*t**2 + cx*t + dx
        y = ay*t**3 + by*t**2 + cy*t + dy

        dotx = 3*ax*t**2 + 2*bx*t + cx
        doty = 3*ay*t**2 + 2*by*t + cy
        psi = arctan2(doty, dotx)

        if i == 0:
            boat.store_Xd = np.array([x, y, psi])
        else:
            boat.store_Xd = np.hstack((boat.store_Xd, np.array([x, y, psi])))

def update_mission_traj2(boat): 
    if boat.i < max(boat.store_Xd.shape)-1: 
        boat.i += 1
        boat.Xd = boat.store_Xd[:,boat.i]





def update_mission_default(boat): 
    # boat.store_Xd = boat.Xd*np.arange(boat.t0, boat.tf, boat.dt)
    # boat.store_dXd = boat.dXd*np.arange(boat.t0, boat.tf, boat.dt)
    # boat.store_ddXd = boat.ddXd*np.arange(boat.t0, boat.tf, boat.dt)
    # boat.store_d3Xd = boat.d3Xd*np.arange(boat.t0, boat.tf, boat.dt)
    pass

def update_mission_traj1(boat): 
    if boat.i < len(boat.store_Xd)-1: 
        boat.i += 1
        boat.Xd = boat.store_Xd[boat.i]

def gaus2d(x, y, qhat, sigma):    
    tmp = ( ((x-qhat[0])**2)/(2*sigma**2) + ((y-qhat[1])**2)/(2*sigma**2))
    return exp(-tmp)/(2*pi*sigma**2) 

def findTangentPoint(qhat, x, R):
	x0 = qhat[0]; y0 = qhat[1]; xB = x[0]; yB = x[1];
	alpha = xB*xB + yB*yB + x0*x0 + y0*y0 -2*xB*x0 - 2*y0*yB -R*R
	
	slope1 = (x0*y0 + xB*yB - xB*y0 - x0*yB + R*alpha**0.5)/(-R*R + x0*x0 -2*x0*xB + xB*xB)
	xT1 = (-slope1*yB + x0 + slope1*slope1*xB + slope1*y0)/(1+slope1*slope1)
	yT1 = (slope1*x0 + yB + slope1*slope1*y0 - slope1*xB)/(1+slope1*slope1)
	
	slope2 = (x0*y0 + xB*yB - xB*y0 - x0*yB - R*alpha**0.5)/(-R*R + x0*x0 -2*x0*xB + xB*xB)
	xT2 = (-slope2*yB + x0 + slope2*slope2*xB + slope2*y0)/(1+slope2*slope2)
	yT2 = (slope2*x0 + yB + slope2*slope2*y0 - slope2*xB)/(1+slope2*slope2)
	
	return array([xT1, yT1]), array([xT2, yT2])

def draw_buoy(boat):

    x = boat.X.flatten()
    w = boat.Xd[0,0]
    ax = boat.fig
    ech = 5
    L = 1

    # x=x.flatten()
    plot([-10,10],[0,0],'black',linewidth=1)    
    d=x[0]
    P=array([[-ech,-1.8*ech],[ech,-1.8*ech],[ech,0],[-ech,0]])
    draw_polygon(ax,P,'blue')
    plot([   0,   L,  L,  L/2,   L/2,   L/2,  0,  0],
         [-L-d,-L-d, -d,   -d,   2-d,    -d, -d,-L-d],'black',linewidth=3)
    plot([-ech, ech],[-w,-w],'red',linewidth=1)
    b=-x[2]     
    P=array([[0,-L-d+L],[L,-L-d+L],[L,-L/2-L*b/2-d],[0,-L/2-L*b/2-d]])
    draw_polygon(ax,P,'white')
    

def put_in_vector(x, vec): 
    if vec is np.empty:
        return_vec = x
    else:
        return_vec = hstack((vec, x))
    return return_vec

def coef_polynoms_order3(t0, t1, x0, x1, dx0, dx1): 
    A = array([[t0**3, t0**2, t0, 1], [t1**3, t1**2, t1, 1], [3*t0**2, 2*t0, 1, 0], [3*t1**2, 2*t1, 1, 0]])
    B = array([x0, x1, dx0, dx1])

    return np.linalg.solve(A, B)


def coef_polynoms_order4(t0, t1, x0, x1, dx0, dx1, ddx0, ddx1): 
    A = array([[t0**4, t0**3, t0**2, t0, 1], [t1**4, t1**3, t1**2, t1, 1], [4*t0**3, 3*t0**2, 2*t0, 1, 0],
                [4*t1**3, 3*t1**2, 2*t1, 1, 0], [12*t0**2, 6*t0, 0, 0, 0], [12*t1**2, 6*t1, 0, 0, 0]])
    B = array([x0, x1, dx0, dx1, ddx0, ddx1])

    return np.linalg.solve(A, B)

def plot_end(boat_list): 
    for b in boat_list: 
        fig1, ax1 = plt.subplots(len(b.X), 1, figsize=(5, 5))
        for i in range(len(b.X)): 
            ax1[i].plot(b.store_t, b.store_X[i,:].flatten(), b.color)
            ax1[i].plot(b.store_t, b.store_Xd[i,:].flatten())
            ax1[i].grid(True)

        if len(b.U) > 1:
            fig2, ax2 = plt.subplots(len(b.U), 1, figsize=(5, 5))
            for i in range(len(b.U)): 
                ax2[i].plot(b.store_t, b.store_U[i,:].flatten())
                ax2[i].grid(True)
        else: 
            fig2, ax2 = plt.subplots(len(b.U), 1, figsize=(5, 5))
            ax2.plot(b.store_t, b.store_U)
            ax2.grid(True)

    show()