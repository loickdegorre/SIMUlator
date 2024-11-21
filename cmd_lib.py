from roblib import *
from scilab2py import Scilab2Py, scilab


def cmd_default(boat):
    return 0

def cmd_LOS(boat): 
    if boat.mission_name == "line": 
        x, y, psi = boat.X.flatten()
        a, b, m= boat.A, boat.B, boat.X[0:2]
        h = 1 #lookahead

        kp = 1

        e = det(hstack(((b-a)/norm(b-a),m-a)))

        psi_line = angle(b-a)
        psi_des = psi_line - arctan(e/h)

        u1 = 2
        u2 = kp*(psi_des-psi)

        return np.array([[u1, u2]]).T
    else: 
        return np.zeros(len(boat.U))
    
def cmd_PID1(boat): 
    if boat.mission_name == "traj1" or boat.mission_name == "traj2": 
        _, _, psi = boat.X.flatten()

        kp = 1

        boat.e = boat.Xd - boat.X[0:2]
        psi_des = np.arctan2(boat.e[1,0], boat.e[0,0])

        u1 = 2
        u2 = kp*(psi_des-psi)
        return np.array([[u1, u2]]).T
    
def cmd_sailboat(boat): 
    x, y, theta, v, w = boat.X.flatten()
    delta_r, delta_smax = boat.U.flatten()
    a, b = boat.A, boat.B
    r = boat.r

    m = np.array([[x],[y]])
    
    e = det(hstack(((b-a)/norm(b-a),m-a)))
    if abs(e) > r:
        boat.q = sign(e)
    
    φ = angle(b-a)
    
    theta_bar = φ - arctan(e/r)
    boat.theta_voulu = theta_bar

    if cos(boat.psi_wind-theta_bar) + cos(boat.zeta) < 0 or (abs(e) < r and cos(boat.psi_wind-φ)+cos(boat.zeta) < -0.1) :
        #theta_bar = -boat.psi_wind-boat.q*boat.zeta
        theta_bar = (boat.psi_wind-pi) - boat.q*boat.zeta
        
    boat.theta_bar = theta_bar
    δr=0.3*sawtooth(theta-theta_bar)
    δsmax=(pi/4)*(cos(boat.psi_wind-theta_bar)+1)
    u=array([[δr],[δsmax]])
    return u

def cmd_sailboat_obstacle(boat): 
    x, y, theta, _, _ = boat.X.flatten()
    a, b = boat.A, boat.B
    r = boat.r

    m = np.array([[x],[y]])
    
    e = det(hstack(((b-a)/norm(b-a),m-a)))

    if abs(e) > r:
        boat.q = sign(e)
    
    if len(boat.detected_obstacles) > 0:
        N_ctrl = np.sum(boat.N_control, axis=0) 
        theta_bar = arctan2(N_ctrl[1,0], N_ctrl[0,0])
        boat.theta_voulu = theta_bar

    else: 
        φ = angle(b-a)
        theta_bar = φ - arctan(e/r)
        boat.theta_voulu = theta_bar

        if cos(boat.psi_wind-theta_bar) + cos(boat.zeta) < 0 or (abs(e) < r and cos(boat.psi_wind-φ)+cos(boat.zeta) < -0.1) :
            # theta_bar = -boat.psi_wind - boat.q*boat.zeta
            theta_bar = (boat.psi_wind-pi) - boat.q*boat.zeta
        
    boat.theta_bar = theta_bar
    dr=0.3*sawtooth(theta-theta_bar)
    dsmax=(pi/4)*(cos(boat.psi_wind-theta_bar)+1)
    u=array([[dr],[dsmax]])
    return u

def cmd_MFC_float(boat): 
    scilab.getd('/home/loick/Documents/00-POSTDOC/09-FLOAT/01-Scilab_ED/bib-csm')

    z, dz, _ = boat.X.flatten()
    ddz = boat.dX[1,0]
    zd = boat.Xd[0,0] 
    dzd = boat.Xd[1,0]
    ddzd = boat.dXd[1,0]
    d3zd = boat.ddXd[1,0]

    e = zd -z
    de = dzd - dz
    dde = ddzd - ddz

    kdd, kd, kp = 5, 10, 10
    PDD = kdd*dde + kd*de + kp*e

    boat.alpha = 1
    # print(boat.alpha)

    boat.csm = scilab.csm_estiv(boat.csm, boat.t, z, boat.U, boat.alpha)

    u =  (d3zd + PDD - boat.csm.F)/boat.alpha
    return np.array([u])

def cmd_HEOL_float(boat): 
    scilab.getd('/home/loick/Documents/00-POSTDOC/09-FLOAT/01-Scilab_ED/bib-csm')

    z, dz, _ = boat.X.flatten()
    ddz = boat.dX[1,0]
    zd = boat.Xd[0,0] 
    dzd = boat.Xd[1,0]
    ddzd = boat.dXd[1,0]
    d3zd = boat.ddXd[1,0]

    e = zd -z
    de = dzd - dz
    dde = ddzd - ddz

    kdd, kd, kp = 5, 6, 2 #5, 3, 0.2
    PDD = kdd*dde + kd*de + kp*e

    rho,g,cx,β=boat.rho, boat.g, boat.cx, boat.β
    V = boat.V

    ud = (d3zd*(rho*V*g+cx*dzd*abs(dzd))/(g-ddzd) + cx*ddzd*abs(dzd))/(rho*(1-ddzd))

    boat.alpha = rho*(g-ddzd)**2/(rho*V*g+cx*dzd*abs(dzd)) # Calcul de alpha ne correspond pas au f utilisé dans la simu -> Changer f avec ordre 3 + caractéristiques de masse de flappy
    # print(boat.alpha)

    boat.csm = scilab.csm_estiv(boat.csm, boat.t, e, boat.delta_u, boat.alpha)

    boat.delta_u = (PDD - boat.csm.F)/boat.alpha # 
    
    # COMMENT ON GERE LE F* du désiré ??


    u =  ud + boat.delta_u
    return np.array([u])