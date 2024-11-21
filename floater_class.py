from boat_class import Boat
import simu_lib as sl

from roblib import *
from scilab2py import Scilab2Py, scilab

class Floater(Boat): 
    def __init__(self, t0, dt, tf, xmin,xmax,ymin,ymax, name="float", cmd_name="MFC_float", mission="sine", plot_flag=True, color='r'):
        #super().__init__(t0, dt, tf, xmin,xmax,ymin,ymax, "float", "MFC_float", misson_name, True)
        super().__init__(t0, dt, tf, xmin,xmax,ymin,ymax, name, cmd_name, mission, plot_flag, color)
        sci = Scilab2Py()
        scilab.getd('/home/loick/Documents/00-POSTDOC/09-FLOAT/01-Scilab_ED/bib-csm')

        Ns = 50# 50
        self.alpha = 1
        self.delta_u = 0

        self.store_F = np.empty
        self.store_alpha = np.empty

        self.rho,self.g,self.cx,self.β=1000,9.81,1.05,0.1
        self.m0 = 4.6
        self.V = 5e-3
        self.L = 1

        self.csm = scilab.csm_init(3, Ns, dt, self.alpha, "degre3", 0, 0)#ordre,Ns,Ts,alpha,methode,Kp,Kd

    def store(self):
        super().store()
        self.store_F = sl.put_in_vector(self.csm.F, self.store_F)
        self.store_alpha = sl.put_in_vector(self.alpha, self.store_alpha)


    def plot_continuous(self):
        clear(self.fig)
        self.fig.set_title(f'Time = {round(self.t, 1)}')
        sl.draw_buoy(self)  

    def plot_finish(self): 
        fig, [ax1, ax2] = plt.subplots(2, 1, figsize=(5, 5))
        ax1.set_title("F")
        ax1.plot(self.store_t, self.store_F)
        ax1.grid(True)
        ax2.set_title("alpha")
        ax2.plot(self.store_t, self.store_alpha)
        ax2.grid(True)
