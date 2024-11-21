import simu_lib as sl
import cmd_lib as cl
import model_lib as ml

# import sys
# sys.path.append('/home/loick/Documents/00-POSTDOC/roblib_package')
from roblib import *

global xmin,xmax,ymin,ymax

class Boat:
    def __init__(self, t0, dt, tf, xmin,xmax,ymin,ymax, name="USV", cmd_name="default", mission="line", plot_flag=True, color='b'):
        self.t0, self.dt, self.tf = t0, dt, tf 
        self.name = name
        self.cmd_name = cmd_name
        self.mission_name = mission
        self.plot_flag = plot_flag

        self.xmin, self.xmax, self.ymin, self.ymax = xmin, xmax, ymin, ymax

        self.color = color

        self.X = np.empty
        self.dX = np.empty
        self.Xd = np.empty
        self.dXd = np.empty
        self.ddXd = np.empty
        self.e = np.empty
        self.U = np.empty
        self.t = t0
        self.i = 0 # current index in a priori stored vectors

        self.A, self.B = np.empty, np.empty # path following checkpoints

        self.waypoints = np.empty

        # Storage
        self.store_X = np.empty
        self.store_U = np.empty
        self.store_t = np.empty
        self.store_Xd = np.empty 
        self.store_dXd = np.empty
        self.store_ddXd = np.empty
        self.store_e = np.empty

        # Build init function from name
        init_func_name = f"init_{self.name}"
        print(f" Init Func = {init_func_name}")
        self.init_func = getattr(sl, init_func_name, sl.init_default)  
        self.init_func(self)

        # Build model and command functions from name
        model_func_name = f"f_{self.name}"
        print(f" Model Func = {model_func_name}")
        self.model_func = getattr(ml, model_func_name, ml.f_default)

        cmd_func_name = f"cmd_{self.cmd_name}"
        print(f" Cmd func = {cmd_func_name}")
        self.cmd_func = getattr(cl, cmd_func_name, cl.cmd_default)

        mission_name = f"init_mission_{self.mission_name}"
        print(f" Mission = {mission_name}")
        self.init_mission_func = getattr(sl, mission_name, sl.init_mission_default)
        self.init_mission_func(self)
        self.update_mission_func = getattr(sl, f"update_mission_{self.mission_name}", sl.update_mission_default)


        # Figure, real time plotting
        if self.plot_flag: 
            self.fig = init_figure(xmin,xmax,ymin,ymax)
            xlim([xmin, xmax])
            ylim([ymin, ymax])


    def __str__(self):
        return f"{self.name} state: {self.X} command: {self.U}"
    
    def set_wind(self, psi_wind, a_wind):
        self.psi_wind = psi_wind
        self.a_wind = a_wind
    
    def plot_continuous(self):
        clear(self.fig)
        self.fig.grid()
        self.fig.set_title(f'Time = {round(self.t, 1)}')
        if len(self.X) > 3:
            plot(self.X[0], self.X[1], self.X[2], 'ro')
        else:
            plot(self.X[0], self.X[1], 'ro')
            plot([self.X[0], self.X[0]+cos(self.X[2])], [self.X[1], self.X[1]+sin(self.X[2])], '-r')

            if self.mission_name == "line": 
                plot(self.A[0], self.A[1], 'go')
                plot(self.B[0], self.B[1], 'mo')
                plot([self.A[0,0], self.B[0,0]], [self.A[1,0], self.B[1,0]], 'k') 

            if self.mission_name == "traj1" or self.mission_name == "traj2": 
                plot(self.Xd[0], self.Xd[1], 'g*')
                plot(self.store_Xd[0,:], self.store_Xd[1,:], '--k')

    def plot_finish(self): 
        fig1, ax1 = plt.subplots(len(self.X), 1, figsize=(5, 5))
        for i in range(len(self.X)): 
            ax1[i].plot(self.store_t, self.store_X[i,:].flatten(), self.color)
            ax1[i].plot(self.store_t, self.store_Xd[i,:].flatten(), '-k')
            ax1[i].grid(True)

        if len(self.U) > 1:
            fig2, ax2 = plt.subplots(len(self.U), 1, figsize=(5, 5))
            for i in range(len(self.U)): 
                ax2[i].plot(self.store_t, self.store_U[i,:].flatten())
                ax2[i].grid(True)
        else: 
            fig2, ax2 = plt.subplots(len(self.U), 1, figsize=(5, 5))
            ax2.plot(self.store_t, self.store_U)
            ax2.grid(True)

    def store(self): 
        # self.store_t.append(self.t)
        # self.store_X = np.append(self.store_X, self.X)

        self.store_X = sl.put_in_vector(self.X, self.store_X)
        self.store_U = sl.put_in_vector(self.U, self.store_U)
        # self.store_Xd = sl.put_in_vector(self.Xd, self.store_Xd)

        # if self.store_X is np.empty:
        #     self.store_X = self.X
        # else:
        #     self.store_X = hstack((self.store_X, self.X))
        # if self.store_U is np.empty:
        #     self.store_U = self.U
        # else:
        #     self.store_U = hstack((self.store_U, self.U))




    def loop(self):
        self.store_t = np.arange(self.t0, self.tf, self.dt)

        for t in self.store_t:
            self.t = t

            self.U = self.cmd_func(self)

            # print(self.model_func(self))
            self.dX = self.model_func(self)
            self.X = self.X + self.dX*self.dt

            self.update_mission_func(self)
            
            self.store()

            if self.plot_flag: self.plot_continuous()

            # print(f"Time {t}")
            # print(self)
        print('FINISHED')
        self.plot_finish()



    
