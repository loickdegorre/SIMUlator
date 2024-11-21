from boat_class import Boat
import simu_lib as sl

from roblib import *

class Sailboat(Boat):
    def __init__(self, t0, dt, tf, xmin,xmax,ymin,ymax, cmd_name): #, name="USV", cmd_name="default", mission="line", plot_flag=False):
        super().__init__(t0, dt, tf, xmin,xmax,ymin,ymax, "sailboat", cmd_name, "line", True)

        # sailboat params
        self.psi_wind, self.a_wind = pi/4, 3
        self.theta_bar = 0
        self.theta_voulu = 0
        self.delta_s, self.q = 0, 1
        self.r, self.zeta = 5, pi/3

        self.Re = 20 # Steering radius # To be checked

        self.N_visu_X, self.N_visu_Y, self.N_control = [], [], []
        AB = self.B-self.A
        self.vhat = 50*(AB)/norm(AB) #  np.array([[100, 100]]).T # Uniform field 
        n_ort = array([[-1*AB[1,0]],[AB[0,0]]])
        n_ort = n_ort/norm(n_ort)
        self.N = n_ort@n_ort.T
        self.obstacles = [[78.67650616, 81.70578604, 8.135249170987722, 50.78311894832775],
                          [67.4086331 , 14.54589553, 2.9594863687447392, 52.63383402534154],
                          [79.26016865, 18.41405933, 0.7711233290236452, 19.38677498734013],
                          [95.88067909, 45.29571236, 12.916927872266527, 47.49827716533487],
                          [32.7391846 , 50.06268223, 6.390945169185061, 52.47926308522325]]
        # self.obstacles = [(np.array([[30], [50]]),5, 20)] #, 
        # #                   #(np.array([[30], [30]]),5, 50)]
        self.obstacles = [[30, 30 ,5, 20]] #, 
                          #(np.array([[30], [30]]),5, 50)]

        self.detected_obstacles = []

    def plot_finish(self): 

        AB = np.linspace(self.A, self.B, len(self.store_t))

        fig0, ax0 = plt.subplots(1, 1, figsize=(5, 5))
        ax0.plot(self.store_X[0,:].flatten(), self.store_X[1,:].flatten(), self.color)
        ax0.plot(AB[:,0], AB[:,1], '--k')
        for i in range(0, len(self.obstacles), 1):
            obs = self.obstacles[i]
            # ax0.add_patch(plt.Circle((obs[0][0], obs[0][1]), obs[1], color='k', fill=False))
            ax0.add_patch(plt.Circle(obs[0:2], obs[2], color='k', fill=False))


        ax0.grid()


        fig1, ax1 = plt.subplots(3, 1, figsize=(5, 5))

        # ax1[0].plot(self.store_t, line_points[:, 0], '--r')
        # ax1[1].plot(self.store_t, line_points[:, 1], '--r')
        # ax1[2].plot(self.store_t, angle(line_points[:, 0] + 1j * line_points[:, 1]), '--r')
        ax1[0].plot(self.store_t, self.store_X[0,:].flatten(), self.color)
        # ax1[0].plot(self.store_t, AB[:,0]*ones(len(self.store_t)), '--k')
        ax1[0].grid(True)
        ax1[1].plot(self.store_t, self.store_X[1,:].flatten(), self.color)
        # ax1[1].plot(self.store_t, AB[:,1]*ones(len(self.store_t)), '--k')
        ax1[1].grid(True)

        ax1[2].plot(self.store_t, self.store_X[2,:].flatten(), self.color)
        ax1[2].plot(self.store_t, angle(self.B-self.A)*ones(len(self.store_t)), '--k')
        ax1[2].grid(True)


        # if len(self.U) > 1:
        #     fig2, ax2 = plt.subplots(len(self.U), 1, figsize=(5, 5))
        #     for i in range(len(self.U)): 
        #         ax2[i].plot(self.store_t, self.store_U[i,:].flatten())
        #         ax2[i].grid(True)
        # else: 
        #     fig2, ax2 = plt.subplots(len(self.U), 1, figsize=(5, 5))
        #     ax2.plot(self.store_t, self.store_U)
        #     ax2.grid(True)
        # show() 

    def plot_continuous(self):
        a, b, r = self.A, self.B, self.r

        phi = angle(b-a)
        psi = self.psi_wind

        clear(self.fig)
        self.fig.grid()
        self.fig.set_title(f'Sailboat - Time = {round(self.t, 1)}')
        draw_sailboat(self.X, self.delta_s, self.U[0,0], self.psi_wind, self.a_wind)
        
        plot(self.A[0], self.A[1], 'go')
        plot(self.B[0], self.B[1], 'mo')
        plot([a[0,0], b[0,0]], [a[1,0], b[1,0]], 'k') 
        plot([a[0,0]+r*cos(phi+pi/2), b[0,0]+r*cos(phi+pi/2)], [a[1,0]+r*sin(phi+pi/2), b[1,0]+r*sin(phi+pi/2)], '--k')
        plot([a[0,0]+r*cos(phi-pi/2), b[0,0]+r*cos(phi-pi/2)], [a[1,0]+r*sin(phi-pi/2), b[1,0]+r*sin(phi-pi/2)], '--k')
        
        plot([self.X[0,0], self.X[0,0] + 20*cos(psi-pi+self.zeta)], [self.X[1,0], self.X[1,0] +20*sin(psi-pi+self.zeta)], '--k')
        plot([self.X[0,0], self.X[0,0] + 20*cos(psi-pi-self.zeta)], [self.X[1,0], self.X[1,0] +20*sin(psi-pi-self.zeta)], '--k')
        plot([self.X[0,0], self.X[0,0] + 20*cos(self.X[2,0])], [self.X[1,0], self.X[1,0] +20*sin(self.X[2,0])], '--g')
        plot([self.X[0,0], self.X[0,0] + 20*cos(self.theta_voulu)], [self.X[1,0], self.X[1,0] + 20*sin(self.theta_voulu)], '--b')
        plot([self.X[0,0], self.X[0,0] + 20*cos(self.theta_bar)], [self.X[1,0], self.X[1,0] + 20*sin(self.theta_bar)], '--r')


        # for obs in self.obstacles:
        for i in range(0, len(self.obstacles), 1):
            obs=self.obstacles[i]
            # self.fig.add_patch(plt.Circle((obs[0][0], obs[0][1]), obs[1], color='k', fill=False))
            self.fig.add_patch(plt.Circle(obs[0:2], obs[2], color='k', fill=False))
            self.fig.add_patch(plt.Circle(obs[0:2], obs[2]+self.Re, color='g', fill=False))

        if len(self.detected_obstacles) > 0:
            X_field = arange(self.xmin,self.xmax,5)
            Y_field = arange(self.ymin,self.ymax,5)
            P1, P2 = meshgrid(X_field,Y_field)
            for i in range(0, len(self.detected_obstacles), 1):
                obs = self.detected_obstacles[i]
                self.fig.add_patch(plt.Circle(obs[0:2], obs[2], color='r', fill=False))
            N_x , N_y = np.sum(self.N_visu_X, axis = 0), np.sum(self.N_visu_Y, axis = 0)
            VX = N_x/(sqrt(N_x**2 + N_y**2))
            VY = N_y/(sqrt(N_x**2 + N_y**2))
            quiver(P1, P2, VX, VY)


    def PotentialField(self):
        '''
            Computes the potential field associated with the obstacles, the line (and the wind)
            N_obs_control is the vector relative to 1 obstacle, calculated at the current position of the obstacle and used for control
            N_control is the sum of all N_obs_control + the line ( + the wind)
            N_visu_X and N_visu_Y are used only for plotting, they represent the field calculated at each point of the grid
        '''
        X_field = arange(self.xmin,self.xmax,5)
        Y_field = arange(self.ymin,self.ymax,5)
        P1, P2 = meshgrid(X_field,Y_field)

        N_visu_obst_X, N_visu_obst_Y, N_obs_control = 0, 0, 0


        # Vector field associated with the line - Uniform field + Field pushing to line
        N_line_control = self.vhat # 
        N_visu_line_X = N_line_control[0,0]*ones(P1.shape)
        N_visu_line_Y = N_line_control[1,0]*ones(P1.shape)
        self.N_control.append(N_line_control)
        self.N_visu_X.append(N_visu_line_X)
        self.N_visu_Y.append(N_visu_line_Y)

        # N_back_to_line_control = 10*self.N@(self.A - self.X[0:2])
        # self.N_control.append(N_back_to_line_control)
        # --- Trouver une solution pour le print
        # N_visu_back_to_line_X = N_back_to_line_control[0,0]*ones(P1.shape)
        # N_visu_back_to_line_Y = N_back_to_line_control[1,0]*ones(P1.shape)
        # self.N_visu_X.append(N_visu_back_to_line_X)
        # self.N_visu_Y.append(N_visu_back_to_line_Y)

        # Vector field associated with the wind
        N_wind_control = 50*array([[cos(self.psi_wind)],[sin(self.psi_wind)]]) # Should it depend on the wind speed ?
        self.N_control.append(N_wind_control)
        N_wind_visu_X = N_wind_control[0,0]*ones(P1.shape)
        N_wind_visu_Y = N_wind_control[1,0]*ones(P1.shape)
        self.N_visu_X.append(N_wind_visu_X)
        self.N_visu_Y.append(N_wind_visu_Y)


        # Vector field associated with the obstacles
        for i in range(0, len(self.detected_obstacles), 1):
            obs = self.detected_obstacles[i] #self.obstacles: 
            R = obs[2]
            q = obs[0:2]#obs[0]

            sigma = R+self.Re
            K = 850000*sigma/2

            T1, T2 = sl.findTangentPoint(q, self.X, R)
            plot(T1[0], T1[1], "or")
            plot(T2[0], T2[1], "or")

            # Repulsive field components associated with the obstacle - Gaussian field
            z = sl.gaus2d(P1, P2, q, sigma)

            z_control_tmp = sl.gaus2d(self.X[0] , self.X[1], q, sigma)
            N_obs_control = N_obs_control -K*array([ -((self.X[0] - q[0])/sigma**2)*z_control_tmp, -((self.X[1] - q[1])/sigma**2)*z_control_tmp])
            
            N_visu_obst_X = N_visu_obst_X + K*((P1 - q[0])/sigma**2)*z
            N_visu_obst_Y = N_visu_obst_Y + K*((P2 - q[1])/sigma**2)*z

            # Adds repulsive field associated with the tangent points
            gain = 3000*R  
            # T1
            Nq_control = gain*(self.X[0:2] - T1)/(norm((self.X[0:2] - T1))**3) 
            N_obs_control = N_obs_control + Nq_control

            Vect_visu = array([P1 - T1[0], P2 - T1[1]])
            N_visu_obst_X = N_visu_obst_X + gain*Vect_visu[0]/(sqrt(Vect_visu[0]**2 + Vect_visu[1]**2)**(3)) 
            N_visu_obst_Y = N_visu_obst_Y + gain*Vect_visu[1]/(sqrt(Vect_visu[0]**2 + Vect_visu[1]**2)**(3)) 

            # T2
            Nq_control = gain*(self.X[0:2] - T2)/(norm((self.X[0:2] - T2))**3) 
            N_obs_control = N_obs_control + Nq_control
            
            Vect_visu = array([P1 - T2[0], P2 - T2[1]])
            N_visu_obst_X = N_visu_obst_X + gain*Vect_visu[0]/(sqrt(Vect_visu[0]**2 + Vect_visu[1]**2)**(3)) 
            N_visu_obst_Y = N_visu_obst_Y + gain*Vect_visu[1]/(sqrt(Vect_visu[0]**2 + Vect_visu[1]**2)**(3)) 


            self.N_visu_X.append(N_visu_obst_X)
            self.N_visu_Y.append(N_visu_obst_Y)
            self.N_control.append(N_obs_control)



    def SeekObstacles(self): 
        '''
            Detects if the boat is close to an obstacle and adds it to the list of detected obstacles
        '''
        temp = []
        for i in range(0, len(self.obstacles), 1):
            obs = self.obstacles[i]
            R = obs[2]
            q = obs[0:2]
            if norm(self.X[0:2].T - q) <= R + self.Re:
                self.detected_obstacles.append(obs)
            else: 
                temp.append(obs)
        self.obstacles = temp
        # for obs in self.obstacles:
        #     R = obs[1]
        #     q = obs[0]
        #     if norm(self.X[0:2] - q) < R + self.Re:
        #         self.detected_obstacles.append(obs)
        #         self.obstacles = [x for x in self.obstacles if x[0:2] != obs[0:2]]

    def ValidateObstacle(self): 
        temp = []
        for i in range(0, len(self.detected_obstacles), 1):
            ob = self.detected_obstacles[i]
            if dot((self.B-self.A).T, self.B - self.X[0:2])<= dot(self.B.T-ob[0:2], (self.B-self.A)) or norm(self.X[0:2].T - ob[0:2]) > 3*(ob[2]+self.Re) : #  or 
                self.obstacles.append(ob)
                
            else: 
                temp.append(ob)
        self.detected_obstacles = temp


    def loop(self):
        self.store_t = np.arange(self.t0, self.tf, self.dt)

        for t in self.store_t:
            self.t = t

            # self.detected_obstacles = []
            self.SeekObstacles()
            # print(len(self.detected_obstacles))
            self.ValidateObstacle()

            self.N_visu_X, self.N_visu_Y, self.N_control = [], [], []
            self.PotentialField()

            self.U = self.cmd_func(self)

            self.X = self.X + self.model_func(self)*self.dt

            self.update_mission_func(self)
            
            self.store()
            if self.plot_flag: self.plot_continuous()

            # print(f"Time {t}")
            # print(self)
        print('FINISHED')
        self.plot_finish()
        show(block=True)

    