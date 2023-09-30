###########################This file will hold all the functions for a cart pole visual simulation###################
from vpython import *
import math
#Parameters
g = 9.8 # m/s^2
i = 0  # For time text box
t = 0
dt = 0.01
Xc = 0 #initial position of cart x cordinate
Xcdot = 0 #initial velocity of cart
thetadot = 0 # inital angular velocity


class Visual_Model:


    #This function is for initialising the parameters of this class
    def __init__(self, Length = 2, mass_p = 0.1, mass_c = 10, Force = 0, theta = 7):
        self.L = Length # length of rod
        self.m = mass_p # mass of pendelum in grams
        self.M = mass_c # mass of cart in grams
        self.Force = Force # force input
        self.theta = -math.radians(theta) #the math library uses radians and considers ccw positive. I want to use deg and have cw positive
        self.Text_Theta = None #Initializing it here so its public and accessible to all methods
        self.cart = None
        self.Text_Time = None #Initializing it here so its public and accessible to all methods
        self.PendelumMass = None
        rod = None
        #Graph
        self.f_Pendulum_Pos = None
        self.f_Pendulum_Vel= None
        self.f_Pendulum_A = None
        self.f_Cart_Pos = None
        self.f_Cart_V = None 


    #This function will crete thevisual model of the cart and pendelum along with the time and theta text boxes
    def Visual_Modle_Make(self):
        Xp = Xc - self.L*sin(self.theta) #original
        Yp = self.L*cos(self.theta)
        ground = box(color = color.white, pos = vector(0,0,0), size = vector(30,0.02,10))
        self.cart = box(color = color.blue, pos = vector(Xc,0.52,0.5), size = vector(1,1,1))
        self.PendelumMass = sphere(color = color.red, radius = 0.1, make_trail = True, pos = vector(Xp,Yp,0.5))
        self.rod = cylinder(color = color.green,  radius = 0.05, pos = self.cart.pos, axis = self.PendelumMass.pos-self.cart.pos)
        self.Text_Theta = label(pos=vec(-5, 4, 0), text='Theta: 0', height=15)
        self.Text_Time = label(pos=vec(5, 4, 0), text='Time: 0 Sec', height=15)
        print("hello")


    #This function will run the simulation
    def run_visual_sim(self,run_time = 10):
        g = 9.8 # m/s^2
        i = 0  # For time text box
        t = 0
        dt = 0.01
        Xc = 0 #initial position of cart x cordinate
        Xcdot = 0 #initial velocity of cart
        thetadot = 0 # inital angular velocity
        
        while t < run_time:
            #Visual Simulation
            self.Text_Theta.text = f'Theta: {round(-math.degrees(self.theta),1)}'
            i = i + 1 #Update time text box
            self.Text_Time.text = f'Time: {i/100} Sec' 
            rate(100) # Max number of calculations
            #print(thetaddot, thetadot)
            #Xcddot = (Force + (m*g*theta))/M   #Equation for accelartaion of cart (Linear)
            Xcddot = (self.Force + self.m*g*sin(self.theta)*cos(self.theta) - self.m*self.L*thetadot*thetadot*sin(self.theta))/(self.M + self.m + self.m*cos(self.theta)*cos(self.theta))  #Non-Linear
            Xcdot = Xcdot + Xcddot*dt  # new velocity of cart
            Xc = Xc + Xcdot*dt  # New position of cart
            self.cart.pos = vector(Xc,0.52,0.5) # Update position of the cart
            #thetaddot = (Force + g * theta * (M + m))/(L * M) #Linear
            thetaddot = (self.Force*cos(self.theta) + (self.M + self.m)*g*sin(self.theta) - self.m*self.L*thetadot*thetadot*sin(self.theta)*cos(self.theta))/(self.L*(self.M + self.m) - self.m*self.L*cos(self.theta)*cos(self.theta)) #Non-Linear
            thetadot = thetadot + thetaddot*dt
            self.theta = self.theta + thetadot*dt
            Xp = Xc - self.L * sin(self.theta)
            Yp = self.L*cos(self.theta) 
            self.PendelumMass.pos = vector(Xp,Yp,0.5)
            self.rod.pos = self.cart.pos
            self.rod.axis = self.PendelumMass.pos-self.cart.pos
            t = t + dt  #update time
            


    def graph_model(self):
        #Graphs
        #graph of pendulum angle
        self.Graph_P_Pos = graph(title = 'Pendulum Position vs Time', width = 400, height = 200, xtitle = "Time [s]", ytitle = "Angle [deg]")
        self.f_Pendulum_Pos = gcurve(color=color.blue)
        # graph of pendulum velocity
        self.Graph_P_V = graph(title = "Pendulum Velocity vs Time", width = 400, height = 200, xtitle = "Time [s]", ytitle = "Angular velocity [deg/s]")
        self.f_Pendulum_Vel = gcurve(color = color.red)
        #graph of pendulum accelaration
        self.Graph_P_A = graph(title = 'Pendulum Accelaration vs Time', width = 400, height = 200, xtitle = "Time [s]", ytitle = "Angular Accelaration [deg/s^2]")
        self.f_Pendulum_A = gcurve(color=color.orange)
        # graph of cart position
        self.Graph_C_P = graph(title = "Cart Position vs Time", width = 400, height = 200, xtitle = "Time [s]", ytitle = "Position [m]")
        self.f_Cart_Pos = gcurve(color = color.purple)
        # graph of cart velocity
        self.Graph_C_V = graph(title = "Cart Velocity vs Time", width = 400, height = 200, xtitle = "Time [s]", ytitle = "Velocity [m/s]")
        self.f_Cart_V = gcurve(color = color.black)

    def run_graph_sim(self, run_time = 10):
        g = 9.8 # m/s^2
        i = 0  # For time text box
        t = 0
        dt = 0.01
        Xc = 0 #initial position of cart x cordinate
        Xcdot = 0 #initial velocity of cart
        thetadot = 0 # inital angular velocity

        while t < run_time:
            
            rate(100) # Max number of calculations
            #print(thetaddot, thetadot)
            #Xcddot = (Force + (m*g*theta))/M   #Equation for accelartaion of cart (Linear)
            Xcddot = (self.Force + self.m*g*sin(self.theta)*cos(self.theta) - self.m*self.L*thetadot*thetadot*sin(self.theta))/(self.M + self.m + self.m*cos(self.theta)*cos(self.theta))  #Non-Linear
            Xcdot = Xcdot + Xcddot*dt  # new velocity of cart
            Xc = Xc + Xcdot*dt  # New position of cart
            #self.cart.pos = vector(Xc,0.52,0.5) # Update position of the cart
            #thetaddot = (Force + g * theta * (M + m))/(L * M) #Linear
            thetaddot = (self.Force*cos(self.theta) + (self.M + self.m)*g*sin(self.theta) - self.m*self.L*thetadot*thetadot*sin(self.theta)*cos(self.theta))/(self.L*(self.M + self.m) - self.m*self.L*cos(self.theta)*cos(self.theta)) #Non-Linear
            thetadot = thetadot + thetaddot*dt
            self.theta = self.theta + thetadot*dt
            Xp = Xc - self.L * sin(self.theta)
            Yp = self.L*cos(self.theta) 
            #self.PendelumMass.pos = vector(Xp,Yp,0.5)
            #self.rod.pos = self.cart.pos
            #self.rod.axis = self.PendelumMass.pos-self.cart.pos


            #Graph
            self.f_Pendulum_Pos.plot(t,-math.degrees(self.theta)) #Plot theta in degrees
            self.f_Pendulum_Vel.plot(t, -math.degrees(thetadot)) #Plot thetadot
            self.f_Pendulum_A.plot(t, -math.degrees(thetaddot)) #Plot thetadot
            self.f_Cart_Pos.plot(t, Xc) #Plot cart position
            self.f_Cart_V.plot(t, Xcdot) #Plot cart velocity

            t = t + dt  #update time


    

    def test(self):
        print("teset")


        
        