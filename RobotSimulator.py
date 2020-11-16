import sympy as sym
from math import cos,sin,pi
from SymHelpers import *
import plotly.express as px
import pandas as pd
from Robot import Robot

color_map = {
    0:'blue',
    1:'red',
    2:'green',
    3:'orange'
}

class RobotSimulator:
    def __init__(self,robot:Robot):
        self.robot = robot
        self.data = {'x':[],'y':[], 'z':[],'joint':[],'t':[],'color':[]}
    
    def get_initial_positon(self,initial_joint_states):
        size = len(self.robot.frames)
        ts = [self.robot.sub_into_matrix(self.robot.get_T(i),initial_joint_states) for i in range(1,len(self.robot.frames)+1)]
        state_vector = []
        for i in range(size):
            pxyz = sym.Matrix([ts[i][0:3,3],[0],[0],[0]])
            state_vector.append([pxyz])
        return sym.Matrix(state_vector)

    def get_velocities(self,state_dict,velocity_dict):
        J = self.robot.get_all_Jacobians()
        evaluated_J = self.robot.sub_into_matrix(J,state_dict)
        joint_vels = sym.Matrix([velocity_dict[str(x)] for x in self.robot.syms])
        velocity = evaluated_J*joint_vels
        return velocity
    
    def update_states(self,states,velocities,timestep):
        for key in states:
            states[key]+=velocities[key]*timestep
        return states

    def save_state(self,xyzs,time):
        size = len(self.robot.frames)
        for i in range(size):
            state_i = xyzs[6*i:(i+1)*6]
            self.data['x'].append(str(state_i[0]))
            self.data['y'].append(str(state_i[1]))
            self.data['z'].append(str(state_i[2]))
            self.data['joint'].append(str(i))
            self.data['t'].append(str(time))
            self.data['color'].append(color_map[i])

    def simulate(self,initial_joint_states,velocity_dict,loops,timestep):
        joints = initial_joint_states
        xyzs = self.get_initial_positon(initial_joint_states)
        self.save_state(xyzs,0)
        for i in range(loops):
            joints = self.update_states(joints,velocity_dict,timestep)
            vels = self.get_velocities(joints,velocity_dict)
            xyzs += vels*timestep
            self.save_state(xyzs,i*timestep)

    def write_data_to_csv(self,csv_filename):
        df = pd.DataFrame(self.data)
        df.to_csv(csv_filename)
    
    def plot_data(self,axis_dict):
        df = pd.DataFrame(self.data)
        max_x = max([float(x) for x in self.data[axis_dict["x"]]])
        max_y = max([float(y) for y in self.data[axis_dict["y"]]])
        fig = px.scatter(df,x=axis_dict["x"],y=axis_dict["y"],animation_frame="t",animation_group="joint",color="color",range_x=[-max_x-1,max_x+1],range_y=[-max_y-1,max_y+1])
        fig.show()        
