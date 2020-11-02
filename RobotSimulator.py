import sympy as sym
from math import cos,sin,pi
from SymHelpers import *
import plotly.express as px
import pandas as pd

color_map = {
    0:'blue',
    1:'red',
    2:'green',
    3:'orange'
}

class FrameParameters:
    def __init__(self,theta,theta_offset,d,d_offset,a,a_offset,alpha,alpha_offset):
        self.theta = theta
        self.theta_offset = theta_offset
        self.d = d
        self.d_offset = d_offset
        self.a = a
        self.a_offset = a_offset
        self.alpha = alpha
        self.alpha_offset = alpha_offset

    @classmethod
    def without_offset(cls,theta,d,a,alpha):
        return cls(theta,0,d,0,a,0,alpha,0)
    
    @classmethod
    def from_2d_array(cls,arr):
        value_arr = arr[0]
        offset_arr = arr[1]
        return cls(value_arr[0],offset_arr[0],value_arr[1],offset_arr[1],value_arr[2],offset_arr[2],value_arr[3],offset_arr[3])

    def get_syms(self):
        syms = []
        if type(self.theta) == str:
            syms.append(self.theta)
        if type(self.d) == str:
            syms.append(self.d)
        if type(self.a) == str:
            syms.append(self.a)
        if type(self.alpha) == str:
            syms.append(self.alpha)
        return syms

    def get_theta_expr(self):
        if type(self.theta) == str:
            if self.theta_offset ==0:
                return sym.Symbol(self.theta)
            elif self.theta_offset > 0:
                return sym.Symbol(self.theta) + self.theta_offset
        if type(self.theta) == int or type(self.theta) == float:
            return self.theta*pi/180 + self.theta_offset

    def get_d_expr(self):
        if type(self.d) == str:
            if self.d_offset ==0:
                return sym.Symbol(self.d)
            elif self.d_offset > 0:
                return sym.Symbol(self.d) + self.d_offset
        if type(self.d) == int or type(self.d) == float:
            return self.d + self.d_offset

    def get_a_expr(self):
        if type(self.a) == str:
            if self.a_offset ==0:
                return sym.Symbol(self.a)
            elif self.a_offset > 0:
                return sym.Symbol(self.a) + self.a_offset
        if type(self.a) == int or type(self.a) == float:
            return self.a + self.a_offset
    
    def get_alpha_expr(self):
        if type(self.alpha) == str:
            if self.alpha_offset ==0:
                return sym.Symbol(self.alpha)
            elif self.alpha_offset > 0:
                return sym.Symbol(self.alpha) + self.alpha_offset
        if type(self.alpha) == int or type(self.alpha) == float:
            return self.alpha*pi/180 + self.alpha_offset
    
    def get_R(self):
        row1 = [sym.cos(self.get_theta_expr()),-sym.sin(self.get_theta_expr())*sym.cos(self.get_alpha_expr()),sym.sin(self.get_theta_expr())*sym.sin(self.get_alpha_expr())]
        row2 = [sym.sin(self.get_theta_expr()),sym.cos(self.get_theta_expr())*sym.cos(self.get_alpha_expr()),-sym.cos(self.get_theta_expr())*sym.sin(self.get_alpha_expr())]
        row3 = [0,sym.sin(self.get_alpha_expr()),sym.cos(self.get_alpha_expr())]
        return sym.Matrix([row1,row2,row3])

    def get_T(self):
        R = self.get_R()
        column = sym.Matrix([[self.get_a_expr()*sym.cos(self.get_theta_expr())],[self.get_a_expr()*sym.sin(self.get_theta_expr())],[self.get_d_expr()]])
        return sym.Matrix([[R,column],[0,0,0,1]])

class RobotSimulator:
    def __init__(self):
        self.frames = []
        self.types = []
        self.syms = []
        self.data = {'x':[],'y':[], 'z':[],'joint':[],'t':[],'color':[]}

    def add_frame(self,frame):
        self.frames.append(frame)
        frame_syms = frame.get_syms()
        for s in frame_syms:
            if s not in self.syms:
                self.syms.append(s)
                self.types.append(1 if 't' in s else 0)
        if len(frame_syms) == 0:
            self.types.append(0)
    
    def sub_into_matrix(self,symmat,value_dict):
        mat = []
        for i in range(symmat.shape[0]):
            row = []
            for j in range(symmat.shape[1]):
                subbed = symmat[i,j]
                for s in self.syms:
                    subbed = subbed.subs(s,value_dict[str(s)]).evalf()
                row.append(subbed)
            mat.append(row)
        return round_sym_mat(sym.Matrix(mat),3)

    def get_T(self,joint=None):
        if joint==None:
            joint = len(self.frames)
        T = sym.eye(4,4)
        for i in range(joint):
            T*=self.frames[i].get_T()
        return T

    def get_R(self,joint=None):
        if joint==None:
            joint = len(self.frames)
        R = self.frames[0].get_R()
        for i in range(1,joint):
            R*=self.frames[i].get_R()
        return round_sym_mat(R,3)

    def get_Jacobian(self,joint=None):
        size = len(self.frames)
        Z_vector = sym.Matrix([[0],[0],[1]])
        T = self.get_T(joint)
        p_xyz = sym.Matrix(T.col(3)[0:3])
        JA = p_xyz.jacobian(self.syms)
        JB = sym.Matrix(3,size,[0]*3*size)
        for i in range(size):
            col = self.types[i]*self.get_R(i)*Z_vector
            # print("col shape: {}, JB shape: {}".format(col.shape,JB.shape))
            JB[0:3,i] = col
        return sym.Matrix([[JA],[JB]])
    
    ### arrange jacobians into matrix
    ##  [J0,J1,J2]^T
    def get_all_Jacobians(self):
        size = len(self.frames)
        # j shape will always be (6 x size)
        J_rows = 6*size
        J_cols = size
        J = sym.SparseMatrix(J_rows,J_cols,[0]*J_rows*J_cols)
        for i in range(size):
            new_jacobian = self.get_Jacobian(i)
            J[i*6:(i+1)*6,0:size] = self.get_Jacobian(i+1)
        return J
    
    def get_initial_positon(self,initial_joint_states):
        size = len(self.frames)
        ts = [self.sub_into_matrix(self.get_T(i),initial_joint_states) for i in range(1,len(self.frames)+1)]
        state_vector = []
        for i in range(size):
            pxyz = sym.Matrix([ts[i][0:3,3],[0],[0],[0]])
            state_vector.append([pxyz])
        return sym.Matrix(state_vector)

    def get_velocities(self,state_dict,velocity_dict):
        J = self.get_all_Jacobians()
        evaluated_J = self.sub_into_matrix(J,state_dict)
        joint_vels = sym.Matrix([velocity_dict[str(x)] for x in self.syms])
        velocity = evaluated_J*joint_vels
        return velocity
    
    def update_states(self,states,velocities,timestep):
        for key in states:
            states[key]+=velocities[key]*timestep
        return states

    def save_state(self,xyzs,time):
        size = len(self.frames)
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


if __name__ == '__main__':
    # frame1_array = [["t1",0,1,0],[90,0,0,0]]
    frame1 = FrameParameters.without_offset("t1",0,1,0)
    frame2 = FrameParameters.without_offset("t2",0,1,0)
    frame3 = FrameParameters.without_offset("t3",0,1,0)
    frame4 = FrameParameters.without_offset("t4",0,1,0)

    # frame2_array = [["t2",0,1,0],[90,0,0,0]]
    # frame2 = FrameParameters.from_2d_array(frame2_array)
    # frame2 = FrameParameters.without_offset("t2",0,1,0)
    
    RS = RobotSimulator()
    RS.add_frame(frame1)
    RS.add_frame(frame2)
    RS.add_frame(frame3)
    RS.add_frame(frame4)

    joint_0 = {"t1":0,"t2":0,"t3":0,"t4":0}
    vel = {"t1":1,"t2":1,"t3":1,"t4":1}

    RS.simulate(joint_0,vel,100,.1)
    RS.write_data_to_csv("rs.csv")
    RS.plot_data({"x":"x","y":"y"})