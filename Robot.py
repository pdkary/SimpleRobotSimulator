import sympy as sym
from math import pi
from SymHelpers import *

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

class Robot:
    def __init__(self):
        self.frames = []
        self.types = []
        self.syms = []

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
        
    def get_all_Jacobians(self):
        size = len(self.frames)
        # j shape will always be (6 x size)
        J_rows = 6*size
        J_cols = size
        J = sym.SparseMatrix(J_rows,J_cols,[0]*J_rows*J_cols)
        for i in range(size):
            J[i*6:(i+1)*6,0:size] = self.get_Jacobian(i+1)
        return J

    def get_joint_torques(self,value_dict,force_vector,gravity_vector):
        J = self.get_Jacobian()
        J = self.sub_into_matrix(J,value_dict)[[0,1,5],0:3]
        jtf = J.transpose()*force_vector
        return jtf + gravity_vector

    def get_effective_force(self,value_dict,torque_vector,gravity_vector):
        J  = self.get_Jacobian()
        J = self.sub_into_matrix(J,value_dict)[[0,1,5],0:3]
        return J.inv().transpose()*(torque_vector - gravity_vector)
