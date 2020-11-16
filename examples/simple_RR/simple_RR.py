import sys
sys.path.append("../../")
from Robot import FrameParameters,Robot
from RobotSimulator import RobotSimulator

if __name__ == '__main__':
    frame1 = FrameParameters.without_offset("t1",0,1,0)
    frame2 = FrameParameters.without_offset("t2",0,1,0)

    robot = Robot()
    robot.add_frame(frame1)
    robot.add_frame(frame2)

    RS = RobotSimulator(robot)

    joint_initial_states = {"t1":0,"t2":0}
    joint_velocities = {"t1":1,"t2":1} # velocities in radians/sec

    RS.simulate(joint_initial_states,joint_velocities,100,.1)
    RS.write_data_to_csv("simpleRR.csv")
    RS.plot_data({"x":"x","y":"y"})