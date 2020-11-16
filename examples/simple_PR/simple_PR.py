import sys
sys.path.append("../../")
from Robot import FrameParameters,Robot
from RobotSimulator import RobotSimulator

if __name__ == '__main__':
    frame_1_array = [["t1",0,0,90],[90,0,0,0]]
    frame1 = FrameParameters.from_2d_array(frame_1_array)
    frame2 = FrameParameters.without_offset(0,"d2",0,0)

    robot = Robot()
    robot.add_frame(frame1)
    robot.add_frame(frame2)

    RS = RobotSimulator(robot)

    joint_initial_states = {"t1":0,"d2":0}
    joint_velocities = {"t1":1,"d2":1} # velocities in radians/sec

    RS.simulate(joint_initial_states,joint_velocities,100,.1)
    RS.write_data_to_csv("simplePR.csv")
    RS.plot_data({"x":"x","y":"y"})