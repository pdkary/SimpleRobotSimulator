# SimpleRobotSimulator

SimpleRobotSimulator is a simulation engine that takes in joint parameters for REVOLUTE or PRISMATIC joints, and respective joint variable VELOCITIES, and produces a plotly animation showing the movement of the robot for a given timestep

## Installation

```bash
pip3 install -r requirements.txt
```
## Usage
Create a simple RR robot with frame parameters t1 (theta 1) and t2 (theta 2)
```python3
frame1 = FrameParameters.without_offset("t1",0,1,0)
frame2 = FrameParameters.without_offset("t2",0,1,0)

RS = RobotSimulator()
RS.add_frame(frame1)
RS.add_frame(frame2)

joint_initial_states = {"t1":0,"t2":0}
joint_velocities = {"t1":1,"t2":1}

RS.simulate(joint_0,vel,100,.1)
RS.write_data_to_csv("rs.csv")
RS.plot_data({"x":"x","y":"y"})
```
This call will simulator the motion of each joint, for 100 steps of timestep .1 (10 seconds of motion), save the data to a csv file, and then produce an interactive plotly animation in your web browser

## License
[MIT](https://choosealicense.com/licenses/mit/)