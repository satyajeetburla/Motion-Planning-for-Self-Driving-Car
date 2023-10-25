# Motion Planning for Self-Driving Car

The objective of this project is to create a functional motion planning stack that can track a lane's center line, avoid static and dynamic obstructions, and manage stop signs. You will need to use path selection, velocity profile creation, static collision testing, and behavioral planning logic in order to achieve this. This project is the Capstone project for famous <a href="https://www.coursera.org/specializations/self-driving-cars?action=enroll">Self-driving Car Specialization</a> by <a href= "https://www.utoronto.ca/">University of Toronto</a> taught by <a href="https://www.trailab.utias.utoronto.ca/stevenwaslander">Steven L. Waslander</a> & <a href="https://scholar.google.com/citations?user=KtSR8_0AAAAJ&hl=en">Jonathan Kelly</a>.


## Setup Instructions:

### **1. Setting up CARLA Simulator:**

- Ensure the CARLA simulator is installed. Follow the <a href="https://carla.readthedocs.io/en/latest/start_quickstart/#carla-installation">installation guide</a>.

### **2. Accessing Project Resources:**

- Put eveything under python client tab of CARLA. **PythonClient\Project Codes**

### **3. Kickstarting Your Motion Planner:**

The major part of the Motion Planner will be based of:
- `behavioral_planner.py`
- `collision_checker.py`
- `local_planner.py`
- `path_optimizer.py`
- `velocity_planner.py`

#### **Behavioral Planning**:

In this segment, we've developed a state machine within behavioral_planner.py to efficiently handle scenarios involving stop signs. This allows the system to seamlessly transition between lane following, decelerating for stop signs, coming to a full stop, and then returning to lane following. To achieve this, we introduced helper functions: get_closest_index() and get_goal_index(). These functions enable the planner to pinpoint its position in relation to the global path and ascertain the current goal point. Furthermore, we've integrated the transition_state() function, encapsulating the core logic of the state machine. For a deeper understanding, the code comments provide comprehensive details on each function's operation.

#### **Path Crafting**:

During the path generation phase, we utilized provided mathematical frameworks to produce spiral paths. Our primary contribution was calculating the goal state set through the get_goal_state_set() function. We also implemented the thetaf() function to determine the car's yaw based on arc length points of a specified spiral. The optimize_spiral() function was employed to fine-tune the optimization problem for each path. Once optimized, we used the sample_spiral() function to derive the actual path. Further insights can be gleaned from the code comments in local_planner.py and path_optimizer.py.

#### **Static Collision Verification**:

In the motion planning phase, we refined collision_checker.py to incorporate circle-based collision checks on our generated path set. Specifically, we calculated the circle's location for each path point using the collision_check() function. More details can be found in the code comments.

#### **Optimal Path Selection**:

For path selection, we assessed an objective function across the generated paths to determine the optimal route. Our primary objectives were to avoid paths conflicting with static obstacles and ensure paths closely followed the global centerline. We introduced a penalty term to discourage proximity to paths that clash with obstacles. The selection process was executed in the select_best_path_index() function within collision_checker.py.

#### **Velocity Profile Creation**:

In the final phase, we developed a velocity profile. While our planner doesn't cater to every scenario, it effectively manages stop signs, leading dynamic obstacles, and standard lane upkeep. This was encapsulated in the compute_velocity_profile() function found in velocity_planner.py. We integrated the requisite physics functions for velocity planning. After addressing all the tasks, we activated the corresponding sections in "module7.py" and prepared the system for simulator execution.

### **4. Launching the CARLA Simulator & Running the Client Controller:**

1. start the CARLA simulator at a 30hz fixed time-step.
2. In terminal run :
```
python3 module_7.py
```

