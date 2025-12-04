# %% [markdown]
# # Homework 6 key
# %%
import kinematics as kin
import transforms as tr
from visualization import VizScene
import numpy as np

# %% [markdown]
# # Problem 1:
dh_params = [[0,    0.0,   1.0,   -np.pi/2.0],
             [0,    0.0,   1.0,   0]]
arm = kin.SerialArm(dh_params, jt = ['r', 'r'])

# joint angles defined in each part of problem 1
q_a = [0, np.pi/4.0]
q_b = [0, np.pi/2.0]
q_c = [np.pi/4.0, np.pi/4.0]
q_d = [0, 0]
q_e = [0, 0]
q_s = [q_a, q_b, q_c, q_d, q_e]

# Forces defined for each part of problem 1
    # note: the forces given in the problem are the forces applied by the environment on the end
# each of these are multiplied by negative 1 to get the reaction force. 
F_a = np.array([-1, 0, 0]).reshape((3,)) * -1 # N
F_b = np.array([-1, 0, 0]).reshape((3,)) * -1 # N
F_c = np.array([-1, -1, 0]).reshape((3,)) * -1 # N
F_d = np.array([0, 0, 1]).reshape((3,)) * -1 # N
F_e = np.array([1, 0, 0]).reshape((3,)) * -1 # N
Forces = [F_a, F_b, F_c, F_d, F_e]

# now calculate the Jacobian and torque for each part of problem 1
for i in range(len(q_s)):
    J = arm.jacob(q_s[i])
    tau = J[0:3,:].T @ Forces[i]
    print('---------------------')
    print(f'For q = {q_s[i]}, F = {Forces[i]}')
    print('Torque:\n', tau)
    print('---------------------')

## this next part is not necessary, but you can uncomment to visualize the 
## arm and the force being applied at the tip: 
# for i in range(len(q_s)):
#     viz = VizScene()

#     # add the arm to the visualization
#     viz.add_arm(arm, draw_frames=True)
#     viz.update(qs=[q_s[i]])

#     # find the tip of the arm, to place the force vector there
#     T_2_in_0 = arm.fk(q_s[i])
#     tip_pos = T_2_in_0[0:3,3]

#     # we scale the force vector for visualization
#     viz.add_axis(axis=Forces[i], pos_offset=tip_pos, scale=np.linalg.norm(Forces[i])*4, label=f'F_{i+1}')

#     viz.hold()
#     viz.close_viz()



# %% [markdown]
# # Problem 2:

# %%
dh = [[0, 0.2, 0, -np.pi/2.0],
      [0, 0, 0.2, 0],
      [np.pi/2.0, 0, 0, np.pi/2.0],
      [np.pi/2, 0.4, 0, -np.pi/2.0],
      [0, 0, 0, np.pi/2.0],
      [0, 0.4, 0, 0]]

jt_types = ['r', 'r', 'r', 'r', 'r', 'r']

# making the 6 DoF arm
arm = kin.SerialArm(dh, jt=jt_types, tip=tr.se3(tr.roty(-np.pi/2.0)))

# defining joint angles
q = [np.pi/4.0]*6
T = arm.fk(q)

# show the robot and a goal (just for demo's sake)
viz = VizScene()
viz.add_arm(arm, draw_frames=True)
viz.add_marker(T[0:3,3], radius=0.10)
viz.update(qs = [q])

viz.hold()

# %% [markdown]
# # Problem 3:

# %%
q_set1 = np.array([0]*6)
q_set2 = np.array([np.pi/4]*6)

# if this runs for too long or bogs down your computer, we can do 1 goal at a time instead
goals = np.array([[-0.149, 0.364, 1.03],
                  [-0.171, -0.682, -0.192],
                  [0.822, -0.1878, 0.533],
                  [-0.336, 0.095, 0.931],
                  [0.335, 0.368, 0.88]])

# part a)i)
sln_pinv_q_set1 = []
sln_pinv_q_set2 = []

for goal in goals:
    qf,_,_,_ = arm.ik_position(goal, q0=q_set1, method='pinv', K=np.eye(3), max_iter=1000)
    sln_pinv_q_set1.append(qf)

    qf,_,_,_ = arm.ik_position(goal, q0=q_set2, method='pinv', K=np.eye(3), max_iter=1000)
    sln_pinv_q_set2.append(qf)


# %%
# part a)ii)
sln_J_T_q_set1 = []
sln_J_T_q_set2 = []

for goal in goals:
    qf,_,_,_ = arm.ik_position(goal, q0=q_set1, method='J_T', K=np.eye(3), max_iter=1000)
    sln_J_T_q_set1.append(qf)

    qf,_,_,_ = arm.ik_position(goal, q0=q_set2, method='J_T', K=np.eye(3), max_iter=1000)
    sln_J_T_q_set2.append(qf)


#%%
# part a)iii)
viz = VizScene()

# this arm with joints that are darker red is for the starting configuration
viz.add_arm(arm)

# colors defined below were obtained from - https://www.rapidtables.com/web/color/RGB_Color.html
# this arm with joints that are brighter/lighter red is for the J_T solutions
viz.add_arm(arm, joint_colors=[np.array([1.0, 51.0/255.0, 51.0/255.0, 1])]*arm.n)

# this arm with joints that are almost pink is for the pinv solutions
viz.add_arm(arm, joint_colors=[np.array([1.0, 51.0/255.0, 1.0, 1])]*arm.n)

# now add a marker for the goal location (initial position doesn't matter)
viz.add_marker([0,0,0], radius=0.10)

counter = 0

time_to_run = 5
for goal in goals:
    viz.update(qs=[q_set1, sln_pinv_q_set1[counter], sln_J_T_q_set1[counter]], poss=[goal])
    viz.hold(time_to_run)
    counter += 1

input('press Enter when ready to see next set starting from q_set2')

counter = 0
for goal in goals:
    viz.update(qs=[q_set2, sln_pinv_q_set2[counter], sln_J_T_q_set2[counter]], poss=[goal])
    viz.hold(time_to_run)
    counter += 1

viz.close_viz()
