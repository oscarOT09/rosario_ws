import numpy as np

def wrap_to_Pi(angle):
        result = np.fmod((angle + np.pi), (2*np.pi))
        if result < 0:
            result += 2 * np.pi
        return result - np.pi

def delta_angle(prev, current, next_):
        v1 = current - prev
        v2 = next_ - current

        angle1 = np.arctan2(v1[1], v1[0])
        angle2 = np.arctan2(v2[1], v2[0])

        delta = angle2 - angle1
        #delta = (delta + np.pi) % (2 * np.pi) - np.pi  # Normalización del ágnulo entre -pi y pi
        return wrap_to_Pi(delta)  # en radianes
'''
(1.73205, 1) | 2 m | 30°
(-0.147334, 1.68404) | 2 m | 130°
(2.31468, 2.11816) | 2.5 m | -150°
(3.76468, -0.393313) | 3 m | -70°
'''


path_queue = [(1.75, 1.0), (-0.15, 1.70), (2.31, 2.15), (3.90, -0.40)]

prev_pose = np.array([0.0, 0.0])
curr_pose = np.array([0.0, 0.0])
next_pose = np.array([0.0, 0.0])

prev_goal_X = 0.0
prev_goal_Y = 0.0

for i in range(len(path_queue)):
    #X_ref, Y_ref = path_queue.pop(0)
    next_pose= np.array(path_queue.pop(0))
    #dx = X_ref - prev_goal_X
    #dy = Y_ref - prev_goal_Y
    #ang_ref = np.arctan2(dy, dx)
    ang_ref = delta_angle(prev_pose, curr_pose, next_pose)

    dx = next_pose[0] - curr_pose[0]
    dy = next_pose[1] - curr_pose[1]
    dist_ref = np.hypot(dx, dy)

    print(f"Angulo de ({curr_pose[0]}, {curr_pose[1]}) a ({next_pose[0]}, {next_pose[1]}): {ang_ref} rad | {np.rad2deg(ang_ref)}°")
    print(f"Distancia de ({curr_pose[0]}, {curr_pose[1]}) a ({next_pose[0]}, {next_pose[1]}): {dist_ref} m\n")

    #prev_goal_X = X_ref
    #prev_goal_Y = Y_ref
    prev_pose = curr_pose.copy()
    curr_pose = next_pose.copy()