import numpy as np

def dh_transform(theta, d, alpha, a):
    # Convierte ángulos a radianes
    theta = np.deg2rad(theta)
    alpha = np.deg2rad(alpha)
    
    # Matriz de transformación utilizando los parámetros DH
    transform_matrix = np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])
    
    return transform_matrix

def forward_kinematics(joint_angles):
    # Parámetros DH
    dh_params = [
        (joint_angles[0], 243.3, -90, 0),
        (joint_angles[1] - 90, 0, 180, 200),
        (joint_angles[2] - 90, 0, 90, 87),
        (joint_angles[3], 227.6, 90, 0),
        (joint_angles[4], 0, -90, 0),
        (joint_angles[5], 61.5, 0, 0)
    ]
    
    T = np.eye(4)
    
    for param in dh_params:
        T = np.dot(T, dh_transform(*param))
    
    position = np.fix(T[0:3, 3])
    
    r11, r12, r13 = T[0, 0], T[0, 1], T[0, 2]
    r21, r22, r23 = T[1, 0], T[1, 1], T[1, 2]
    r31, r32, r33 = T[2, 0], T[2, 1], T[2, 2]
    
    sy = np.sqrt(r11*2 + r21*2)
    
    if sy > 1e-6: 
        x_angle = np.arctan2(r32, r33)
        y_angle = np.arctan2(-r31, sy)
        z_angle = np.arctan2(r21, r11)
    else: 
        x_angle = np.arctan2(-r23, r22)
        y_angle = np.arctan2(-r31, sy)
        z_angle = 0
    
    orientation = np.fix(np.rad2deg([x_angle, y_angle, -z_angle]))
    
    return np.concatenate((position, orientation))

# Prueba del código con diferentes ángulos de las juntas
joint_angles_proofs = [
    [0, 0, 0, 0, 0, 0],
    [90, 0, 84.16, 0, 45, 90],
    [90, 0, -45, 60, -30, 180]
]

for angles in joint_angles_proofs:
    tcp = forward_kinematics(angles)
    print(f"TCP position and orientation: {tcp}")