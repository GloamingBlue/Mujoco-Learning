import pinocchio as pin
import numpy as np

# 加载 URDF 文件
urdf_path = "/home/dar/MuJoCoBin/mujoco-learning/franka_panda_description/robots/panda_arm.urdf"
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()

# 获取关节限位
lower_limits = model.lowerPositionLimit
upper_limits = model.upperPositionLimit
# 打印关节限位
print("Lower limits:", lower_limits)
print("Upper limits:", upper_limits)

# 打印配置向量和速度向量的维度
print(f"Number of configuration variables (nq): {model.nq}")
print(f"Number of velocity variables (nv): {model.nv}")

# 计算正运动学
q = np.zeros(model.nq)
data = model.createData()
pin.framesForwardKinematics(model, data, q)

# 打印 joints 信息
for i, joint in enumerate(model.joints):
    print(f"Joint {i}:")
    print(f"  Name: {model.names[i]}")

# 打印 jointPlacements 信息
print("\nJoint Placements Information:")
for i, placement in enumerate(model.jointPlacements):
    print(f"Joint Placement {i}:")
    print(f"  Translation: {placement.translation}")
    print(f"  Rotation Matrix:\n{placement.rotation}")

# 打印 inertias 信息
print("\nInertias Information:")
for i, inertia in enumerate(model.inertias):
    print(f"Inertia {i}:")
    print(f"  Mass: {inertia.mass}")
    print(f"  Center of Mass: {inertia.lever}")
    print(f"  Inertia Matrix:\n{inertia.inertia}")

