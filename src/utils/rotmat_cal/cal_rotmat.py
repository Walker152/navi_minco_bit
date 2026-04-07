import math

# Set roll/pitch/yaw here (degrees)
roll_deg = -20.0
pitch_deg = 0.0
yaw_deg = 0.0


def euler_xyz_intrinsic_to_matrix(roll_d, pitch_d, yaw_d):
    """Return rotation matrix for intrinsic xyz (roll, pitch, yaw)."""
    roll = math.radians(roll_d)
    pitch = math.radians(pitch_d)
    yaw = math.radians(yaw_d)

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


rot_matrix = euler_xyz_intrinsic_to_matrix(roll_deg, pitch_deg, yaw_deg)

print("extrinsic_R: [")
for row in rot_matrix:
    print(f"  {row[0]:.6f}, {row[1]:.6f}, {row[2]:.6f},")
print("]")