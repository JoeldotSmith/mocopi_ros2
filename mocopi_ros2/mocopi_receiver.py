import socket
import struct
import math
import rclpy
import time
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.time import Time
import tf2_ros
from geometry_msgs.msg import TransformStamped, Transform
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation



# Mocopi joint names and hierarchy
joint_map = [
    "root",        # 0
    "torso_1",     # 1
    "torso_2",     # 2
    "torso_3",     # 3
    "torso_4",     # 4
    "torso_5",     # 5
    "torso_6",     # 6
    "torso_7",     # 7
    "neck_1",      # 8
    "neck_2",      # 9
    "head",        # 10
    "l_shoulder",  # 11
    "l_up_arm",    # 12
    "l_low_arm",   # 13
    "l_hand",      # 14
    "r_shoulder",  # 15
    "r_up_arm",    # 16
    "r_low_arm",   # 17
    "r_hand",      # 18
    "l_up_leg",    # 19
    "l_low_leg",   # 20
    "l_foot",      # 21
    "l_toes",      # 22
    "r_up_leg",    # 23
    "r_low_leg",   # 24
    "r_foot",      # 25
    "r_toes"       # 26
]

mapping = {
    "root": [("pelvis_contour_joint", None)],
    "torso_1": [("waist_yaw_joint", "yaw"), ("waist_roll_joint", "roll"), ("waist_pitch_joint", "pitch")],

    # Left Arm
    "l_up_arm": [
        ("left_shoulder_pitch_joint", "pitch"),
        ("left_shoulder_roll_joint", "roll"),
        ("left_shoulder_yaw_joint", "yaw")
    ],
    "l_low_arm": [ ("left_wrist_pitch_joint", None), ],
    "l_shoulder": [("left_wrist_pitch_joint", None)],
    "l_hand": [
        ("left_wrist_roll_joint", None),
        ("left_elbow_joint", "pitch"),
        ("left_wrist_yaw_joint", None)
    ],

    # Right Arm
    "r_up_arm": [
        ("right_shoulder_pitch_joint", "pitch"),
        ("right_shoulder_roll_joint", "roll"),
        ("right_shoulder_yaw_joint", "yaw")
    ],
    "r_low_arm": [("right_wrist_pitch_joint", None)], 
    "r_shoulder": [("right_wrist_pitch_joint", None)],
    "r_hand": [
        ("right_wrist_roll_joint", None),
        ("right_elbow_joint", "pitch"), 
        ("right_wrist_yaw_joint", None)
    ],

    # Left Leg
    "l_up_leg": [
        ("left_hip_pitch_joint", "pitch"),
        ("left_hip_roll_joint", "roll"),
        ("left_hip_yaw_joint", "yaw")
    ],
    "l_low_leg": [("left_knee_joint", "pitch")],
    "l_foot": [
        ("left_ankle_pitch_joint", "pitch"),
        ("left_ankle_roll_joint", "roll")
    ],

    # Right Leg
    "r_up_leg": [
        ("right_hip_pitch_joint", "pitch"),
        ("right_hip_roll_joint", "roll"),
        ("right_hip_yaw_joint", "yaw")
    ],
    "r_low_leg": [("right_knee_joint", "pitch")],
    "r_foot": [
        ("right_ankle_pitch_joint", "pitch"),
        ("right_ankle_roll_joint", "roll")
    ]
}

pairs = [
    (99, 0),  # base to root
    (0, 1), (1, 7), (7, 8), (8, 10),  # spine to neck/head
    (7, 11),  # torso to l_shoulder
    (11, 12), (12, 13), (13, 14),  # left arm
    (7, 15),  # torso to r_shoulder
    (15, 16), (16, 17), (17, 18),  # right arm
    (0, 19),  # root to l_up_leg
    (19, 20), (20, 21), (21, 22),  # left leg
    (0, 23),  # root to r_up_leg
    (23, 24), (24, 25), (25, 26)   # right leg
]

def QuaternionToE(x, y, z, w):
    r = Rotation.from_quat([x, y, z, w])
    return r.as_euler('xyz', degrees=False)  

def rotate_quaternion(q, axis):
    rot_90 = Rotation.from_euler(axis, 90, degrees=True).as_quat()
    q_rot = Rotation.from_quat(q) * Rotation.from_quat(rot_90)
    return q_rot.as_quat()

def rotate_quaternion_negative(q, axis):
    rot_90 = Rotation.from_euler(axis, -90, degrees=True).as_quat()
    q_rot = Rotation.from_quat(q) * Rotation.from_quat(rot_90)
    return q_rot.as_quat()

def is_field(name):
    return name.isalpha()

def _deserialize(data, index, length, is_list=False):
    result = [] if is_list else {}
    end_pos = index + length
    while end_pos - index > 8 and is_field(data[index+4:index+8]):
        size = struct.unpack("@i", data[index: index+4])[0]
        index += 4
        field = data[index:index+4]
        index += 4
        value, index2 = _deserialize(data, index, size, field in [b"btrs", b"bons"])
        index = index2
        if is_list:
            result.append(value)
        else:
            result[field.decode()] = value
    if len(result) == 0:
        body = data[index:index+length]
        return body, index + len(body)
    else:
        return result, index

def _process_packet(message):
    data = _deserialize(message, 0, len(message), False)[0]
    data["head"]["ftyp"] = data["head"]["ftyp"].decode()
    data["head"]["vrsn"] = ord(data["head"]["vrsn"])
    data["sndf"]["ipad"] = struct.unpack("@BBBBBBBB", data["sndf"]["ipad"])
    data["sndf"]["rcvp"] = struct.unpack("@H", data["sndf"]["rcvp"])[0]
    if "skdf" in data:
        for item in data["skdf"]["bons"]:
            item["bnid"] = struct.unpack("@H", item["bnid"])[0]
            item["pbid"] = struct.unpack("@H", item["pbid"])[0]
            item["tran"] = struct.unpack("@fffffff", item["tran"])
    elif "fram" in data:
        data["fram"]["fnum"] = struct.unpack("@I", data["fram"]["fnum"])[0]
        data["fram"]["time"] = struct.unpack("@I", data["fram"]["time"])[0]
        for item in data["fram"]["btrs"]:
            item["bnid"] = struct.unpack("@H", item["bnid"])[0]
            item["tran"] = struct.unpack("@fffffff", item["tran"])
    return data

class MocopiReceiver(Node):
    def __init__(self):
        super().__init__('mocopi_receiver')
        self.br = tf2_ros.TransformBroadcaster(self)
        
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.wave_pub = self.create_publisher(String, '/wave_detector', 10)
        self.r_hand_pub = self.create_publisher(Transform, '/r_hand', 10)
        self.l_hand_pub = self.create_publisher(Transform, '/l_hand', 10)

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("", 12351))
        self.get_logger().info("Mocopi receiver started")
        self.timer = self.create_timer(0.01, self.receive_data)


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.hand_positions = []  
        self.last_wave_time = 0
        self.current_time = 0
        self.last_handshake_time = 0
        self.wave_threshold = 0.15 
        self.time_window = 5  
    
    def append_data(self, roll, z_pos, x_pos):
        self.current_time = time.time()
        self.hand_positions.append((self.current_time, roll, z_pos, x_pos))
        self.hand_positions = [p for p in self.hand_positions if self.current_time - p[0] < self.time_window]

    def detect_wave(self):
        if len(self.hand_positions) < 4: 
            return False

        roll_positions = [pos[1] for pos in self.hand_positions]
        z_positions = [pos[2] for pos in self.hand_positions]
        
        movement_range = max(roll_positions) - min(roll_positions)
        avg_z_pos = sum(z_positions) / len(z_positions)

        sign_changes = sum(1 for i in range(1, len(roll_positions)) if (roll_positions[i] - roll_positions[i-1]) * (roll_positions[i-1] - roll_positions[i-2]) < 0)

        if ( movement_range > self.wave_threshold and avg_z_pos > 1.5 and sign_changes >= 2 and  (self.current_time - self.last_wave_time) > self.time_window):
            self.last_wave_time = self.current_time
            print("Wave Detected")
            self.wave_pub.publish(String(data="Wave detected"))
    
    def detect_handshake(self):
        if len(self.hand_positions) < 4: 
            return False
        
        x_positions = [pos[3] for pos in self.hand_positions] 
        avg_x_pos = sum(x_positions) / len(x_positions)

        if ( avg_x_pos > 0.45 and (self.current_time - self.last_wave_time) > self.time_window):
            self.last_handshake_time = self.current_time
            print("Handshake Detected")
            self.wave_pub.publish(String(data="Handshake detected"))

    def receive_data(self):
        try:
            message, _ = self.socket.recvfrom(2048)
            data = _process_packet(message)
            self.broadcast_transforms(data)
        except Exception as e:
            print(e)

    def make_tf(self, pframe_id, cframe_id, data):
        t = TransformStamped()
        t.header.frame_id = joint_map[pframe_id] if 0 <= pframe_id <= 26 else "pelvis"

        for btdt in data["fram"]["btrs"]:
            if btdt["bnid"] == cframe_id:
                t.child_frame_id = joint_map[cframe_id]
                trans = btdt["tran"]

                t.header.stamp = self.get_clock().now().to_msg()
                if cframe_id in [1, 7]:  # Torso joints
                    t.transform.translation.y = trans[4] * 7/2
                    t.transform.translation.z = trans[5] * 7/2
                    t.transform.translation.x = trans[6] * 7/2
                else:
                    t.transform.translation.x = trans[6]
                    t.transform.translation.y = trans[4]
                    t.transform.translation.z = trans[5]

                t.transform.rotation.x = trans[2]
                t.transform.rotation.y = trans[0]
                t.transform.rotation.z = trans[1]
                t.transform.rotation.w = trans[3]
                return t
        return None
    
    def get_absolute_transform(self, target_frame, source_frame):
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            return trans.transform
        except Exception as e:
            self.get_logger().warn(f"Could not get transform {target_frame} -> {source_frame}: {e}")
            return None

    def broadcast_transforms(self, data):
        transforms = []
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        for (p, c) in pairs:
            trans = self.make_tf(p, c, data)
            if trans and joint_map[c] == "r_hand":
                absolute_hand_transform = self.get_absolute_transform("pelvis", "r_hand")
                absolute_hand_transform_left = self.get_absolute_transform("root", "l_hand")
                absolute_hand_transform_right = self.get_absolute_transform("root", "r_hand")

                if absolute_hand_transform:
                    roll, _, _ = QuaternionToE(trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w)
                    self.append_data(roll, absolute_hand_transform.translation.z, absolute_hand_transform.translation.x)
                    self.detect_wave()
                    self.detect_handshake()

                if absolute_hand_transform_left:
                    self.l_hand_pub.publish(absolute_hand_transform_left)                

                if absolute_hand_transform_right:
                    self.r_hand_pub.publish(absolute_hand_transform_right)

            if trans:
                transforms.append(trans)
                mocopi_joint = joint_map[c] if c < len(joint_map) else None
                if mocopi_joint and mocopi_joint in mapping:


                    x = trans.transform.rotation.x
                    y = trans.transform.rotation.y
                    z = trans.transform.rotation.z
                    w = trans.transform.rotation.w
                    q = [x, y, z, w]

                    rotated_q_x_neg = rotate_quaternion_negative(q, 'x')
                    rotated_roll_x_neg, rotated_pitch_x_neg, rotated_yaw_x_neg = QuaternionToE(*rotated_q_x_neg)
                    rotated_q_x = rotate_quaternion(q, 'x')
                    rotated_roll_x, rotated_pitch_x, rotated_yaw_x = QuaternionToE(*rotated_q_x)


                    roll, pitch, yaw = QuaternionToE(x, y, z, w)

                    for urdf_joint, axis in mapping[mocopi_joint]:
                        if urdf_joint == "left_shoulder_yaw_joint" or urdf_joint == "left_shoulder_roll_joint" or urdf_joint == "left_shoulder_pitch_joint":
                            angle = {"roll": rotated_roll_x, "pitch": rotated_pitch_x, "yaw": rotated_yaw_x}.get(axis, 0.0)

                        elif urdf_joint == "left_elbow_joint":
                            rotate = rotate_quaternion_negative(q, 'x')
                            rotate = rotate_quaternion_negative(rotate, 'y')
                            r, p, y = QuaternionToE(*rotate)
                            angle = {"roll": r, "pitch": p, "yaw": y}.get(axis, 0.0)

                        elif urdf_joint == "right_shoulder_yaw_joint" or urdf_joint == "right_shoulder_roll_joint" or urdf_joint == "right_shoulder_pitch_joint":
                            angle = {"roll": rotated_roll_x_neg, "pitch": rotated_pitch_x_neg, "yaw": rotated_yaw_x_neg}.get(axis, 0.0)

                        elif urdf_joint == "right_elbow_joint":
                            rotate = rotate_quaternion(q, 'x')
                            rotate = rotate_quaternion_negative(rotate, 'y')
                            r, p, y = QuaternionToE(*rotate)
                            angle = {"roll": r, "pitch": p, "yaw": y}.get(axis, 0.0)

                        else:
                            angle = {"roll": roll, "pitch": pitch, "yaw": yaw}.get(axis, 0.0)



                        joint_state_msg.name.append(urdf_joint)
                        joint_state_msg.position.append(angle)

                    transforms.append(trans)
        self.br.sendTransform(transforms)
        self.joint_state_pub.publish(joint_state_msg)

def main():
    rclpy.init()
    node = MocopiReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
