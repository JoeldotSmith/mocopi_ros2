import socket
import base64
import struct
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

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

# all urdf joints
# floating_base_joint
# pelvis_contour_joint
# left_hip_pitch_joint
# left_hip_roll_joint
# left_hip_yaw_joint
# left_knee_joint
# left_ankle_pitch_joint
# left_ankle_roll_joint
# right_hip_pitch_joint
# right_hip_roll_joint
# right_hip_yaw_joint
# right_knee_joint
# right_ankle_pitch_joint
# right_ankle_roll_joint
# waist_yaw_joint/
# waist_roll_joint
# waist_pitch_joint
# logo_joint
# head_joint
# waist_support_joint
# imu_in_torso_joint
# imu_in_pelvis_joint
# d435_joint
# mid360_joint
# left_shoulder_pitch_joint
# left_shoulder_roll_joint
# left_shoulder_yaw_joint
# left_elbow_joint
# left_wrist_roll_joint
# left_wrist_pitch_joint
# left_wrist_yaw_joint
# left_hand_palm_joint
# right_shoulder_pitch_joint
# right_shoulder_roll_joint
# right_shoulder_yaw_joint
# right_elbow_joint
# right_wrist_roll_joint
# right_wrist_pitch_joint
# right_wrist_yaw_joint
# right_hand_palm_joint
mapping = {
        "root": [("floating_base_joint", None)],
        "torso_1": [("pelvis_contour_joint", None)],
        "torso_7": [("waist_yaw_joint", "yaw"), ("waist_roll_joint", "roll"), ("waist_pitch_joint", "pitch")],
        "neck_2": [("logo_joint", None)],
        "head": [("head_joint", "yaw")],
        "l_shoulder": [("left_shoulder_pitch_joint", "pitch"), ("left_shoulder_roll_joint", "roll"), ("left_shoulder_yaw_joint", "yaw")],
        "l_up_arm": [("left_elbow_joint", "pitch")],
        "l_low_arm": [("left_wrist_roll_joint", "roll"), ("left_wrist_pitch_joint", "pitch"), ("left_wrist_yaw_joint", "yaw")],
        "l_hand": [("left_hand_palm_joint", None)],
        "r_shoulder": [("right_shoulder_pitch_joint", "pitch"), ("right_shoulder_roll_joint", "roll"), ("right_shoulder_yaw_joint", "yaw")],
        "r_up_arm": [("right_elbow_joint", "pitch")],
        "r_low_arm": [("right_wrist_roll_joint", "roll"), ("right_wrist_pitch_joint", "pitch"), ("right_wrist_yaw_joint", "yaw")],
        "r_hand": [("right_hand_palm_joint", None)],
        "l_up_leg": [("left_hip_pitch_joint", "pitch"), ("left_hip_roll_joint", "roll"), ("left_hip_yaw_joint", "yaw")],
        "l_low_leg": [("left_knee_joint", "pitch")],
        "l_foot": [("left_ankle_pitch_joint", "pitch"), ("left_ankle_roll_joint", "roll")],
        "r_up_leg": [("right_hip_pitch_joint", "pitch"), ("right_hip_roll_joint", "roll"), ("right_hip_yaw_joint", "yaw")],
        "r_low_leg": [("right_knee_joint", "pitch")],
        "r_foot": [("right_ankle_pitch_joint", "pitch"), ("right_ankle_roll_joint", "roll")]
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

LOG_FILE="mocopi_data_log.bin"

def quaternion_to_euler(w, x, y, z):
    roll = math.atan2(2*(w*x + y*z), 1-2*(x*x+y*y))
    pitch = math.asin(2*(w*y - z*x))
    yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return roll, pitch, yaw



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
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("", 12351))
        self.get_logger().info("Mocopi receiver started")
        self.timer = self.create_timer(0.01, self.receive_data)
        self.log_data = []

    def receive_data(self):
        try:
            message, _ = self.socket.recvfrom(2048)
            self.log_data.append(message)
            data = _process_packet(message)
            self.broadcast_transforms(data)

            if len(self.log_data) % 100 == 0:
                print(message)
                self.save_data()
        except KeyError as e:
            self.get_logger().error(f"Socket error: {e}")
    
    def save_data(self):
        with open(LOG_FILE, "w") as f:  # Open file in text mode
            for raw_data in self.log_data:
                encoded_data = base64.b64encode(raw_data).decode('utf-8')  # Convert to base64 string
                f.write(encoded_data + "\n")  # Save as string with newline separator


    
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

    # def broadcast_transforms(self, data):
    #     transforms = []
    #     if "fram" in data:
    #         for (p, c) in pairs:
    #             trans = self.make_tf(p, c, data)
    #             if trans:
    #                 transforms.append(trans)
    #     if transforms:
    #         self.br.sendTransform(transforms)

    def broadcast_transforms(self, data):
        transforms = []
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        for (p, c) in pairs:
            trans = self.make_tf(p, c, data)
            if trans:
                transforms.append(trans)
                mocopi_joint = joint_map[c] if c < len(joint_map) else None
                if mocopi_joint and mocopi_joint in mapping:
                    roll, pitch, yaw = quaternion_to_euler(
                        trans.transform.rotation.w,
                        trans.transform.rotation.x,
                        trans.transform.rotation.y,
                        trans.transform.rotation.z
                    )
                    for urdf_joint, axis in mapping[mocopi_joint]:
                        angle = {"roll": roll, "pitch": pitch, "yaw": yaw}.get(axis, 0.0)
                        joint_state_msg.name.append(urdf_joint)
                        joint_state_msg.position.append(angle)
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
