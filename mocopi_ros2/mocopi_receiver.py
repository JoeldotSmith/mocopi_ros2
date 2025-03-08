import socket
import struct
import math
 
import rclpy
from rclpy.node import Node
from rclpy.time import Time
 
import tf2_ros
from geometry_msgs.msg import TransformStamped
 
joint_map = [
    "root", # 0
    "torso_0", # 1
    "torso_1", # 2
    "torso_2", # 3
    "neck", # 4
    "head", # 5
    "l_shoulder", # 6
    "l_up_arm", # 7
    "l_low_arm", # 8
    "l_hand", # 9
    "r_shoulder", # 10
    "r_up_arm", # 11
    "r_low_arm", # 12
    "r_hand", # 13
    "l_up_leg", # 14
    "l_low_leg", # 15
    "l_foot", # 16
    "r_up_leg", # 17
    "r_low_leg", # 18
    "r_foot", # 19
]

pairs = [
    (99, 0),  # base to root
    (0, 1),   # root to torso_0
    (1, 2),   # torso_0 to torso_1
    (2, 3),   # torso_1 to torso_2
    (3, 4),   # torso_2 to neck
    (4, 5),   # neck to head
    (3, 6),   # torso_2 to l_shoulder
    (6, 7),   # l_shoulder to l_up_arm
    (7, 8),   # l_up_arm to l_low_arm
    (8, 9),   # l_low_arm to l_hand
    (3, 10),  # torso_2 to r_shoulder
    (10, 11), # r_shoulder to r_up_arm
    (11, 12), # r_up_arm to r_low_arm
    (12, 13), # r_low_arm to r_hand
    (0, 14),  # root to l_up_leg
    (14, 15), # l_up_leg to l_low_leg
    (15, 16), # l_low_leg to l_foot
    (0, 17),  # root to r_up_leg
    (17, 18), # r_up_leg to r_low_leg
    (18, 19), # r_low_leg to r_foot
]



def is_field(name):
    """
    The is_field function quoted from:
    Project: mcp-receiver
    URL: https://github.com/seagetch/mcp-receiver
    LICENSE: MIT
    Copyright (c) 2022 seagetch
    """
    return name.isalpha()
 
def _deserialize(data, index, length, is_list=False):
    """
    The deserialize function quoted from:
    Project: mcp-receiver
    URL: https://github.com/seagetch/mcp-receiver
    LICENSE: MIT
    Copyright (c) 2022 seagetch
    """
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
    """
    The process_packet function quoted from:
    Project: mcp-receiver
    URL: https://github.com/seagetch/mcp-receiver
    LICENSE: MIT
    Copyright (c) 2022 seagetch
    """
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
    """
    The MocopiReciever class was based on:
    Project: mocopi_ros
    URL: https://github.com/hello-world-lab/mocopi_ros
    LICENSE: MIT
    """
    def __init__(self):
        super().__init__('mocopi_receiver')
        self.br = tf2_ros.TransformBroadcaster(self)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("", 12351))
        self.get_logger().info("Mocopi receiver started")
        self.timer = self.create_timer(0.01, self.receive_data)
 
    def receive_data(self):
        try:
            message, _ = self.socket.recvfrom(2048)
            data = _process_packet(message)
            self.broadcast_transforms(data)
        except KeyError as e:
            self.get_logger().error(f"Socket error: {e}")
 
    def make_tf(self, pframe_id, cframe_id, data):
        t = TransformStamped()
        t.header.frame_id = joint_map[pframe_id] if 0 <= pframe_id <= 26 else "map"
 
        for btdt in data["fram"]["btrs"]:
            if btdt["bnid"] == cframe_id:
                t.child_frame_id = joint_map[cframe_id]
                trans = btdt["tran"]
 
                t.header.stamp = self.get_clock().now().to_msg()
                t.transform.translation.x = trans[6]
                t.transform.translation.y = trans[4]
                t.transform.translation.z = trans[5]
                t.transform.rotation.x = trans[2]
                t.transform.rotation.y = trans[0]
                t.transform.rotation.z = trans[1]
                t.transform.rotation.w = trans[3]
                return t
        return None
 
    def broadcast_transforms(self, data):
        transforms = []
 
        if "fram" in data:
            for (p, c) in pairs:
                trans = self.make_tf(p, c, data)
                if trans:
                    transforms.append(trans)
 
        if transforms:
            self.br.sendTransform(transforms)
 
def main():
    rclpy.init()
    node = MocopiReceiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
