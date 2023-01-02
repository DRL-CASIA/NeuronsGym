import socket
import time
import json
import struct

class UDPClient:
    def __init__(self, port=12345, ip=None):
        self.recv_msg = {"time": 0.0, "odom_x": 0.0, "odom_y": 0.0, "odom_yaw": 0.0, "vx": 0.0, "vy": 0.0, "vw": 0.0}
        self.send_msg = {"time": 0.0, "pose_x": 0.0, "pose_y": 0.0, "pose_yaw": 0.0, "laser": [0.0]*61, "collision_info": 0.0}
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if ip is None:
            host = socket.gethostname()
        else:
            host = ip
        self.addr = (host, port)
        self.socket.bind(self.addr)
    
    def build_header(self, data_len):
        header = {"data_length": data_len}
        return json.dumps(header).encode('UTF-8')
    
    def encode_msg(self, data):
        return json.dumps(data).encode('UTF-8')
    
    def send(self, data):
        msg = self.encode_msg(data)
        data_len = len(msg)
        header = self.build_header(data_len)
        header_len = len(header)
        struct_bytes = struct.pack("i", header_len)

        self.socket.sendto(struct_bytes, self.addr)
        self.socket.sendto(header, self.addr)
        self.socket.sendto(msg, self.addr)
    
    def recv(self):
        struct_btyes, _ = self.socket.recvfrom(4)
        header_len = struct.unpack("i", struct_btyes)[0]
        header_bytes, _ = self.socket.recvfrom(header_len)
        if len(header_bytes)!=header_len:
            return None
        try:
            header = json.loads(header_bytes.decode('UTF-8'))
        except Exception as e:
            return None
        keys = header.keys()
        if "data_length" not in keys:
            return None
        data_len = header["data_length"]

        # gap_abs = data_len % 1024
        # count = data_len // 1024
        # print(gap_abs, count)
        # recv_data = b''
        # for _ in range(count):
        #     data, _ = self.socket.recvfrom(1024, socket.MSG_WAITALL)
        #     recv_data += data
        recv_data, _ = self.socket.recvfrom(data_len, socket.MSG_WAITALL)
        if len(recv_data) != data_len:
            return None
        # recv_data += data
        try:
            recv_msg = json.loads(recv_data.decode('UTF-8'))
        except Exception as e:
            return None
        return recv_msg
    
    def update_msg(self):
        self.send_msg["time"] = time.time()

    def close(self):
        self.socket.close()


if __name__ == "__main__":

    udp_client = UDPClient(port=12345, ip="0.0.0.0")
    while True:
        recv_msg = udp_client.recv()
        print("received msg: ", time.time(), " ",recv_msg["time"], ", ", recv_msg["laser"])
        # udp_client.update_msg()
        # udp_client.send(udp_client.send_msg)
        # print("send msg: ", udp_client.send_msg["time"], ", ", udp_client.send_msg["laser"])
        time.sleep(0.02)