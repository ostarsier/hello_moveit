import sys
import csv
import redis
import time

# 初始化Redis连接
redis_client = redis.Redis(host='192.168.110.91', port=6379, db=0)

# 执行话筒轨迹
vla_joint_list = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
    "right_claw_joint"
]

def read_trajectory_file(file_path):
    joint_positions = []
    with open(file_path, 'r') as file:
        reader = csv.reader(file)
        header = next(reader)  # 读取表头
        
        # 创建一个字典，用于映射表头到索引
        header_map = {name: index for index, name in enumerate(header)}
        
        # 读取每一行数据
        for row in reader:
            positions = [0.0] * len(vla_joint_list)
            for joint_name, index in zip(vla_joint_list, range(len(vla_joint_list))):
                if joint_name in header_map:
                    positions[index] = float(row[header_map[joint_name]])
            joint_positions.append(positions)
    
    return joint_positions

def send_to_redis(data):
    for row in data:
        redis_client.set('joint_position', str(row))
        print(f"Data sent to Redis: {row}")
        time.sleep(0.1)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python playback.py <filename_without_extension>")
        sys.exit(1)
    
    filename = sys.argv[1]
    file_path = f"/home/yons/ws_moveit_my/src/mojia/trajectory/{filename}.csv"
    positions = read_trajectory_file(file_path)
    send_to_redis(positions)
