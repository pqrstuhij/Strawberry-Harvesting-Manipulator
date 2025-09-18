#!/usr/bin/env python3
import rospy
from co_robot.srv import vision_robot
import socket
import time

SERVER_IP = "192.168.1.207"
SERVER_PORT = 9000
MAX_MESSAGE_SIZE = 100

sock = None
prev_x = None
prev_y = None
prev_z = None

def tcp_client_start():
    global sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((SERVER_IP, SERVER_PORT))
        print(f"Connected to TCP server at {SERVER_IP}:{SERVER_PORT}")
    except socket.error as e:
        print("연결 실패:", e)
        exit(1)

def tcp_client_send(msg):
    if len(msg) >= MAX_MESSAGE_SIZE:
        msg = msg[:MAX_MESSAGE_SIZE - 1]
    try:
        sock.sendall(msg.encode())
        print(f"Sent to server: {msg}")
    except socket.error as e:
        print("메시지 전송 실패:", e)

def tcp_client_receive():
    try:
        received = sock.recv(MAX_MESSAGE_SIZE).decode()
        if received:
            print(f"Received from server: {received}")
        return received
    except socket.error as e:
        print("응답 수신 실패:", e)
        return None

def robot_move(req):
    global prev_x, prev_y, prev_z

    val_1 = str(req.x)
    val_2 = str(req.y)
    val_3 = str(req.z)
    null = " "

    # (1) 좌표가 모두 0일 경우 무시
    if req.x == 0 and req.y == 0 and req.z == 0:
        return 100

    # (2) x 값이 허용 범위 벗어나면 무시
    if req.x > 590 or req.x < 298:
        rospy.loginfo("X value not sending the command")
        return 100

    # (3) 이전 값과 너무 비슷하면 무시
    if all(v is not None for v in [prev_x, prev_y, prev_z]):
        diff_x = abs(req.x - prev_x)
        diff_y = abs(req.y - prev_y)
        diff_z = abs(req.z - prev_z)
        if diff_x < 50 and diff_y < 50 and diff_z < 50:
            rospy.loginfo("이전값과 너무 유사하여 무시")
            return 100

    # (4) 명령 전송
    output = val_1 + "," + val_2 + "," + val_3 + "," + null
    print(f"명령 전송: {output}")

    received = tcp_client_receive()
    if received == "ok":
        tcp_client_send(output)

    time.sleep(1)

    global count_number
    count = str(count_number)

    received = tcp_client_receive()
    if received == "ok":
        tcp_client_send(count)

    count_number += 1

    # (5) 현재 값 갱신
    prev_x = req.x
    prev_y = req.y
    prev_z = req.z

    time.sleep(1)
    return 100

def main():
    global count_number
    count_number = 1

    tcp_client_start()
    time.sleep(1)

    rospy.init_node("tcp_srv")
    rospy.Service("service", vision_robot, robot_move)
    rospy.spin()

if __name__ == "__main__":
    main()
