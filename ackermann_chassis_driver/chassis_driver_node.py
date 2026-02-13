#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import threading
import struct
from collections import deque
from typing import Optional, Tuple

import serial

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time as TimeMsg


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    """仅绕Z轴的四元数"""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class XProtocol:
    FRAME_HEAD = b"\xAA\x55"
    CMD_VELOCITY = 0x50
    CMD_ROBOT_TYPE = 0x5A

    # 0x10 是综合上报（IMU/速度/电压等），这里先占位
    CMD_REPORT = 0x10

    def __init__(self, port: str, baud: int):
        self.ser = serial.Serial(port, baud, timeout=0.02, write_timeout=0.1)
        self.lock = threading.Lock()

        self.tx_queue = deque(maxlen=5)
        self.last_tx_time = time.time()
        self.last_cmd_time = time.time()

        self.emergency_flag = threading.Event()

        # 启动线程
        self._stop = False
        threading.Thread(target=self._tx_worker, daemon=True).start()
        threading.Thread(target=self._rx_worker, daemon=True).start()

        # 设置阿克曼型号 = 0x01（R10_AKM）
        self.send_robot_type(0x01)

        # 回调（由外部注入）
        self.on_report_frame = None  # type: Optional[callable]

    def close(self):
        self._stop = True
        self.emergency_flag.set()
        time.sleep(0.05)
        try:
            self.ser.close()
        except Exception:
            pass

    def _pack(self, fid: int, data: bytes) -> bytes:
        # length = head(2)+len(1)+fid(1)+data+checksum(1) => 5 + len(data)
        length = 5 + len(data)
        checksum = (0xAA + 0x55 + length + fid + sum(data)) & 0xFF
        return bytes([0xAA, 0x55, length, fid]) + data + bytes([checksum])

    def _send_immediate(self, frame: bytes) -> bool:
        if not self.ser or not self.ser.is_open:
            return False
        try:
            with self.lock:
                self.ser.write(frame)
            self.last_tx_time = time.time()
            return True
        except Exception:
            return False

    def _tx_worker(self):
        while not self._stop:
            if self.emergency_flag.is_set():
                # 急停：发0速度
                frame = self._pack(self.CMD_VELOCITY, struct.pack(">hhh", 0, 0, 0))
                self._send_immediate(frame)
                time.sleep(0.05)
                continue

            if self.tx_queue:
                frame = self.tx_queue.popleft()
                self._send_immediate(frame)
            else:
                time.sleep(0.001)

    def _rx_worker(self):
        """简单帧同步读取：AA 55 + length + fid + ... + checksum"""
        buf = bytearray()
        while not self._stop:
            try:
                chunk = self.ser.read(128)
                if chunk:
                    buf.extend(chunk)
                else:
                    time.sleep(0.001)
                    continue

                # 解析循环
                while True:
                    # 找帧头
                    idx = buf.find(self.FRAME_HEAD)
                    if idx < 0:
                        # 丢掉太长垃圾
                        if len(buf) > 2048:
                            buf.clear()
                        break
                    if idx > 0:
                        del buf[:idx]

                    if len(buf) < 4:
                        break

                    length = buf[2]
                    if length < 5 or length > 255:
                        # 异常长度，跳过一个字节重新找
                        del buf[0:1]
                        continue

                    if len(buf) < length:
                        break

                    frame = bytes(buf[:length])
                    del buf[:length]

                    # 校验
                    fid = frame[3]
                    data = frame[4:-1]
                    checksum = frame[-1]
                    calc = (0xFF + length + fid + sum(data)) & 0xFF
                    if calc != checksum:
                        continue

                    # 回调上报帧
                    if fid == self.CMD_REPORT and self.on_report_frame:
                        self.on_report_frame(data)

            except Exception:
                time.sleep(0.01)

    def send_robot_type(self, robot_type: int):
        frame = self._pack(self.CMD_ROBOT_TYPE, bytes([robot_type & 0xFF]))
        self._send_immediate(frame)
        time.sleep(0.05)

    def send_velocity(self, x_mps: float, w_rps: float):
        """x,w -> 放大1000倍 -> int16，按你的协议发 >hhh (x,y,w)"""
        x_mm = int(max(-2000, min(2000, x_mps * 1000.0)))
        w_mm = int(max(-2000, min(2000, w_rps * 1000.0)))
        data = struct.pack(">hhh", x_mm, 0, w_mm)
        frame = self._pack(self.CMD_VELOCITY, data)

        if len(self.tx_queue) < self.tx_queue.maxlen:
            self.tx_queue.append(frame)

        self.last_cmd_time = time.time()

    def set_emergency(self, enable: bool):
        if enable:
            self.emergency_flag.set()
        else:
            self.emergency_flag.clear()


class AckermannChassisDriver(Node):
    def __init__(self):
        super().__init__("ackermann_chassis_driver")

        # 参数
        self.declare_parameter("serial_port", "/dev/serial0")
        self.declare_parameter("serial_baud", 230400)

        self.declare_parameter("wheelbase", 0.148)
        self.declare_parameter("max_steering_angle_deg", 45.0)

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_publish_hz", 50.0)

        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("max_linear_speed", 0.6)
        self.declare_parameter("max_angular_speed", 3.0)

        self.declare_parameter("use_reported_twist_from_board", False)

        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baud = self.get_parameter("serial_baud").get_parameter_value().integer_value

        self.wheelbase = float(self.get_parameter("wheelbase").value)
        self.max_steer = math.radians(float(self.get_parameter("max_steering_angle_deg").value))

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        self.cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)
        self.max_v = float(self.get_parameter("max_linear_speed").value)
        self.max_w = float(self.get_parameter("max_angular_speed").value)

        self.use_reported_twist = bool(self.get_parameter("use_reported_twist_from_board").value)

        # 串口
        self.proto = XProtocol(port, int(baud))
        self.proto.on_report_frame = self._on_report_0x10

        # ROS 接口
        self.sub_cmd = self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self.pub_odom = self.create_publisher(Odometry, "odom", 20)

        self.tf_broadcaster = TransformBroadcaster(self)

        # 里程计状态
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # 用于积分的速度（默认用下发速度；以后可切换用回传）
        self.v_last = 0.0
        self.w_last = 0.0

        self.last_odom_time = self.get_clock().now()

        hz = float(self.get_parameter("odom_publish_hz").value)
        self.timer = self.create_timer(1.0 / max(1.0, hz), self._on_timer)

        self.get_logger().info(f"Chassis driver started. port={port}, baud={baud}")

    def destroy_node(self):
        try:
            self.proto.close()
        except Exception:
            pass
        super().destroy_node()

    def _apply_ackermann_constraint(self, v: float, w: float) -> Tuple[float, float]:
        # 限幅
        v = max(-self.max_v, min(self.max_v, v))
        w = max(-self.max_w, min(self.max_w, w))

        # 阿克曼几何约束：|w| <= |v| * tan(delta_max) / L
        if abs(v) > 1e-3:
            w_lim = abs(v) * math.tan(self.max_steer) / self.wheelbase
            w = max(-w_lim, min(w_lim, w))
        else:
            # v 很小：不允许“原地转”
            w = 0.0
        return v, w

    def _on_cmd_vel(self, msg: Twist):
        v = float(msg.linear.x)
        w = float(msg.angular.z)
        v, w = self._apply_ackermann_constraint(v, w)

        self.proto.set_emergency(False)
        self.proto.send_velocity(v, w)

        # 记录用于积分
        self.v_last = v
        self.w_last = w

    def _on_report_0x10(self, data: bytes):
        """
        这里是 0x10 综合回传数据帧 payload（不含AA55/len/fid/checksum）
        你需要根据手册把 data 的字段 unpack 出来：
        - 至少拿到线速度 v、角速度 w（或者轮速/舵角也行）
        - 也可以拿IMU、battery等
        """
        if not self.use_reported_twist:
            return

        # TODO: 按手册补齐解析
        # 示例（占位）：假设前4字节是 int16 v_mmps, int16 w_millirad
        # v_mmps, w_millirad = struct.unpack(">hh", data[0:4])
        # self.v_last = v_mmps / 1000.0
        # self.w_last = w_millirad / 1000.0
        pass

    def _on_timer(self):
        # watchdog：超过 timeout 没收到新 cmd，就急停
        now_wall = time.time()
        if now_wall - self.proto.last_cmd_time > self.cmd_timeout:
            self.proto.set_emergency(True)
            self.v_last = 0.0
            self.w_last = 0.0

        # 里程计积分
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        self.last_odom_time = now

        v = self.v_last
        w = self.w_last

        # simple unicycle integration（对阿克曼来说，低速足够用；后续你可换成用舵角+后轮速）
        self.yaw += w * dt
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        self._publish_odom(now, v, w)
        if self.publish_tf:
            self._publish_tf(now)

    def _publish_odom(self, stamp: Time, v: float, w: float):
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.pub_odom.publish(odom)

    def _publish_tf(self, stamp: Time):
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        qx, qy, qz, qw = yaw_to_quat(self.yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = AckermannChassisDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
