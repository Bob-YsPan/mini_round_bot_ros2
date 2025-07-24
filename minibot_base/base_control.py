from threading import Thread
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from collections import deque
import tf2_ros
import serial
import time
import sys
import math
import struct
from tf_transformations import quaternion_from_euler

class BaseControl(Node):
    def __init__(self):
        super().__init__('base_control')
        
        # 定義參數以及預設值
        # a.k.a. Child Frame
        self.declare_parameter('base_id', 'base_footprint')
        # a.k.a. Header Frame
        self.declare_parameter('odom_id', 'odom')
        # 這邊按照舊有的設定，轉移成ROS2的格式
        self.declare_parameter('port', '/dev/stm32base')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('odom_freq', 20.0)
        self.declare_parameter('wheel_separation', 0.158)
        self.declare_parameter('wheel_radius', 0.032)
        self.declare_parameter('pose_cov', 0.0001)    # Sync the linorobot 2's firmware
        self.declare_parameter('twist_cov', 0.00001)  # Sync the linorobot 2's firmware
        self.declare_parameter('imu_cov', 0.00001)  # Append ImuCov
        self.declare_parameter('threshold_wheel', 0.55)  # Mininum acceptable rotate speed of the wheel
        # 底盤odom訊息需發布到 /odom/unfiltered
        self.declare_parameter('odom_topic', '/odom/unfiltered')
        # (20250401)底盤Z軸陀螺儀訊息需發布到 /imu/data
        self.declare_parameter('imu_topic', '/imu/data')
        # (20250422)打開後可以由這邊解算TF資料，方便使用者檢查底盤程式狀況
        self.declare_parameter('pub_tf', False)
        # 測試底盤訊號用(除錯再打開)
        self.declare_parameter('debug_mode', False)
        
        # 將餐數轉為內部變數
        self.baseId = self.get_parameter('base_id').get_parameter_value().string_value
        self.odomId = self.get_parameter('odom_id').get_parameter_value().string_value
        self.device_port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.odom_freq = self.get_parameter('odom_freq').get_parameter_value().double_value
        self.wheelSep = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.wheelRad = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.PoseCov = self.get_parameter('pose_cov').get_parameter_value().double_value
        self.TwistCov = self.get_parameter('twist_cov').get_parameter_value().double_value
        self.ImuCov = self.get_parameter('imu_cov').get_parameter_value().double_value
        self.threshold_wheel = self.get_parameter('threshold_wheel').get_parameter_value().double_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.pub_tf = self.get_parameter('pub_tf').get_parameter_value().bool_value
        self.debug_mode = self.get_parameter('debug_mode').get_parameter_value().bool_value
        
        if self.debug_mode:
            self.get_logger().warning(f"WARNING: Debug mode enabled!, /rosout will be messy!!!")

        # 設定串列通訊
        try:
            # 通訊逾時調整成10ms，以免等待資料卡住整個程式
            self.serial = serial.Serial(self.device_port, self.baudrate, timeout=0.01)
            time.sleep(1)
        except serial.serialutil.SerialException:
            self.get_logger().error(f"Can not receive data from the port: {self.device_port}. Did you specify the correct port?")
            sys.exit(0)
        # 通訊建立列印訊息
        self.get_logger().info("Communication success !")
        
        # 設定 ROS 訂閱與發布
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmdCB, 10)   # 控制訊號
        self.pub = self.create_publisher(Odometry, self.odom_topic, 10)         # 車體位置(Odom)
        self.pub_imu = self.create_publisher(Imu, self.imu_topic, 10)           # (20250403) Z陀螺儀訊號(Imu)

        # 建立兩個Timer，負責處理位置訊息發布、控制命令寫入
        self.timer_odom = self.create_timer(1.0/self.odom_freq, self.timerOdomCB)   # 定義於上方，原廠為10Hz

        # 發布TF轉換
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 變數初始化
        self.trans_x = 0.0
        self.rotat_z = 0.0
        self.WL_send = 0.0
        self.WR_send = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.step = 0
        self.buf = b""
        self.safestop = False
        self.reading_loop_flag = True
        # Put the signal receive from base
        self.base_signal_deque = deque(maxlen=3)
        # Put the signal that receive from cmd_vel
        self.cmd_signal_deque = deque(maxlen=3)
        self.current_time = self.get_clock().now()
        self.previous_time_vel = self.current_time
        self.previous_time_odom = self.current_time
        self.previous_time_cmd = self.current_time

        # 程式開始先寫入無速度的指令
        # 將資料轉換為32bit浮點數(無線速度&旋轉)
        WR_send_ba = struct.pack("f", 0.0)
        WL_send_ba = struct.pack("f", 0.0)
        # 將資料與Header、Function Code、Footer合併
        output = b'SV' + WR_send_ba + WL_send_ba + b'E'
        # 除錯用
        if self.debug_mode:
            self.get_logger().info(f"Initial test command = {output}")
        # 寫入初始命令
        self.serial.write(output)
        # 等待底盤反應
        time.sleep(1)
        # 啟動執行緒接收
        self.base_thread = Thread(target=self.reading_loop, daemon=True)
        self.base_thread.start()

    # Function to write speed
    def write_spd(self, vx: float, vrz: float):
        # 反解運動學
        # Sep = 0.158 Rad = 0.032 vx = 0.03 >> WR = WL 大約 0.95
        # Min WR at linear 0.03: (0.03 + 0.158 / 2.0 * 0) / 0.032 = 0.9375
        # Min rad: (0 + 0.158 / 2.0 * vrz) / 0.032 = 0.9375
        # 0.079 * vrz = 0.03, vrz = 0.3797468354 >> 大約抓 0.5 rad/s
        # 剛好移動 Min rad = 0.45
        # 0.45 = (0 + 0.158 / 2.0 * vrz) / 0.032
        # 0.0144 = 0.079 * vrz
        # vrz = 0.1822784810 >> 抓 0.3 rad/s
        WR = (vx + self.wheelSep / 2.0 * vrz) / self.wheelRad
        WL = (vx - self.wheelSep / 2.0 * vrz) / self.wheelRad
        # 解算出太小的數值，以最小容許的數值旋轉
        def limit_to_threshold(value, threshold=self.threshold_wheel):
            if abs(value) < threshold:
                if value > 0:
                    return threshold
                elif value < 0:
                    return -threshold
            return value
        if (abs(WR) < self.threshold_wheel):
            WR = limit_to_threshold(WR)
        if (abs(WL) < self.threshold_wheel):
            WL = limit_to_threshold(WL)
        # 將資料轉換為32bit浮點數
        WR_send_ba = struct.pack("f", WR)
        WL_send_ba = struct.pack("f", WL)
        # 除錯用
        if self.debug_mode:
            self.get_logger().info(f"Send vx = {str(vx)}, vrz = {str(vrz)}, WR = {str(WR)}, WL = {str(WL)}")
        # 將資料與Header、Function Code、Footer合併
        output = b'SV' + WR_send_ba + WL_send_ba + b'E'
        # 除錯用
        if self.debug_mode:
            self.get_logger().info(f"Send packet = {output}")
        # 寫入初始命令
        self.serial.write(output)

    # 持續等待資料、拉取序列埠收到訊息、分類訊息
    def reading_loop(self):
        # 切換狀態的變數
        self.safestop = False
        self.get_logger().info("Base receiver thread started!")
        while(self.reading_loop_flag):
            self.current_time = self.get_clock().now()
            # 發送部分
            # Have data in the deque
            if(len(self.cmd_signal_deque) > 0):
                # 抽出Buffer最早的資料
                self.trans_x, self.rotat_z = self.cmd_signal_deque.popleft()
                # Reset timer
                self.previous_time_cmd = self.current_time
                self.safestop = False
            else:
                # 超過500ms上游沒有控制命令，則安全停車
                if((self.current_time - self.previous_time_cmd).nanoseconds / 1e6 > 500.0 and not self.safestop):
                    self.trans_x, self.rotat_z = 0.0, 0.0
                    self.write_spd(self.trans_x, self.rotat_z)
                    self.get_logger().info("Safe stop!")
                    self.safestop = True
                # 沒有命令時，每50ms發送目前速度訊號以確保取得底盤狀態回傳
                if((self.current_time - self.previous_time_vel).nanoseconds / 1e6 > 50.0):
                    # 再送出一次當前速度
                    self.write_spd(self.trans_x, self.rotat_z)
                    # Reset timer
                    self.previous_time_vel = self.current_time
            # 接收部分
            # Step 0: Wait data coming
            if(self.step == 0):
                # Try to read buffer
                self.buf = self.serial.read(1)
                # Check vaild header
                if self.buf == b'S':
                    if(self.debug_mode):
                        self.get_logger().info("Header vaild!")
                    self.step = 1
            # Step 1: Second header check and packet classification
            elif(self.step == 1):
                self.buf = self.serial.read(1)
                # Battery message
                if self.buf == b'B':
                    self.step = 2
                # Velocity message
                elif self.buf == b'V':
                    self.step = 3
            # Step 2: Battery message vaildation
            elif(self.step == 2):
                self.buf = self.serial.read(5)
                # Length check
                if(self.buf < 5):
                    # Not match, drop it
                    self.step = 0
                else:
                    # Footer check
                    if (self.buf[-1] == ord('E')):
                        self.get_logger().info(f"Got battery message and vaild!")
                        step = 0
                    else:
                        self.get_logger().warn(f"Packet battery footer vaildation fail!, footer = {buf[-1]}")
                        step = 0
            # Step 3: Velocity message vaildation
            elif(self.step == 3):
                self.buf = self.serial.read(13)
                # Length check
                if(len(self.buf) < 13):
                    # Not match, drop it
                    self.step = 0
                else:
                    # Footer check
                    if (self.buf[-1] == ord('E')):
                        if self.debug_mode:
                            self.get_logger().info(f"Got velocity message and vaild!")
                        self.step = 4
                    else:
                        self.get_logger().warn(f"Packet velocity footer vaildation fail!, footer = {buf[-1]}")
                        self.step = 0
            # Step 4: Velocity handler
            elif(self.step == 4):
                try:
                    # 解碼資料回浮點數
                    VR = struct.unpack('f', self.buf[0:4])[0]
                    VL = struct.unpack('f', self.buf[4:8])[0]
                    gyro_z = struct.unpack('f', self.buf[8:12])[0]
                    # VR、VL、gyro_z，小於 0.01 時，使其值為 0
                    # VR, VL, gyro_z = map(lambda v: 0.0 if abs(v) < 0.01 else v, [VR, VL, gyro_z])
                    # 解算差動車體運動學 (Unit of VR and VS are rps)
                    VR *= self.wheelRad
                    VL *= self.wheelRad
                    Vyaw = (VR - VL) / self.wheelSep
                    Vx = (VR + VL) / 2.0
                    if self.debug_mode:
                        self.get_logger().info(f"VR = {VR}, VL = {VL}, gyro_z = {gyro_z}")
                    # 放進Deque
                    self.base_signal_deque.append((self.current_time, Vx, Vyaw, gyro_z))
                    self.step = 0
                except Exception as e:
                    self.get_logger().error(f"Velocity parse error: {e}")
                    self.step = 0
        
    # 接收到cmd_vel的控制指令時，將最新的控制指令儲存
    def cmdCB(self, msg):
        # 將新的控制命令儲存
        self.cmd_signal_deque.append(msg.linear.x, msg.angular.z)
    
    # 更新最新的機器人Odom訊息
    def timerOdomCB(self):
        if self.debug_mode:
            self.get_logger().info("OdomCB Triggered!")
        try:
            if(len(self.base_signal_deque) > 0):
                # 取出最新資料
                ts_signal, Vx, Vyaw, gyro_z = self.base_signal_deque.popleft()
                # 計算時間差
                dt = (ts_signal - self.previous_time_odom).nanoseconds / 1e9
                self.previous_time_odom = ts_signal
                self.pose_x += Vx * math.cos(self.pose_yaw) * dt
                self.pose_y += Vx * math.sin(self.pose_yaw) * dt
                self.pose_yaw += Vyaw * dt
                pose_quat = quaternion_from_euler(0, 0, self.pose_yaw)
                # 包裝 Odometry 訊息
                msg = Odometry()
                msg_imu = Imu()
                msg.header.stamp = ts_signal.to_msg()   # 時間戳記
                msg.header.frame_id = self.odomId   # Header Frame(需要跟EKF符合)
                msg.child_frame_id = self.baseId    # Child Frame(需要跟EKF符合)
                # 計算出的X、Y、Z座標(Z始終為0，因為是2D平面)
                msg.pose.pose.position.x = self.pose_x
                msg.pose.pose.position.y = self.pose_y
                msg.pose.pose.position.z = 0.0
                # 角度四元數
                msg.pose.pose.orientation.x = pose_quat[0]
                msg.pose.pose.orientation.y = pose_quat[1]
                msg.pose.pose.orientation.z = pose_quat[2]
                msg.pose.pose.orientation.w = pose_quat[3]
                msg.twist.twist.linear.x = Vx
                msg.twist.twist.angular.z = Vyaw
                # 斜方差訊息
                # 250501 >> Sync the linorobot2's firmware
                for i in range(36):
                    msg.twist.covariance[i] = 0
                    msg.pose.covariance[i] = 0
                msg.twist.covariance[0] = self.TwistCov
                msg.twist.covariance[7] = self.TwistCov
                msg.twist.covariance[35] = self.TwistCov
                msg.pose.covariance[0] = self.PoseCov
                msg.pose.covariance[7] = self.PoseCov
                msg.pose.covariance[35] = self.PoseCov
                # (20250403) IMU 資料包裝
                # 只有Z軸旋轉有效，換算deg to rad * 10
                gyro_z *= 0.17453
                msg_imu.angular_velocity.z = gyro_z
                if self.debug_mode:
                    self.get_logger().info(f"Converted gyro = {str(gyro_z)}")
                # 其他欄位保險起見填0
                msg_imu.angular_velocity.x = 0.0
                msg_imu.angular_velocity.y = 0.0
                msg_imu.linear_acceleration.x = 0.0
                msg_imu.linear_acceleration.y = 0.0
                msg_imu.linear_acceleration.z = 0.0
                msg_imu.orientation_covariance[0] = 0.0
                msg_imu.linear_acceleration_covariance[0] = 0.0
                for i in range(9):
                    msg_imu.angular_velocity_covariance[i] = 0.0
                msg_imu.angular_velocity_covariance[8] = self.ImuCov
                # 發布訊息
                self.pub.publish(msg)
                # (20250403) IMU 資料發布
                self.pub_imu.publish(msg_imu)
                # 發布 TF (轉成ROS2格式，需要一個TransformStamped物件)
                # ROS1 = self.tf_broadcaster.sendTransform( (self.pose_x, self.pose_y, 0.0), pose_quat, self.current_time, self.baseId, self.odomId)
                if self.pub_tf:
                    t = tf2_ros.TransformStamped()
                    t.header.stamp = ts_signal.to_msg()
                    t.header.frame_id = self.odomId
                    t.child_frame_id = self.baseId
                    t.transform.translation.x = self.pose_x
                    t.transform.translation.y = self.pose_y
                    t.transform.translation.z = 0.0
                    t.transform.rotation.x = pose_quat[0]
                    t.transform.rotation.y = pose_quat[1]
                    t.transform.rotation.z = pose_quat[2]
                    t.transform.rotation.w = pose_quat[3]
                    self.tf_broadcaster.sendTransform(t)
        # 發生例外的處理
        except Exception as e:
            self.get_logger().error(f"Error in sensor value: {e}")
        
def main(args=None):
    rclpy.init(args=args)
    # 建立、啟動節點
    node = BaseControl()
    try:
        rclpy.spin(node)
    # 鍵盤 Ctrl + C
    except KeyboardInterrupt:
        node.get_logger().info("Shutting Down!")
        # Join thread to wait it shut down
        node.reading_loop_flag = False
        node.base_thread.join()
    # 其他例外
    except Exception as e:
        node.get_logger().error(f"Error occurs! {e}\n{e.printStackTrace}")
    finally:
        # 回收節點
        node.destroy_node()
        # 檢查rclpy是否還在運作，是的話一同關閉
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
