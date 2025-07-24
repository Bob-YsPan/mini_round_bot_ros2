import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
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
        self.declare_parameter('odom_freq', 10.0)
        self.declare_parameter('wheel_separation', 0.158)
        self.declare_parameter('wheel_radius', 0.032)
        # self.declare_parameter('vx_cov', 1.0)
        # self.declare_parameter('vyaw_cov', 1.0)
        self.declare_parameter('vx_cov', 0.00001)    # Sync the linorobot 2's firmware
        self.declare_parameter('vyaw_cov', 0.00001)  # Sync the linorobot 2's firmware
        self.declare_parameter('vy_cov', 0.00001)  # Append VyCov
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
        self.VxCov = self.get_parameter('vx_cov').get_parameter_value().double_value
        self.VyCov = self.get_parameter('vy_cov').get_parameter_value().double_value
        self.VyawCov = self.get_parameter('vyaw_cov').get_parameter_value().double_value
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
        # self.pub_imu = self.create_publisher(Imu, self.imu_topic, 10)           # (20250403) Z陀螺儀訊號(Imu)

        # 建立兩個Timer，負責處理位置訊息發布、控制命令寫入
        self.timer_odom = self.create_timer(1.0/self.odom_freq, self.timerOdomCB)   # 定義於上方，原廠為10Hz
        self.timer_cmd = self.create_timer(0.1, self.timerCmdCB)  # 10Hz

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
        self.current_time = self.get_clock().now()
        self.previous_time = self.get_clock().now()
        self.current_time_cmd = self.get_clock().now()
        self.previous_time_cmd = self.get_clock().now()

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
        # 拉取第一筆資料
        self.reading_loop()

    # 持續等待資料、拉取序列埠收到訊息、分類訊息
    def reading_loop(self):
        # 嘗試接收1 byte
        reading = self.serial.read(1)
        # 除錯用
        if self.debug_mode:
            self.get_logger().info("One reading loop!")
        # 判斷接收到的1 byte是不是Header(這裡是比對整個bytes資料)
        if reading == b'S':
            # 再往下接收Function Code
            function_code = self.serial.read(1)
            # 除錯用
            if self.debug_mode:
                self.get_logger().info(f"Function code: {function_code}")
            # Function Code比對
            if function_code == b'B':        #'B' >> 電量資訊
                # 嘗試讀取至Footer
                serial_buf = self.serial.read(5)
                # 除錯用
                if self.debug_mode:
                    self.get_logger().info(f"Received: {str(serial_buf)}")
                # 這裡為取出單個byte進行檢查，比對對象需要加上ord才可以進行比對
                if serial_buf[4] == ord('E'):      #'E' >> 檢查是不是Footer
                    # 電量資料存入變數
                    self.batt_fb = serial_buf
                    # 除錯用
                    if self.debug_mode:
                        self.get_logger().info("Type: batt fb")
                else:
                    # Footer檢查失敗
                    self.get_logger().error(f"Footer not match: {str(serial_buf[4])}")

            if function_code == b'V':        #'V' >> 底盤速度資訊
                # 嘗試讀取至Footer
                serial_buf = self.serial.read(13)
                # 除錯用
                if self.debug_mode:
                    self.get_logger().info(f"Received: {str(serial_buf)}")
                if serial_buf[12] == ord('E'):      #'E' >> 檢查是不是Footer
                    # 速度資料存入變數，等待下一步處理
                    self.vel_fb = serial_buf
                    # 除錯用
                    if self.debug_mode:
                        self.get_logger().info("Type: vel fb")
                else:
                    self.get_logger().error(f"Footer not match: {str(serial_buf[12])}")
        
    # 接收到cmd_vel的控制指令時，將最新的控制指令儲存
    def cmdCB(self, msg):
        # 重設累計時間
        self.current_time_cmd = self.get_clock().now()
        self.previous_time_cmd = self.current_time_cmd
        # 將新的控制命令儲存
        self.trans_x = msg.linear.x
        self.rotat_z = msg.angular.z
    
    # 更新最新的機器人Odom訊息
    def timerOdomCB(self):
        # 探測一次序列埠的資料
        self.reading_loop()
        # 除錯用
        if self.debug_mode:
            self.get_logger().info("OdomCB Triggered!")
        try:
            # 抓取目前得到的vel_fb資料(序列埠收來的原始值)
            vel_fb = self.vel_fb
            # 資料長度檢查
            if len(vel_fb) == 13:
                # 解碼資料回浮點數
                VR = struct.unpack('f', vel_fb[0:4])[0]
                VL = struct.unpack('f', vel_fb[4:8])[0]
                gyro_z = struct.unpack('f', vel_fb[8:12])[0]
            else:
                # 長度不符
                self.get_logger().error("vel_fb Error!")
                return
            # 解出所有Odom需要的數值(位置&姿態)
            # VR、VL、gyro_z，小於 0.01 時，使其值為 0
            VR, VL, gyro_z = map(lambda v: 0.0 if abs(v) < 0.01 else v, [VR, VL, gyro_z])
            if self.debug_mode:
                self.get_logger().info(f"Current gyro = {str(gyro_z)}")
            # V = omega * radius, unit: m/s
            VR *= self.wheelRad
            VL *= self.wheelRad
            Vyaw = (VR - VL) / self.wheelSep
            Vx = (VR + VL) / 2.0
            # 計算時間差
            self.current_time = self.get_clock().now()
            dt = (self.current_time - self.previous_time).nanoseconds / 1e9
            self.previous_time = self.current_time
            self.pose_x += Vx * math.cos(self.pose_yaw) * dt
            self.pose_y += Vx * math.sin(self.pose_yaw) * dt
            self.pose_yaw += Vyaw * dt
            pose_quat = quaternion_from_euler(0, 0, self.pose_yaw)
            # 包裝 Odometry 訊息
            msg = Odometry()
            msg_imu = Imu()
            msg.header.stamp = self.current_time.to_msg()   # 時間戳記
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
            msg.twist.covariance[0] = self.VxCov
            msg.twist.covariance[7] = self.VyCov
            msg.twist.covariance[35] = self.VyawCov
            msg.pose.covariance = msg.twist.covariance
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
            msg_imu.orientation_covariance[0] = -1  # 沒有資料的協方差設為未知
            msg_imu.linear_acceleration_covariance[0] = -1  # 沒有資料的協方差設為未知
            for i in range(9):
                msg_imu.angular_velocity_covariance[i] = 0.0
            msg_imu.angular_velocity_covariance[8] = self.VyawCov
            # 發布訊息
            self.pub.publish(msg)
            # (20250403) IMU 資料發布
            # self.pub_imu.publish(msg_imu)
            # 發布 TF (轉成ROS2格式，需要一個TransformStamped物件)
            # ROS1 = self.tf_broadcaster.sendTransform( (self.pose_x, self.pose_y, 0.0), pose_quat, self.current_time, self.baseId, self.odomId)
            if self.pub_tf:
                t = tf2_ros.TransformStamped()
                t.header.stamp = self.current_time.to_msg()
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
    
    # 定時將cmd_vel收到的最新資料傳送至底盤控制器，並檢查是否已經超過0.5秒沒有發送新速度指令
    def timerCmdCB(self):
        # 抓取現在的時間戳
        self.current_time_cmd = self.get_clock().now()
        # 檢查距離上次觸發間隔時間
        dt_cmd = (self.current_time_cmd - self.previous_time_cmd).nanoseconds / 1e9
        # 超過0.5秒，將速度寫 0 ，之後於下方送出
        if(dt_cmd > 0.5):
            self.trans_x, self.rotat_z = 0, 0
            self.previous_time_cmd = self.current_time_cmd
            if self.debug_mode:
                # 除錯用
                self.get_logger().info("No new command after 0.5s, Safe Stop!")
        # 除錯用
        if self.debug_mode:
            self.get_logger().info("CmdCB Triggered!")
        # 計算底盤運動學(跟舊程式算法相同)
        WR = (self.trans_x + self.wheelSep / 2.0 * self.rotat_z) / self.wheelRad
        WL = (self.trans_x - self.wheelSep / 2.0 * self.rotat_z) / self.wheelRad
        # 計算出來的速度轉換為32bit浮點數
        WR_send_ba = struct.pack("f", WR)
        WL_send_ba = struct.pack("f", WL)
        # 除錯用
        if self.debug_mode:
            self.get_logger().info(f"Send WR = {str(WR)}, WL = {str(WL)}")
        # 跟Header、Function Code、Footer包裝成控制命令
        output = b'SV' + WR_send_ba + WL_send_ba + b'E'
        # 除錯用
        if self.debug_mode:
            self.get_logger().info(f"Command = {output}")
        # 寫入底盤控制器
        self.serial.write(output)
        
def main(args=None):
    rclpy.init(args=args)
    # 建立、啟動節點
    node = BaseControl()
    try:
        rclpy.spin(node)
    # 鍵盤 Ctrl + C
    except KeyboardInterrupt:
        node.get_logger().info("Shutting Down!")
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
