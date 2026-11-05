# 3-wheel omni robot with micro-ROS over Wi-Fi

## GY-25 IMU heading hold

เฟิร์มแวร์อ่านมุม yaw จาก GY-25 ผ่าน UART2 และรักษาทิศหัวหุ่นยนต์ขณะมีคำสั่ง
`linear.x` หรือ `linear.y` แต่ไม่มีคำสั่ง `angular.z` เมื่อสั่งหมุนด้วยจอย การหมุนจาก
ผู้ขับจะมี priority และเมื่อปล่อยแกนหมุน ระบบจะจำทิศใหม่เป็นเป้าหมายทันที

การต่อสาย:

- GY-25 `TX` -> ESP32 `GPIO16` (`RX2`)
- GY-25 `RX` -> ESP32 `GPIO17` (`TX2`)
- GY-25 `GND` -> ESP32 `GND`
- ใช้ baud rate `115200`

หากค่า yaw เพิ่มขึ้นเมื่อหุ่นหมุนตามเข็มนาฬิกา ให้เปลี่ยน
`GY25_YAW_DIRECTION` จาก `1.0f` เป็น `-1.0f` ปรับความแรงการรักษาทิศด้วย
`DEFAULT_HEADING_HOLD_KP`, `DEFAULT_HEADING_HOLD_KD` และ `HEADING_HOLD_MAX_WZ`
หากไม่ได้รับข้อมูล IMU เกิน 300 ms ระบบจะใช้ `angular.z` จาก `/cmd_vel` โดยตรง
โดยอัตโนมัติ

## Encoder, PID และ Nav2

ระบบใช้ quadrature encoder แบบ x4 ควบคุมความเร็วแต่ละล้อด้วย PID และคำนวณ
odometry ของฐาน omni จากความเร็วล้อจริง

| ล้อ | Encoder A | Encoder B |
| --- | ---: | ---: |
| Motor 1 | GPIO32 | GPIO33 |
| Motor 2 | GPIO34 | GPIO35 |
| Motor 3 | GPIO2 | GPIO15 |

GPIO34/35 ไม่มี internal pull-up จึงต้องมีตัวต้านทาน pull-up ภายนอกหากเอาต์พุต
encoder เป็น open collector ส่วน GPIO2/15 เป็น strapping pins ของ ESP32 ห้ามให้
วงจร encoderบังคับระดับผิดขณะเปิดเครื่อง

ค่าที่ต้องวัดและแก้ให้ตรงกับฮาร์ดแวร์:

- `DEFAULT_ODOM_WHEEL1_RADIUS_M` ถึง `DEFAULT_ODOM_WHEEL3_RADIUS_M`:
  รัศมีการกลิ้งจริงของล้อแต่ละล้อ หน่วยเมตร
- `DEFAULT_ENCODER_TICKS_PER_REV`: จำนวน count ต่อรอบหลังนับ quadrature x4 และหลังเกียร์
- `ENCODERx_DIRECTION`: เปลี่ยนเป็น `-1` หาก count มีเครื่องหมายกลับด้าน
- `DEFAULT_WHEEL_PID_KP/KI/KD`: gain PID ความเร็วล้อ
- `ROBOT_RADIUS`: ระยะจากศูนย์กลางฐานถึงล้อ

ROS 2 interfaces สำหรับ Nav2:

- Subscribe `/cmd_vel` (`geometry_msgs/msg/Twist`)
- Publish `/odom` (`nav_msgs/msg/Odometry`) เฟรม `odom` -> `base_link`
- Publish `/tf` (`tf2_msgs/msg/TFMessage`) transform `odom` -> `base_link`
- Publish `/imu/data` (`sensor_msgs/msg/Imu`)
- Subscribe `/reset_odom` (`std_msgs/msg/Bool`)
- Debug `/debug/wheel_ticks` และ `/debug/wheel_vel_mps`

ตรวจสอบก่อนเริ่ม Nav2:

```bash
ros2 topic hz /odom
ros2 topic echo /debug/wheel_ticks
ros2 run tf2_ros tf2_echo odom base_link
ros2 topic info /tf --verbose
```

ต้องมี static transform `base_link` -> `imu_link` ใน URDF หรือ
`static_transform_publisher` และให้ Nav2 controller ส่งคำสั่งมายัง `/cmd_vel`
หากระบบใช้ `/cmd_vel_nav` หรือ `/cmd_vel_smoothed` ให้ remap มาที่ `/cmd_vel`

### อ่านและหารัศมีล้อแต่ละล้อ

เมื่อเปิด Serial Monitor ที่ `115200 baud` ฟังก์ชัน `printOdomWheelRadii()` จะพิมพ์
ค่าทั้งสามล้อตอนเปิดเครื่อง ตัวอย่าง:

```text
Odometry wheel radius (effective rolling radius):
  wheel 1: 0.05000 m (50.00 mm)
  wheel 2: 0.04970 m (49.70 mm)
  wheel 3: 0.05030 m (50.30 mm)
```

ในโค้ดสามารถอ่านทีละล้อได้ด้วย `getOdomWheelRadiusM(1)` ถึง
`getOdomWheelRadiusM(3)` หากหมายเลขไม่ใช่ 1–3 ฟังก์ชันจะคืน `NAN`

วิธีวัดแนะนำคือทำเครื่องหมายบนยาง หมุนล้อด้วยมือ 10 รอบโดยให้ล้อรับน้ำหนักใกล้เคียง
ตอนใช้งาน วัดระยะที่ผิวล้อเคลื่อนที่ แล้วคำนวณ:

```text
radius_m = distance_m / (2 × PI × number_of_revolutions)
```

ตัวอย่าง ระยะ 10 รอบเท่ากับ 3.135 เมตร จะได้
`3.135 / (2 × PI × 10) = 0.04990 m` จากนั้นใส่ค่าใน constant ของล้อนั้น
และอัปโหลดใหม่ ควรวัดแต่ละล้ออย่างน้อยสามครั้งแล้วใช้ค่าเฉลี่ย

ค่ารัศมีจะพิมพ์ใน Serial Monitor ตอนเปิดเครื่อง สำหรับไลบรารี precompiled ชุดนี้
interface ตั้งค่าเสริมและ parameter server ถูกปิดไว้ เพื่อสงวน micro-ROS entity ให้
`/cmd_vel`, `/odom` และ `/tf` หากต้องเปิดใช้งาน ต้อง rebuild ไลบรารีโดยเพิ่มค่า
`RMW_UXRCE_MAX_SUBSCRIPTIONS` ก่อน

เฟิร์มแวร์ ESP32 รับ `geometry_msgs/msg/Twist` จาก topic `/cmd_vel` ผ่าน
micro-ROS UDP และแปลงเป็น PWM สำหรับล้อ omni 3 ล้อที่วางทำมุม 120 องศา

## ตั้งค่าก่อนอัปโหลด

แก้ค่าต่อไปนี้ใน `omni_joy_microROS.ino`:

- `WIFI_SSID` และ `WIFI_PASSWORD`
- `AGENT_IP`: IPv4 ของเครื่อง ROS 2 host ในวง LAN เดียวกัน ห้ามใช้ `127.0.0.1`
- `AGENT_PORT`: ค่าเริ่มต้นคือ UDP 8888
- `MOTOR1_DIRECTION` ถึง `MOTOR3_DIRECTION`: เปลี่ยนเป็น `-1` เฉพาะมอเตอร์ที่หมุนกลับด้าน
- `ROBOT_RADIUS`: ระยะจากจุดศูนย์กลางหุ่นยนต์ถึงจุดสัมผัสล้อ หน่วยเมตร
- `MAX_WHEEL_LINEAR_SPEED`: ความเร็วแนวสัมผัสของล้อที่ PWM 100% หน่วย m/s
- pin PWM/DIR ให้ตรงกับวงจรขับมอเตอร์จริง

เลือกบอร์ด ESP32 และติดตั้งไลบรารี `micro_ros_arduino` ที่ตรงกับ ROS 2
distro/ESP32 Arduino core ที่ใช้ จากนั้นจึง compile และ upload

## เริ่ม micro-ROS agent บน ROS 2 host

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

อนุญาต inbound UDP port 8888 ใน firewall ของ host แล้วตรวจว่า node เชื่อมต่อ:

```bash
ros2 node list
ros2 topic info /cmd_vel
```

หาก `ros2 topic list` เห็น topic บ้างไม่เห็นบ้าง ให้ตรวจตามลำดับ:

```bash
# Terminal 1: ดูว่า ESP32 สร้าง/ทำลาย session ซ้ำหรือไม่
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6

# Terminal 2: รีเฟรช ROS graph cache
ros2 daemon stop
ros2 daemon start
ros2 node list
ros2 topic list
```

เครื่องที่รัน agent และ terminal ต้องใช้ `ROS_DOMAIN_ID` เดียวกัน และ firewall ต้อง
อนุญาต UDP 8888 ตรวจ `AGENT_IP` ว่ายังเป็น IP ของ ROS host ปัจจุบัน แนะนำกำหนด
DHCP reservation ให้ ROS host และเปิด Serial Monitor เพื่อดูข้อความ
`micro-ROS agent disconnected` หากข้อความนี้เกิดซ้ำ ให้ตรวจสัญญาณ Wi-Fi,
แหล่งจ่าย ESP32 และสัญญาณรบกวนจากมอเตอร์

## ควบคุมด้วยคีย์บอร์ด

ติดตั้งและเริ่ม teleop ในอีก terminal:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  --remap cmd_vel:=/cmd_vel
```

ทดสอบโดยยกล้อให้พ้นพื้นก่อน หากปล่อยปุ่มหรือ stream `/cmd_vel` ขาดเกิน
400 ms เฟิร์มแวร์จะสั่ง PWM เป็นศูนย์ และจะหยุดเช่นกันเมื่อ agent หลุด

แกนอ้างอิงตาม ROS REP-103: `linear.x` เดินหน้า, `linear.y` ไปทางซ้าย และ
`angular.z` หมุนทวนเข็มนาฬิกา หากทิศจริงไม่ตรง ให้ตรวจลำดับล้อและค่า
`MOTORx_DIRECTION` ก่อนแก้สมการคิเนเมติกส์

## ทดสอบทีละแกน

```bash
# เดินหน้า 0.2 m/s
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"

# เลื่อนไปทางซ้าย 0.2 m/s
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {y: 0.2}, angular: {z: 0.0}}"

# หมุนทวนเข็ม 0.5 rad/s
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}"
```

กด `Ctrl-C` หลังทดสอบแต่ละคำสั่ง ระบบ timeout จะหยุดมอเตอร์อัตโนมัติ
