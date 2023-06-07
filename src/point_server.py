import cv2
import json
import base64
import rospy
import websocket
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
websocket_url = "ws://<remote-app-ip>:<port>/rosbridge"
topic_name = "/camera/image_raw"

# 创建 WebSocket 连接
ws = websocket.create_connection(websocket_url)

# 定义 ROS 订阅回调函数
def image_callback(msg):
    # 将 ROS 中的图像数据转换为 OpenCV 格式
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

    # 将 OpenCV 格式的图像数据转换为 base64 编码字符串
    _, img_encoded = cv2.imencode('.png', cv_image)
    img_base64 = base64.b64encode(img_encoded).decode('utf-8')

    # 构造要发送的 JSON 消息
    json_data = {
        "op": "publish",
        "topic": topic_name,
        "msg": {
            "data": img_base64,
            "height": msg.height,
            "width": msg.width,
            "encoding": "bgr8",
            "is_bigendian": 0,
            "step": msg.step
        }
    }

    # 将 JSON 消息发送到 WebSocket
    ws.send(json.dumps(json_data))

# 创建 ROS 节点和订阅者
rospy.init_node("image_publisher")
rospy.Subscriber(topic_name, Image, image_callback)

# 保持程序运行，直到收到中断信号
rospy.spin()

# 关闭 WebSocket 连接
ws.close()

