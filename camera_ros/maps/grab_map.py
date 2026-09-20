#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2 # OpenCV 사용 (대부분의 ROS 환경에 깔려있습니다)

def map_callback(msg):
    # 지도 데이터를 numpy 배열로 변환
    map_array = np.array(msg.data).reshape((msg.info.height, msg.info.width))
    
    # 이미지 생성 (0: 미지, 100: 벽, -1: 빈공간) -> (255: 빈공간, 0: 벽, 127: 미지)
    img = np.zeros((msg.info.height, msg.info.width), dtype=np.uint8)
    img[map_array == 0] = 255   # 갈 수 있는 곳 (흰색)
    img[map_array == 100] = 0   # 벽 (검은색)
    img[map_array == -1] = 127  # 모르는 곳 (회색)

    # 지도가 거꾸로 저장되는 것 방지
    img = np.flipud(img)
    
    # 이미지 저장
    cv2.imwrite('my_map_result.png', img)
    rospy.loginfo("지도 저장 완료! corridor_result.png 파일을 확인하세요.")
    rospy.signal_shutdown("Success")

if __name__ == '__main__':
    rospy.init_node('map_grabber')
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.spin()
