#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import pandas as pd
import os

def publish_from_csv():
    # 상대경로 기준 데이터 경로 설정
    base_path = os.path.dirname(__file__) + "/../data/"
    gps_csv = os.path.abspath(base_path + "gps_cleaned.csv")
    zed_csv = os.path.abspath(base_path + "zed_cleaned.csv")

    # CSV 파일 불러오기
    gps_df = pd.read_csv(gps_csv)
    zed_df = pd.read_csv(zed_csv)

    print("✅ CSV 파일 불러오기 성공!")
    print(f"GPS 데이터 {len(gps_df)}개, ZED 데이터 {len(zed_df)}개")

    # ROS 노드 초기화
    rospy.init_node("csv_replayer")
    pub_gps = rospy.Publisher("/gps/odom", Odometry, queue_size=10)
    pub_zed = rospy.Publisher("/zed/odom", Odometry, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    # 시간 기준 설정
    start_time = rospy.Time.now()

    for i in range(min(len(gps_df), len(zed_df))):
        t = start_time + rospy.Duration.from_sec(i * 0.1)  # 10Hz 간격 시간 증가

        # GPS 메시지 생성
        msg_gps = Odometry()
        msg_gps.header.stamp = t
        msg_gps.header.frame_id = "odom"
        msg_gps.pose.pose.position.x = gps_df['fused_x'][i]
        msg_gps.pose.pose.position.y = gps_df['fused_y'][i]

        # ZED 메시지 생성
        msg_zed = Odometry()
        msg_zed.header.stamp = t
        msg_zed.header.frame_id = "odom"
        msg_zed.pose.pose.position.x = zed_df['fused_x'][i]
        msg_zed.pose.pose.position.y = zed_df['fused_y'][i]

        # 퍼블리시
        pub_gps.publish(msg_gps)
        pub_zed.publish(msg_zed)

        if i % 10 == 0:
            rospy.loginfo(f"[{i}] GPS: ({msg_gps.pose.pose.position.x:.2f}, {msg_gps.pose.pose.position.y:.2f})  "
                          f"ZED: ({msg_zed.pose.pose.position.x:.2f}, {msg_zed.pose.pose.position.y:.2f})")

        rate.sleep()

if __name__ == "__main__":
    try:
        publish_from_csv()
    except rospy.ROSInterruptException:
        pass

