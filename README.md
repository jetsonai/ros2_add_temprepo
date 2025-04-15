# ros2_add_temprepo


# 패키지 삭제

새 터미널을 열고 DTA

cdsensor

cd src

rm -rf sensor_test_pack

# 추가되는 파일 다운로드 받기

새 터미널을 열고

cd Downloads/ros2_add_temprepo

git pull

cp -rf ros2_add_temprepo/sensor_test_pack ~/ros2_sensor_ws/src

# 패키지 교체 및 빌드

패키지 삭제했던 터미널로 돌아가서

cdsensor

colcon build --packages-select sensor_test_pack

# lidar_sub_test 실행

새 터미널을 열고 DTA

sensorws

ros2 launch hls_lfcd_lds_driver hlds_laser_rviz2.launch.py

새 터미널을 열고 DTA

sensorws

ros2 run sensor_test_pack lidar_sub_test



