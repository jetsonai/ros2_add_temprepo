# ros2_add_temprepo

새 터미널을 열고

cd Downloads

https://github.com/jetsonai/ros2_add_temprepo

cp -rf ros2_add_temprepo/sensor_test_pack ~/ros2_sensor_ws/src

새 터미널을 열고

DTA

cdsensor

colcon build --packages-select sensor_test_pack

sensor_test_pack.lidar_sub_node

# 추가되는 파일 다운로드 받기

새 터미널을 열고

cd Downloads/ros2_add_temprepo

git pull

cp ./src/lidar_sub_test.py ~/ros2_sensor_ws/src/sensor_test_pack/src

sensor_test_pack 의 setup.py 에 lidar_sub_test 를 추가해주세요.


