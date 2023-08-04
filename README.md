# 2023 앙치킨 ros2 workspace

## gazebo_models
track_test를 구동시킬 때 필요한 models 파일.
.gazebo/models에 2개의 폴더를 옮긴 후 gazebo 실행 필요.

## track_test
자율 주행 코드 test를 위한 pkg
(s자 모양 생성) (track_s_world.launch.py,track_s_test.world) 여튼 s 붙어있는거

## lane_detection_py
방향 인식, 정지선 검출 시 = 0~4초 멈춤 - 4~8초 방향 인식 가능 - 8초~ 방향,정지선 인식 가능

## text_detection
knn은 되는데 개느림. cnn해보는중. 색으로 해야할거같음. 

## move_py
하다가 만거
