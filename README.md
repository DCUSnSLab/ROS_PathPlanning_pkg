# ROS_PathPlanning_pkg
Pathplanning on specified vertices

### hdl Graph-SLAM 기반 셋업

아래 링크에서 현재 사용중인 ROS 버전에 맞는 hdl localization 설치 및 동작 확인

https://github.com/koide3/hdl_localization

이후 동일한 워크스페이스의 src 경로에서 PathPlanning 패키지를 clone

hdl_localization을 통해 차량의 현재 위치를 확인할 수 있는 상황에서 아래와 같이 goal 지점을 지정

### Topics

## Subscribe

- **/odom**

## Publish

- **/defined_vertices**

- **/refined_vertices**


