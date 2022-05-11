# ROS_PathPlanning_pkg
Pathplanning on specified vertices

## hdl Graph-SLAM 기반 셋업

### 테스트트 환경

 - Ubuntu 18.04
    - ROS Melodic Morenia

아래 링크에서 현재 사용중인 ROS 버전에 맞는 hdl localization 설치 및 동작 확인

https://github.com/koide3/hdl_localization

hdl localization의 hdl_localization.launch 파일의 마지막 부분을 아래와 같이 수정

![image](https://user-images.githubusercontent.com/40351426/167768765-bb9f5361-6eda-45ba-9336-10d662cef7f1.png)

<node pkg="path_planning" type="dijkstra_path.py" name="path_planning" output="screen" >
    <param name="json_file" value="$(find path_planning)/data/20211104_map.json" />
</node>

이후 동일한 워크스페이스의 src 경로에서 PathPlanning 패키지를 clone

## Localization

원본 repository 주소의 아래 내용을 따라 Localization 수행

![image](https://user-images.githubusercontent.com/40351426/167769126-3ed58c32-4261-497c-9b17-87cbf1879e68.png)

hdl_localization을 통해 차량의 현재 위치를 확인할 수 있는 상태에서 goal 지점을 지정 2D Nav Goal로 지정

![image](https://user-images.githubusercontent.com/40351426/167769344-62c46cbc-41b9-4a3f-b33b-dbd001deabcb.png)

지정한 목표 지점까지의 경로가 아래 그림처럼 초록색 네모로 출력됨

![image](https://user-images.githubusercontent.com/40351426/167769458-a1f9d96e-ea90-4395-8085-487d1fa8f2fa.png)

## Topics

### Subscribe

- **/odom** (nav_msgs/Odometry)

  - 지도 데이터 내 차량의 현재 위치

### Publish

- **/defined_vertices** (visualization_msgs/MarkerArray)

  - JSON 파일 내 정의된 정점 목록

- **/refined_vertices** (visualization_msgs/MarkerArray)

  - /odom 토픽을 통한 현재 위치, Rviz에서 "2D Pose Estimate" 를 통해 지정한 목적지, 사전 정의된 정점을 바탕으로 생성한 경로로

