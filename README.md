# ROS_PathPlanning_pkg
Pathplanning on specified vertices

## hdl localization 기반 셋업

### 동작 환경

- ROS Melodic
- ROS Noetic

### 패키지 종속성

### 구성

[Pure pursuit](https://github.com/DCUSnSLab/SCV_Pure_pursuit/tree/master/src)

아래 링크에서 현재 사용중인 ROS 버전에 맞는 hdl localization 설치 및 동작 확인

[hdl_localization](https://github.com/koide3/hdl_localization)

위 저장소 외에도 차량의 현재 측위를 추정할 수 있는 패키지 사용 가능

hdl localization의 hdl_localization.launch 파일의 마지막 부분을 아래와 같이 수정

![image](https://user-images.githubusercontent.com/40351426/167768765-bb9f5361-6eda-45ba-9336-10d662cef7f1.png)

<node pkg="path_planning" type="dijkstra_path.py" name="path_planning" output="screen" >
    <param name="json_file" value="$(find path_planning)/data/20211104_map.json" />
</node>

이후 동일한 워크스페이스의 src 경로에서 PathPlanning 패키지를 clone

### Path Creator, Visualizer

create_vertex.launch
- rviz의 2D Nav Goal 을 사용해서 임의의 경로를 생성한다

visualize_path.launch
- 임의의 .pcd 지도 파일과 경로 파일 path.json 을 바탕으로 지도 내에 생성된 경로를 시각화한다.

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

  - /odom 토픽을 통한 현재 위치, Rviz에서 "2D Pose Estimate" 를 통해 목적지를 지정하고, 사전 정의된 정점을 바탕으로 경로 생성 수행
