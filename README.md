# Capstone1 2021 Spring

KAIST 2021 Spring Capstone Design 1

This is a repository for capstone design 1 codes. All the codes are just for your reference, so change the code as necessary (e.g. topic name, ball size and color).

To download the package, write the command below in ~/catkin_ws/src terminal.
```console
git clone https://github.com/yominx/Capstone1_2021Spring.git
```


## ball_detection

Node for detecting red balls.

It publishes the information of ball position and color by calculating geometry between the robot and balls.

#### Usage

```console
rosrun ball_detection ball_detection_node
```



## coppeliasim_models

map_ver_x.ttt, bonus_map.ttt, my_robot_ver_x.ttm files

Map and my_robot can be updated later, so please check the notice board on klms regularly.

Just drag and drop files to coppeliasim window.

For reference, urdf file for my_robot is uploaded.

1. mylidar_hokuyo.ttm

- ROS

frame : base_scan

topic : /scan

msg type : sensor_msgs::LaserScan

- Lidar specs

rate : 5Hz

resolution : 1 deg -> 360 points per 1 scan

min range : 0.12 m

max range : 3.5 m


2. myimu.ttm

- ROS

frame : imu

topic : /imu

msg type : sensor_msgs::Imu


3. mykinect.ttm

- Camera parameters

FOV (field of view) : 57 deg

W : 640 pixels

H : 480 pixels

f : 589.37 pixels


## core_msgs

A package for defining custom messages used in all codes.

ex) ball_position.msg



## data_integrate

Subscribing lidar(laser_scan) and camera data and do something

You need to change data type of lidar, topic name of both sensors, publishing cmd_vel type to use the node correctly.

ex) /scan -> /laser_scan, sensor_msgs::laser_scan -> sensor_msgs::PointCloud

#### Usage

```console
# data integrate
rosrun data_integrate data_integrate_node
# data show
rosrun data_integrate data_show_node
```



## robot_teleop

Nodes for manually manipulating the gripper and robot wheels.

#### Usage

```console
# gripper
rosrun robot_teleop prismatic_teleop_key
# wheel
rosrun robot_teleop wheel_teleop_key
```



## Tips

- If the prismatic joint fall down, check "Lock motor when target velocity is zero" in Joint Dynamic Properties in CoppeliaSim.
- Click "Toggle real-time mode" when you test your algorithms in simulator.
- ...



## Troubleshooting

- TBA

  

## Contact

Jonghwi Kim <stkimjh@kaist.ac.kr>

Haggi Do <kevindo@kaist.ac.kr>

Kyungseo Kim <chalseokim@kaist.ac.kr>


---

# Git 커맨드 사용

## Git 설치하기

https://git-scm.com/book/ko/v1/%EC%8B%9C%EC%9E%91%ED%95%98%EA%B8%B0-Git-%EC%84%A4%EC%B9%98

## 파일 복사하기
터미널에 다음과 같은 커맨드를 입력합니다.
```
git clone https://github.com/yominx/Capstone1_2021Spring
```
유저네임과 패스워드를 입력하면 커맨드를 실행한 폴더에 **Capstone1_2021spring**폴더가 생겨있는 것을 볼 수 있습니다.

## 파일 최신 버전으로 업데이트하기
다른 사람이 github에 새파일을 업로드하거나 내용을 수정할 수 있습니다.
이를 '커밋'이라고합니다. 새로운 커밋이 등록되면 내 디스크에 있는 파일도 업데이트 해주어야합니다.
다음과 같은 커맨드를 Capstone1_2021spring 폴더에서 실행합니다.
```
git pull
```
그러면 뭐라뭐라 나온 뒤 업데이트된 정보를 모두 받아옵니다.

## 파일 수정한 것 올리기
나도 커밋을 해서 깃헙에 올려서 자료를 공유해야합니다.
그러려면 세가지 과정이 필요합니다.
Stage에 올리기 - 커밋하기 - 업로드하기

Stage에 올린다는것은 임시저장입니다. 다음과 같은 커맨드를 입력하면 수정된 모든 사항을 Stage에 올릴 수 있습니다.

```
git add .
```

커밋한다는 것은 새로운 버전으로 등록한다는 의미입니다. 다음과 같은 커맨드를 통해 커밋할 수 있습니다.

```
git commit -m "<하고 싶은 말>"
```

예를들어, 이 버전의 참고사항을 "Updated schedule"으로 하고싶다면,

```
git commit -m "Updated schedule"
```

과 같이 써주시면 됩니다. 

업로드는 내 컴퓨터에서 만든 버전을 깃헙에 올려 공유하는 것입니다. 다음과 같은 커맨드로 실행할 수 있습니다.

```
git push
```
혹은, 
```
git push origin master
```

이 push를 해주지 않으면 혼자만 저장하고 github에는 올라가지 않습니다.

만약에 실패했다고 나온다면, 두가지 정도 이유가 있을 수 있습니다.

1.최신버전이 아닌 경우 
git pull 어쩌구라고 메시지가 나온다면, 현재 여러분이 최신버전을 가지고 있는 것이 아니기 때문에 먼저 최신버전 파일을 내려받은 뒤 업데이트 하라는 의미입니다.
git pull 커맨드를 먼저 입력해준 뒤 다시 시도해보세요.

2.유저정보를 등록하지 않은 경우
```
git config --global user.name ""
git config --global user.email ""
```

이러한 식으로 경고가 출력된다면, 메시지 속 큰따옴표 안에 자신의 정보를 각각 입력해준뒤 ``` git push```를 다시 실행해주세요.

### Git 커맨드를 쓰면 얻는 장점

매번 업데이트된 정보를 다 다운 받을 필요 없이 수정된 정보만 다운받을 수 있기 때문에 관리가 편리합니다.
