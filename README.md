**실행 확인**

1. 기본 제어 시작
`roslaunch turn_on_mercury_robot turn_on_mercury_robot.launch`

3. 배터리 잔량 확인 (21이상)
`rostopic echo /PowerVoltage`

4. X1 제어 확인 (몸통 제어)
`roslaunch mercury_x1_teleop keyboard_teleop.launch` 

5. X1 Arm 제어 확인
`python angle.py`

    

**연결방법**

S570 WiFi 모드로 변경 후 X1에서 elephant 찾아서 wifi 연결 (비밀번호 : elephant)
X1 내 OS에서 코드 작성

 

**ls /dev/tty* 입력해서 시리얼 포트 확인**

S570 : ttyACM3 (확인 필요) - 유선일 경우

Left Arm : ttyTHS0

Right Arm : ttyACM1



**sensor value error로 코드가 작동하지 않을 경우** 

[release.py 코드를 사용해 수동으로 팔 영점 조절 ]

[focus.py로 팔 잠금 및 영점 위치 갱신 ]

[영점 조절 완료 후 angle.py로 작동 확인]



**실행 방법 요약**

1. X1, S570 전원 켜기 
2. S570 첫번째 버튼 눌러서 WiFi 모드로 변경 
3. X1에서 WiFi 킨 후 elephant에 연결 (비밀번호 : elephant)
4. X1에서 터미널 2개 오픈
5. (터미널 1) `roslaunch turn_on_mercury_robot turn_on_mercury_robot.launch`  (바탕화면 터미널 아이콘 클릭)
6. S570 장착 및 팔 정렬  - sensor value error 때문에 미리 장착해야함
7. (터미널 2) python start.py 입력 (바탕화면 START 클릭)
8. 코드 실행 후 X1의 팔이 영점을 맞출 때, S570의 조이스틱을 움직이지 마세요. 
9. Q/ESC : 코드 일시정지 , G : 코드 재개
