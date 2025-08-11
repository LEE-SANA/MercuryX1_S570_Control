**MercuryX1_S570_Control**
Mercury X1 휴머노이드 로봇을 S570 외골격(Exoskeleton) 컨트롤러로 제어하기 위한 Python 기반 예제 코드 모음입니다.

**주요 기능**

외골격 S570에서 실시간 모션 데이터를 받아 Mercury X1 로봇 양쪽 팔을 동기 제어
USB 또는 소켓을 통한 통신 방식 지원
조인트 제어, 그리퍼(gripper) 작동, 컬러 표시 등을 통한 직관적 피드백 제공

**필요 조건**
- Python 3.x
- pymycobot 라이브러리 (pip 설치 가능)
- S570 (USB 또는 Bluetooth/Wi-Fi 연결)
- Mercury X1

**셋업 및 실행 순서**
- pip install pymycobot
- X1과 S570을 연결 (S570 첫번째 버튼. ID/PASSWORD : elephant)
- X1 영점 조절, S570 착용
- start_end.py 실행
- S570을 움직이며 X1 반응 확인

