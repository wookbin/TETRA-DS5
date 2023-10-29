# TETRA-DS5
TETRA-DS5 _ ROS1-melodic
# 현장 적용 TETRA-DS5 ROS Package. (가제보 X)
# 종족성 패키지들이 많이 있습니다...

![2ND 448](https://github.com/wookbin/TETRA-DS5/assets/58063370/ec097f8e-8ed6-4cc5-a851-9ff7c611c660)

- tetraDS : TETRA구동 보드와의 Serial통신을 통하여 Motor의 구동을 담당하는 패키지
- tetraDS_description: TETRA-DS5의 URDF가 포함된 패키지.
- tetraDS_interface: TETRA의 전원/초음파센서/GPIO 제어를 제공하는 보드 사용 패키지(Serial통신 사용)
- tetraDS_landmark: 인식되는 ar_tag를 이용하여 마커를 저장/표시하기 위한 패키지 
- tetraDS_TCP: 외부 디바이스와 Socket통신을 위하여 제작한 패키지(프로토콜은 소스코드 내용참고)
- tetraDS_service: TETRA-DS5의 환경지도/자율주행/각종 기능들에 서비스를 사용하기 위한 패키지
- tetraDS_2dnav: TETRA-DS5의 각종 설정 파일과 런치 파일들을 모아둔 패키지

- virtual_costmap_layer: GMahmoud가 만든 가상 레이어 패키지 (Global costmap에 활용)
- virtual_costmap_layer2: GMahmoud가 만든 가상 레이어 패키지에서 플러그인 이름만 수정한 버전 (local costmap에 활용)

