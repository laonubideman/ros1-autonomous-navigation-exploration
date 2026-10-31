# catkin 构建与安装规则；将主办方 Python 节点和配置资源加入功能包。
cmake_minimum_required(VERSION 3.0.2)
project(exam_bringup)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/model_for_sim.py
  scripts/scene_visuals.py
  scripts/velocity_guard.py
  scripts/obstacle_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config maps worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES LICENSE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
