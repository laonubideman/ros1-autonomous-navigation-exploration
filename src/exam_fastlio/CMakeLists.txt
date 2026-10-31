# catkin 构建与安装规则；将主办方 Python 节点和配置资源加入功能包。
cmake_minimum_required(VERSION 3.0.2)
project(exam_fastlio)
find_package(catkin REQUIRED)
catkin_package()
catkin_install_python(PROGRAMS
  scripts/model_for_fastlio.py
  scripts/fastlio_nav_bridge.py
  scripts/fastlio_grid.py
  scripts/frontier_explorer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES scripts/room_tasks.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES LICENSE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
