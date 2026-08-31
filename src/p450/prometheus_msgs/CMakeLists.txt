cmake_minimum_required(VERSION 2.8.3)
project(prometheus_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  geometry_msgs
  message_generation
  sensor_msgs
  std_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  UAVState.msg
  MultiUAVState.msg
  UAVCommand.msg
  UAVControlState.msg
  UAVSetup.msg
  TextInfo.msg
  GlobalAruco.msg
  UGVState.msg
  UGVCommand.msg
  MultiUGVState.msg
  PositionReference.msg
  StationCommand.msg

  ArucoInfo.msg
  MultiArucoInfo.msg
  DetectionInfo.msg
  MultiDetectionInfo.msg
  BoundingBox.msg
  BoundingBoxes.msg

  SwarmCommand.msg
  FormationAssign.msg
  OffsetPose.msg
  GPSData.msg

  LinktrackNode2.msg
  LinktrackNodeframe2.msg

  DetectionInfoSub.msg
  GimbalControl.msg
  GimbalState.msg
  MultiDetectionInfoSub.msg
  VisionDiff.msg
  WindowPosition.msg
  Bspline.msg
  MultiBsplines.msg
  Control.msg
  BasicDataTypeAndValue.msg
  CustomDataSegment.msg
  ParamSettings.msg

  ROI.msg
  Target.msg
  TargetsInFrame.msg
  StartScript.msg

  LivoxCustomMsg.msg
  LivoxCustomPoint.msg

  DepthCompressed.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  SwitchLocationSource.srv
  GetLocationSource.srv
  GimbalFollowTrackResSevice.srv
)

add_action_files(
  DIRECTORY action
  FILES
  CheckForObjects.action
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  actionlib_msgs
  geometry_msgs
  message_runtime
  sensor_msgs
  std_msgs
)
