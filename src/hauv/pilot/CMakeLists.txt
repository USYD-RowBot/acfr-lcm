set (SRC
  pilot_gui/PilotCoverageWaypoints4.java
  pilot_gui/PilotDVL.java
  pilot_gui/PilotGHP.java
  pilot_gui/PilotGUIRectangle.java
  pilot_gui/PilotGUISlicesTampa.java
  pilot_gui/PilotVerticalAltitude.java
  pilot_gui/PilotVerticalDepth.java
  pilot_gui/ReceiveCNV.java
  pilot_gui/ReceiveCommand.java
  pilot_gui/ReceiveNVG.java
  pilot_gui/ReceivePIT.java
  pilot_gui/ReceivePLAN.java
  pilot_gui/ReceiveRNV.java
  pilot_gui/SendCommand.java
)

perls_java_add_library (hauv_pilot.jar ${SRC})
perls_java_target_link_libraries (hauv_pilot.jar perls_lcmtypes.jar)
add_dependencies (hauv_pilot.jar perls_lcmtypes.jar)

perls_bash_install_script (launch-pilot-gui launch-pilot-gui.in)
