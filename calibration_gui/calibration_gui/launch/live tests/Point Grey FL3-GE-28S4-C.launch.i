<?xml version="1.0"?>
<launch>
  <arg name="node_name" default="lms151_1"/>
  <arg name="ball_diameter" default="0.99"/>

  <group ns="$(arg node_name)">
    <node name="$(arg node_name)" pkg="pointgrey_fl3_ge_28s4_c" type="pointgrey_FL3_28S4" required="true" output="screen"></node>

    <node name="BD_$(arg node_name)" pkg="calibration_gui" type="point_grey_camera" required="true" output="screen">
      <param name="ballDiameter" type="double" value="$(arg ball_diameter)"/>
    </node>
  </group>
</launch>
