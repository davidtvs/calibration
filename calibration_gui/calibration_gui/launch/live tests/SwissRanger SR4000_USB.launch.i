<?xml version="1.0"?>
<launch>
    <arg name="host" default="192.168.1.42"/>
    <arg name="node_name" default="SwissRanger"/>
    <arg name="ball_diameter" default="0.99"/>

    <remap from="/$(arg node_name)/$(arg node_name)/pointcloud_raw" to="/$(arg node_name)/pointcloud_raw"/>

    <group ns="$(arg node_name)">
    <node pkg="swissranger_camera" type="swissranger_camera" name="$(arg node_name)" required="true" >
        <!--param name="auto_exposure" value="1"/-->
        <!--param name="integration_time" value="40" /-->
        <!-- Set auto_exposure or integration_time, not both -->

        <param name="camera_name" value="$(arg node_name)"/>
    </node>

    <node name="BD_$(arg node_name)" pkg="calibration_gui" type="swissranger" required="true" output="screen">
        <param name="ballDiameter" type="double" value="$(arg ball_diameter)"/>
    </node>
  </group>

</launch>
