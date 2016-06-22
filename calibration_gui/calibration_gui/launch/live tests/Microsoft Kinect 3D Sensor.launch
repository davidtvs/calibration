<?xml version="1.0"?>
<launch>
    <arg name="node_name" default="kinect"/>
    <arg name="ball_diameter" default="0.99"/>

    <group ns="$(arg node_name)">
        <include file="$(find openni_launch)/launch/openni.launch">
            <arg name="depth_registration" value="true"/>
        </include>

        <node name="BD_$(arg node_name)" pkg="calibration_gui" type="kinect" required="true" output="screen">
            <param name="ballDiameter" type="double" value="$(arg ball_diameter)"/>
        </node>
    </group>
</launch>
