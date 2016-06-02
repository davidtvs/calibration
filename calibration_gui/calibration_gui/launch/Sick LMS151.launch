<?xml version="1.0"?>
<launch>
    <arg name="host" default="192.168.0.134"/>
    <arg name="node_name" default="lms151_1"/>
    <arg name="ball_diameter" default="0.99"/>

    <group ns="$(arg node_name)">
        <node name="$(arg node_name)" pkg="lms1xx" type="lms1xx" required="true" output="screen">
            <param name="host" value="$(arg host)"/>
            <param name="frame_id" value="/$(arg node_name)"/>
        </node>
        
        <node name="BD_$(arg node_name)" pkg="calibration_gui" type="sick_lms151" required="true" output="screen">
            <param name="ballDiameter" type="double" value="$(arg ball_diameter)"/>
        </node>
    </group>
</launch>
