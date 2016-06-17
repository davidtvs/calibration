<?xml version="1.0"?>
<launch>
    <arg name="host" default="192.168.100.244"/>
    <arg name="port" default="12002"/>
    <arg name="node_name" default="ldmrs_1"/>
    <arg name="ball_diameter" default="0.99"/>

    <group ns="$(arg node_name)">
        <!-- <node pkg="sick_ldmrs" type="sickldmrs.py" name="$(arg node_name)" required="true" output="screen">
            <param name="host" value="$(arg host)"/>
            <param name="port" value="$(arg port)"/>
        </node> -->

        <node name="BD_$(arg node_name)" pkg="calibration_gui" type="sick_ldmrs" required="true" output="screen">
            <param name="ballDiameter" type="double" value="$(arg ball_diameter)"/>
        </node>
    </group>
</launch>
