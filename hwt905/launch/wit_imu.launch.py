
<!--imu ros -->
<launch>
	<!--Imu model, default normal
		If the device type is modbus protocol, fill in modbus
		If the device type is wit standard protocol, fill in normal
		If the device type is MODBUS high-precision protocol, fill in hmodbus
		If the device type is CAN protocol, fill in can
		If the device type is CAN high-precision protocol, fill in hcan
		The device number/dev/ttyUSB0 (the default script uses/dev/ttyUSB0) is the number recognized by your computer
		The baud rate is set according to actual usage. The default baud rate for JY6x series modules is 115200, CAN modules are 230400, and other modules are 9600
		If the user modifies the baud rate through the upper computer, it needs to be correspondingly modified to the modified baud rate 
	-->
	
    <arg name="type" default="normal" doc="type [normal, modbus, hmodbus, can, hcan]"/>

    <!-- imu 对应 python 文件 -->
    <node pkg="wit_ros_imu" type="wit_$(arg type)_ros.py" name="imu" output="screen">
        <param name="port"               type = "str"    value="/dev/ttyUSB0"/>
        <param name="baud"               type = "int"    value="9600"/>
    </node>

	<!--
    <node pkg="wit_ros_imu" type="get_imu_rpy.py" name="get_imu" output="screen">
    </node>
	-->
</launch>

