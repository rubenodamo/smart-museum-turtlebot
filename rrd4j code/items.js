// connects to the items we created in OpenHab

Contact Team_3_Door_Sensor_Sensor_Door "Door Sensor" {channel="zwave:device:9d84233791:node2:sensor_door"}
Number Team_3_Button_Scene_Number "Button Scene Number"{channel="zwave:device:9d84233791:node13:scene_number"}

// we created a new item in OpenHab so that we could manipulate the persisted data into values we could display in HABPanel
String LogMessage "Log Message" {channel="zwave:device:9d84233791:node2:sensor_door"}

