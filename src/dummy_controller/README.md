dummy_controller包功能概述
用于基于交通灯识别结果做出驾驶决策，并将决策以字符串形式发布到话题 /traffic_decision 上，供其他模块使用。
输入话题：
    /traffic_light_color（std_msgs::String）
    消息格式："id:state:confidence"
    示例："1:RED:0.92"
    其中 state 为交通灯的颜色，取值为 "RED"、"YELLOW"、"GREEN"。

输出话题：
    /traffic_decision（std_msgs::String）
    发布的决策结果之一：

        "BRAKE"：红灯，车辆应刹车

        "ACCELERATE"：黄灯，车辆应快速通过

        "DRIVE"：绿灯，车辆正常行驶

        "UNKNOWN"：未能识别交通灯状态

控制模块使用：
订阅/traffic_decision话题
话题内容为:
	"BRAKE" 表示检测到红灯
	“ACCELERATE" 表示检测到黄灯
	"DRIVE" 表示检测到绿灯
	"UNKNOWN" 表示为识别到交通灯
控制模块可根据话题的内容作出不同的速度控制策略，例如话题内容为“BRAKE”表示检测到红灯，可发布停止的速度；
话题内容为“ACCELERATE"，表示检测到黄灯，可增加目前的速度行驶；话题内容为"DRIVE"，可保持当前速度；话题
内容为"UNKNOWN",可不做处理
