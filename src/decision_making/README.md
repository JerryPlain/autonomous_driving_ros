Overview of the dummy_controller Package

The dummy_controller package is designed to make driving decisions based on recognized traffic light states and publish the decision as a string message to the /traffic_decision topic for other modules to use.
Input Topic:

    /traffic_light_color (std_msgs::String)

        Message Format: "id:state:confidence"

        Example: "1:RED:0.92"

        The state field represents the traffic light color, with possible values: "RED", "YELLOW", "GREEN".

Output Topic:

    /traffic_decision (std_msgs::String)

        The published decision result will be one of the following:

            "BRAKE": Red light detected, the vehicle should brake.

            "ACCELERATE": Yellow light detected, the vehicle should accelerate to pass.

            "DRIVE": Green light detected, the vehicle should continue driving normally.

            "UNKNOWN": Traffic light state could not be identified.

Usage by Control Module:

The control module should subscribe to the /traffic_decision topic. The topic content indicates:

    "BRAKE": Red light detected → vehicle should stop.

    "ACCELERATE": Yellow light detected → vehicle should increase speed to pass.

    "DRIVE": Green light detected → vehicle should maintain current speed.

    "UNKNOWN": No traffic light state detected → no action required.

The control module can apply different speed control strategies based on the topic's content.

