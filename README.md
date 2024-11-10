# Smart Museum Security Robot with TurtleBot

This repository contains the source code and documentation for a smart museum security system implemented using TurtleBot. The system utilises various sensors to monitor and secure different areas within the museum environment.

## Project Overview

The project simulates a smart museum environment where a TurtleBot serves as a security robot. The TurtleBot responds to sensor states and performs tasks such as navigating to different rooms based on button presses, monitoring door openings, and responding to security breaches detected by motion sensors in sensitive areas like the vault.

In this repository, you'll find the Python script (`move_bot.py`) that controls the movement and actions of the TurtleBot based on MQTT data received from sensors. Additionally, the repository includes configuration files (`door_rules.js`, `items.js`, `rrd4j_persists.js`) used in OpenHAB for sensor integration and rules.


## Features

- **Movement Control**: The TurtleBot navigates between designated locations (Control Room, Great Hall, Vault, and Charging Stations) based on MQTT sensor inputs.
- **Image Capture**: Capable of capturing images upon specific events (e.g., motion detection in the Vault) and converting them into PDF reports.
- **Integration with MQTT**: Uses MQTT for real-time communication with sensors and OpenHAB.
- **Security Response**: Activate silent alarms, capture images upon security breaches, and generate PDF reports for security teams.
- **Integration with OpenHAB**: Includes configuration files (`door_rules.js`, `items.js`, `rrd4j_persists.js`) for OpenHAB rules and sensor management.
  - **Charging Station Detection**: Automatically navigate to the nearest charging station upon low battery using colour detection.
- **Entry Count Monitoring**: Track the number of entries into the museum using door sensors.
- **User Interface**: Utilises HABPanel for a user-friendly interface to monitor and control the TurtleBot and security sensors.


## Project Presentation

For more detailed information about the project, including system architecture, sensor functionalities, and demonstrations, refer to the presentation PowerPoint provided ([Project Presentation](Presentation.pptx.zip)).

## How to Use

1. **Clone the Repository**: Clone this repository to your local machine using Git.
   ```bash
   git clone https://github.com/rubenodamo/smart-museum-turtlebot.git
   ```
   
2. **Install Dependencies**: Ensure you have all necessary dependencies installed, including ROS and Python libraries used in `move_bot.py`.

3. **Run the Script**: Execute `move_bot.py` on your TurtleBot or simulation environment to start the security robot functionalities.

## Additional Improvements

- **Expand Sensor Coverage**: Consider adding more sensors for enhanced security coverage.
- **Multiple TurtleBots**: Integrate multiple TurtleBots to improve response time and coverage across the museum.
- **Machine Learning Integration**: Implement AI/ML for advanced security analytics and anomaly detection.

#### Contributors

- Ruben Odamo
- Tanaya Patel ([@Tanaya-27](https://github.com/Tanaya-27))
- Lauryn Williams-Lewis ([@laurynn-wl](https://github.com/laurynn-wl))
- Matthew Pryke ([@MattCode2003](https://github.com/MattCode2003))
