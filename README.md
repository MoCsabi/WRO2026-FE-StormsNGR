# Introduction
We are the StormsNGR team from Hungary, and we are competing in the 2026 WRO Future Engineers category. This is our third year after two very successful years of competition. You can find our documentation from 2024 [here](https://github.com/MoCsabi/WRO2024-FE-StormsNGR) (🥇) and from 2025 [here](https://github.com/MoCsabi/WRO2025-FE-StormsNGR) (🥉). We will be updating this repository with our progress, which will be documented in our new [Engineering Journal](/Engineering%20Journal.md).

## The team
>Team members
- **Csaba Molnár** from [Budapest University of Technology and Economics](https://www.bme.hu/en) Faculty of Electrical Engineering and Informatics
  - csabi@molnarnet.hu
- **András Gräff** from [Budapest University of Technology and Economics](https://www.bme.hu/en) Faculty of Mechanical Engineering
  - andrasgraff@gmail.com
- **Levente Molnár** from [Vocational Technical School and High School of Biatorbágy](https://en.bit-edu.hu/)
  - levente@molnarnet.hu
>Coach
- József Balázs Gräff
  - graffjozsefb@gmail.com

Links to our socials:

- **Facebook**: https://www.facebook.com/stormsteam/
  - Here you can find updates about the team
- **YouTube**: https://www.youtube.com/channel/UCyzm_Su7qoRCof-ZpbG_9Ig
  - Here you can watch videos about past competitions
- **Instagram**: https://www.instagram.com/storms_team_hun/
  - Cool posts and updates about our, and our sister-team StormsRMS' preparation for upcoming competitions

# Abstract
Our solution is an autonomous car powered by a Raspberry Pi coded in Python, responsible for the main challenge logic, and an ESP microcontroller coded in C, responsible for controlling the motors and processing sensor data. The two devices communicate using UART. We use a 360° 2D LiDAR combined with a gyro to always know where the robot is on the mat. Detection of the traffic sign's color is done by combining the output of the LiDAR with the camera feed to know exactly where the object is. We use ackermann steering geometry calibrated so the car can leave the parking space in one continuous arc. Driving is powered by 1 DC motor through a differential gearbox designed from scratch, with a top speed of around 2.6 m/s.

# Table of contents

### [Our journey](/Our%20Journey.md)
### [Engineering Journal](/Engineering%20Journal.md)
### [Ideas and principles](/Ideas_and_principles.md)
### [Hardware documentation](/schemes/README.md)
### [Software documentation](/src/README.md)
### [measurement- The robot's mechanical characteristics](/The%20robot's%20mechanical%20characteristics.md)
### [RpiCode development tool documentation](/other/README.md)
### [Videos](/video/video.md)
### [Team Photos](/t-photos/)
### [Vehicle Photos](/v-photos/)

# Special thanks
Special thanks to Zsolt Molnár for all the mentoring he's done for us, József Molnár from the [Budapest University of Technology and Economics](https://www.bme.hu/) Faculty of Mechanical Engineering for helping us with the preparations, György Fenyvesi for helping us develop our prototype [custom made interconnect panel](/schemes/README.md/#custom-made-interconnect-panel-wiring-with-connections-labeled), and a bunch of others for helping us by reviewing all the documentation. And of course, to both our families for tolerating us taking up the entire living room with the robot mat 🙂.
