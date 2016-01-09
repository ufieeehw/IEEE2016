Mission Planner
==============

#Goals
Currently, this is the plan:

```
1. Move forward until you get close to the wall.
2. Move left until vision detects QR codes of B blocks.
3. Wait until the robot has loaded all of it's blocks.
4. Move left until near B blocks.
5. Detect box colors, and save box locations.
6. Move based on color of block.
7. Move back to position in (4) and go to (5) until blocks are gone.
```

Currently, this only solves one of the three tasks. Once this works, we will work on developing more states for more tasks.


We wanted to use SLAM as a way of keeping track of the robot's pose, but due to our sensors' racist tendencies, this will take much more time then I'd like to spend on it. This is a goal to implement after we have a working robot.
Instead of SLAM, we will use our other sensors to generate odometery (this should still be accurate enough) and use much simpler methods of generating where we want Shia to go.

#TODO

- [ ] Finish TODO

#####Physical
- [ ] Build Field

#####Controls
- [ ] Get IMU publishing IMU ros messages
- [ ] Publish Encoder data with ros
- [ ] Generate Odometry with IMU/Encoder data
- [ ] Get last year's controller working

#####Vision
- [ ] Test QR code reading abilities (from how far can we see them, how many can we pinpoint in one frame, etc)
- [ ] Generate 3d point based on QR code position (for train boxes and blocks)
- [ ] VSLAM (this would be awesome)
