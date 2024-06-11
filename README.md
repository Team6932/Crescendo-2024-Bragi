# Development of the FRC 6932 BRAGI robot during 2024 Crescendo

This is the code written for the 2024 FRC Crescendo season by FRC 6932 SMART Sylva, NC. The code for this season was written by one person (not a good idea). The steps I took are explained below. 

# Learning

I began the season by trying to understand how programming in Java worked for FRC (I had no clue what I was doing). I went through our team's old robot code for previous seasons (none were command-based). I also went through our one command-based robot, a small robot from our most recent summer camp. With help from YouTube videos, official APIs, and code from other teams, I started to understand what I was doing. I tried to rewrite the swerve drive code from that summer camp. 

# Robot and Laptop Setup

Starting off, I knew about updating VS Code and Driver Station. I had no idea what other software I would need to use or download. 

- Copy-psated code wasn't working: managed vendor libraries in VS Code. 

- Robot didn't seem to be working: imaged the roboRIO with the roboRIO Imaging Tool. 

- SPARK MAX Motor Controllers weren't connecting: used the REV Hardware Client to configure all of the CAN IDs. That took quite a long time since I was using some bad-quality cables that didn't always connect properly. I also had to deal with reconfiguring motor controllers with the same ID stuck underneath the robot. 

- Radio wasn't connecting: reconfigured it with the FRC Radio Configuration Utility. 

- LimeLight wasn't connecting: set static IPs to everything. The static IPs were working, but the Limelight still wasn't working. I didn't want to accidentally mess something else up while working on the LimeLight, so I stopped trying to make a vision system (I was also doing this while we were in the pits at our State Competition). 

# Swerve Drive

The first project was getting swerve drive to work. I figured our summer camp code would work, so I basically copy-pasted a lot of it into the 2024 WPILib version of VS Code. After the robot frame came together, I started testing. They did not go successfully. Either the robot would only go forward/backward or only spin in place. 

With help from our mentors, we started working with the YAGSL template and resources from FRC 3481 Bronc Botz. It still didn't work, but at least it could move and turn simultaneously. At this point, we started working on setting angle offsets for each swerve module. This ended up being a long struggle. Our mentors and I spent quite a long time reading Shuffleboard and Phoenix Tuner X (CANCoder) values while trying to establish offsets and inversions. When one of our mentors discovered Advantage Scope, we could easily visualize what our robot was doing compared to what it thought it was doing. Afterward, swerve drive finally worked. 

# Shooting Mechanism and Command-Based Programming

The second project was working on our simple shooting mechanism (it was the second thing we finished building). While working with swerve drive, I didn't really learn much about command-based programming since it was just a lot of copy-pasting. I started with programming a timed-based shoot command (deleted). 

In an effort to figure out command-based programming, I watched more YouTube videos and copy-pasted a simple example. I understood the basics, so I created ShootSubsystem.java and ShootCommand.java. These started off being very simple. The subsystem only created the motor controllers and included a method that set the motors to input power levels. The command turned the motors on or off depending on if a button was pressed. 

As I got better at programming for FRC, I started working with encoders. The subsystem was modified to include methods that got encoder values and showed if the motor speed was fast enough for a desired action. 

# Intake Mechanism and Climb Mechanism

The development of these two mechanisms was similar to the shooter. I started off with very simple commands that turned on and off depending on whether a button was held or released. The climb mechanism was scrapped when our team had mechanical failures. The intake mechanism is comprised of two parts. One part spins the wheels that suck up the game piece, and the other moves the entire intake over the bumper. 

The part that spun the wheels remained relatively unchanged. The main additions were getting the encoder and limit switch values. 

Moving the intake in/out was a larger challenge. Initially, one button made it move out, and the other made it move in. It was fully manual, resulting in slow and inconsistent movement. The next step was making it move automatically with PID controls. One accident happened when I extremely stretched the chain during testing. I also set the PID to move to the wrong setpoint since I did not read the encoder values. After I determined the correct setpoints and learned about end conditions for commands, the PID controls began to work. Later in the season, I kept increasing the max speed so we could be even faster. 

# Autonomous Programming

Our coach gave me the idea to use PathWeaver, a program that creates trajectories for the robot to follow. As I did more research, I found that PathPlanner would be more effective since it had support for swerve drive. When I installed PathPlanner, I had no idea what I was doing. 

Luckily, the basic setup was already completed in the code I copy-pasted for swerve drive. Then, I registered the commands for the shooting and intake mechanisms. I started with very simple Paths and Autos. Over time, I got more experience and started working with end states, rotation targets, event markers, speed constraints, and starting state previews. 

# Vision

At the beginning of the season, one of the goals was to automatically get and shoot game pieces. This would be done through a vision system (LimeLight). I only began working on vision later on in the season when the code was working. I had fun messing around with vision for like a day, but we had higher priorities (the robot was falling apart). In the end, vision never became a reality. 

# Competition Developments

The first iteration of our shooting procedure was to turn the shooter motors on, wait for one second, and then feed the piece to the shooter motors by reversing the intake motors. This felt very inefficient, so I built off of ShootCommand.java and made SpeakerCommand.java. When the shooter motors got up to speed, the intake motors would feed the piece. This made shooting much faster. I created separate files for SpeakerCommand, AmpCommand, and PassCommand because I wanted RobotContainer.java to be more organized. However, these commands still had power settings as inputs (very inefficient use of text space). As I wrote my Programming Tips/Guide document, I wrote improved versions of these commands that are more efficient with space. 

Field-oriented drive was a major priority because it would allow us to control the robot much more effectively. At our first competition, we had lots of issues. We had a broken intake, metal shavings in the roboRIO, navX sensor connection issues, and the battery was sliding around like crazy (enough for the safety inspectors to become our besties). To allow the drivers to reset the heading in case errors got too extreme, I made ResetHeadingCommand.java. This malfunctioned at our first competition because the command never ended. After getting a better understanding of end conditions, this worked as intended. 

At our State Competition, our drivers were slow to react whenever something malfunctioned. I made additions to the intake and shoot subsystems that would allow our drivers to know if there were issues with the motors. This was done by using encoder values. 

# Final Thoughts

During the season, my team definitely put too much faith in me. While there were fewer programming errors compared to mechanical failures or design oversights, many issues could have been avoided with more testing. They could have also been avoided if there was more than one person working on the code. Some mistakes were made because I simply did not have enough experience. I had prior coding experience (MATLAB, R, Python) but had no idea what I was doing with Java. Other mistakes were made because there was no one to peer review my inefficient or flawed code. 