# frc2689-java-reefscape
A port of FRC 2689's code to Java for the 2025 Reefscape game. I created this project to learn more about Java programming in FRC as we may transition from LabVIEW in advance of the 2027 control system. 

# Architecture
I am using the Command and Control architecture to port the code, given the fact that we use it with LabVIEW.
There are a couple of ways to implement Commands - you can either write dedicated Command classes in *.java files, or use lambda functions.
I decided to use the lambda function approach. In RobotContainer.java->configureBindings(), you'll find all of the control bindings and the corresponding "commands" (really just subsystem function calls).

# Autos
I haven't gotten there yet, but replicating our Autonomous success will likely be the biggest challenge. We use some internal tooling to make paths / generate paths with LabVIEW, and adjusting to WPILib's standard tools will take some time.
I plan on using the generic pose estimation and path generation code provided in WPILib. If I am unable to accomplish an equal or better result with this tooling, I'll explore other options.

# Motivation
The motivation of this project is to better understand the WPILib ecosystem and how Java has changed in FRC since I used it in 2016-2020. If I can learn the language and API and implement a working codebase, I can help train and assist our future programmers with the transition from LabVIEW to Java (or another WPILib supported language).
