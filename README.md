Here’s a clean README for **82855G V2**, written like something you’d actually be proud to put on GitHub. It shows that this project was a big jump from your fall robot and explains how to use the code without sounding fake or corporate.

You can drop this straight into your repo and paste an image later.

---

# 82855G V2 – PROS Codebase

This repository holds **V2 of the 82855G robot**, the build where everything finally started to click. It’s the version where we cleaned up the fall code, rewrote messy subsystems, tuned the drive properly, and built a robot that could actually run consistent matches and clean autos.

V2 became the foundation for everything we’re running now.

## About This Robot

This robot is a full rework of our fall bot. We kept what worked, cut out everything that didn’t, and rebuilt the rest from scratch. This version includes:

* A fully rewritten drivetrain
* Better LemLib config and real tuning
* More reliable auton structure
* Cleaner subsystem layout
* A pushback that actually works
* A wall-stake that doesn’t send the robot flying

> **Insert your image here:**
> `![82855G V2](path/to/image.png)`

## What’s in This Repo

* `/src` – Source code for each subsystem
* `/include` – Headers, structs, and config files
* `/lemlib` – Your LemLib config and PID constants
* `/autons` – Autonomous routines separated by side + route
* `config.hpp` – Port map + global tuning values
* `main.cpp` – Driver control and main event loop

Everything is organized so you can quickly fix bugs or drop in new subsystems without digging through spaghetti.

## How to Set It Up

1. Install **PROS CLI or PROS for VSCode**.
2. Clone the repo:

   ```bash
   git clone https://github.com/yourname/82855G-V2.git
   ```
3. Open the folder in VSCode with PROS installed.
4. Build:

   ```bash
   pros make
   ```
5. Flash to your VEX Brain:

   ```bash
   pros mu
   ```
6. Check ports in `config.hpp`. Update them if needed.
7. Tune LemLib values to match whatever drivetrain you’re running now.
8. Select your auton in `autonSelector` or change the index in code.

## Key Improvements Over V1

* **Consistent turning** using better PID values
* **Real odometry** instead of fake offsets
* **Cleaner auton code**, easier to switch routes
* **Smoother acceleration curves**
* **Better driver control response**
* **Subsystems actually separated** instead of everything in `main.cpp`
* More reliable pushback
* Wall-stake working without sending the robot backwards

## Features

* Full LemLib integration
* Motion chaining support
* Updated controller bindings
* Organized auton chooser
* Solid tracking wheel math
* Failsafes to prevent stalls and runaway motors
* Easy to expand with future subsystems

## Known Issues / To-Do

* Extreme precision still depends on tuning
* Some autons need more real-field testing
* Horizontal tracking wheel TBD
* Pushback timing can be made smoother
* Clean up magic numbers in a few places

## Credits

Code by **Kerry Li – Team 82855S**, with plenty of late-night debugging, bad decisions, and cylinders firing at random times during testing.

---

If you want, I can also:

* add a “Lessons Learned” section
* write you a cleaner contributor guide
* add GIF placeholders
* include a calibration guide for your odometry

Just tell me what else you want.
