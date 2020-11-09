# Using the FRC Simulator

This project provides a simple simulated robot that can be programmed
to navigate in the simulator UI.

This project depends on the WPILib packages for the FIRST Robotics Competition. Information on these packages here: https://docs.wpilib.org/en/stable/

## About this simulator

There is an `AutoDriveCommand` class provided, which makes use of several classes to provide "plausible" simulated physics, and to interface with
the simulator UI. By changing the implementation of the method `execute` within the `AutoDriveCommand` class, you should be able to make the simulated
robot move on the screen.

See the `TODO` comment in `AutoDriveCommand.execute` for more information on the first task to complete.

## How to run the simulation

The simulation can either be run from the command line:

```
gradle simulateJava
```

Or from the Command Palette in Visual Studio Code bu selecting the menu item: `WPILib: Simulate Robot Code on Desktop`
