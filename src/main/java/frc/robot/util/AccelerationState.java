package frc.robot.util;

public class AccelerationState {
    public final double linearAcceleration; // in m/s^2
    public final double rotationalAcceleration; // in degrees/s^2

    public AccelerationState(double linearAcceleration, double turn) {
        this.linearAcceleration = linearAcceleration;
        this.rotationalAcceleration = turn;
    }
}
