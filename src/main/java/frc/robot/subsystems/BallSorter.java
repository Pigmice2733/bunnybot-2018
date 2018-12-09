package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class BallSorter {
    private VictorSPX sortMotor, feedMotor;

    private static double feedSpeed = -0.4;
    private static double sortSpeed = 0.12;

    public enum Direction {
        LEFT, RIGHT, STOP,
    }

    public BallSorter(VictorSPX sortMotor, VictorSPX feedMotor) {
        this.sortMotor = sortMotor;
        this.feedMotor = feedMotor;
    }

    public void set(Direction direction) {
        feedMotor.set(ControlMode.PercentOutput, feedSpeed);
        switch (direction) {
        case LEFT:
            sortMotor.set(ControlMode.PercentOutput, sortSpeed);
            break;
        case RIGHT:
            sortMotor.set(ControlMode.PercentOutput, -sortSpeed);
        default:
            sortMotor.set(ControlMode.PercentOutput, 0.0);
            break;
        }
    }
}