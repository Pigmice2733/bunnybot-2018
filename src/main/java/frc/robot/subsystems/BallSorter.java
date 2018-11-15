package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class BallSorter {
    private WPI_VictorSPX sortMotor, feedMotor;

    private static double feedSpeed = -0.4;
    private static double sortSpeed = 0.12;

    public enum Direction {
        LEFT, RIGHT, STOP,
    }

    public BallSorter(WPI_VictorSPX sortMotor, WPI_VictorSPX feedMotor) {
        this.sortMotor = sortMotor;
        this.feedMotor = feedMotor;
    }

    public void set(Direction direction) {
        feedMotor.set(feedSpeed);
        switch (direction) {
        case LEFT:
            sortMotor.set(sortSpeed);
            break;
        case RIGHT:
            sortMotor.set(-sortSpeed);
        default:
            sortMotor.stopMotor();
            break;
        }
    }
}