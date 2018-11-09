package frc.robot.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Drivetrain {
    private WPI_TalonSRX leftDrive, rightDrive;

    public Drivetrain(WPI_TalonSRX leftDrive, WPI_TalonSRX rightDrive) {
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        leftDrive.set(leftSpeed);
        rightDrive.set(-rightSpeed);
    }

    public void arcadeDrive(double forwardSpeed, double turnSpeed) {
        leftDrive.set(forwardSpeed + turnSpeed);
        rightDrive.set(-forwardSpeed + turnSpeed);
    }
}