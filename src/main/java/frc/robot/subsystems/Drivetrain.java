package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.motion.Setpoint;

public class Drivetrain {
    private TalonSRX leftDrive, rightDrive;
    private double trackwidth;

    // Convert between feet and encoder ticks
    private static final double ticksPerFoot = 4096 / (Math.PI * 0.5);

    public Drivetrain(TalonSRX leftDrive, TalonSRX rightDrive, double trackwidth) {
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        this.trackwidth = trackwidth;
    }

    public double getTrackWidth() {
        return trackwidth;
    }

    public void initializePID() {
        leftDrive.setSelectedSensorPosition(0, 0, 10);
        rightDrive.setSelectedSensorPosition(0, 0, 10);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        leftDrive.set(ControlMode.PercentOutput, leftSpeed);
        rightDrive.set(ControlMode.PercentOutput, rightSpeed);
    }

    public void arcadeDrive(double forwardSpeed, double turnSpeed) {
        tankDrive(forwardSpeed + turnSpeed, forwardSpeed - turnSpeed);
    }

    public void stop() {
        tankDrive(0.0, 0.0);
    }

    public void PIDDrive(Setpoint leftSetpoint, Setpoint rightSetpoint) {
        double position = leftSetpoint.getPosition();
        double feedforward = leftSetpoint.getVelocity();
        leftDrive.set(ControlMode.Position, position * ticksPerFoot, feedforward * ticksPerFoot);

        position = rightSetpoint.getPosition();
        feedforward = rightSetpoint.getVelocity();
        rightDrive.set(ControlMode.Position, position * ticksPerFoot, feedforward * ticksPerFoot);
    }

    public double getSensorPosition() {
        double avg = 0.5 * (leftDrive.getSelectedSensorPosition(0) + rightDrive.getSelectedSensorPosition(0));
        return avg / ticksPerFoot;
    }

    public double getSensorVelocity() {
        double avg = 0.5 * (leftDrive.getSelectedSensorVelocity(0) + rightDrive.getSelectedSensorVelocity(0));
        return avg / ticksPerFoot;
    }
}
