package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.subsystems.BallSorter;

public class Robot extends TimedRobot {
    private Drivetrain drivetrain;
    private BallSorter sorter;

    private Vision vision;

    private Joystick joy;

    public void robotInit() {
        WPI_TalonSRX leftDrive = new WPI_TalonSRX(0);
        WPI_TalonSRX rightDrive = new WPI_TalonSRX(2);

        WPI_TalonSRX leftFollower = new WPI_TalonSRX(1);
        leftFollower.set(ControlMode.Follower, leftDrive.getDeviceID());

        WPI_TalonSRX rightFollower = new WPI_TalonSRX(3);
        rightFollower.set(ControlMode.Follower, rightDrive.getDeviceID());

        drivetrain = new Drivetrain(leftDrive, rightDrive);
        joy = new Joystick(0);

        WPI_VictorSPX sortMotor = new WPI_VictorSPX(5);
        WPI_VictorSPX feedMotor = new WPI_VictorSPX(4);

        sorter = new BallSorter(sortMotor, feedMotor);
        vision = new Vision(this::isEnabled);

        setPeriod(0.02);
    }

    public void autonomousInit() {
        vision.start();
    }

    public void autonomousPeriodic() {
        Vision.Color ballColor = vision.getBallColor();
        switch (ballColor) {
        case RED:
            sorter.set(BallSorter.Direction.LEFT);
            break;
        case BLUE:
            sorter.set(BallSorter.Direction.RIGHT);
            break;
        default:
            sorter.set(BallSorter.Direction.STOP);
            break;
        }
    }

    public void teleopInit() {

    }

    public void teleopPeriodic() {
        drivetrain.arcadeDrive(-joy.getY(), joy.getX());
    }

    public void disabledInit() {
        vision.stop();
    }
}
