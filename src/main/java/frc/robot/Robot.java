package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.autonomous.IAutonomous;
import frc.robot.autonomous.Test;
import frc.robot.autonomous.Turn;
import frc.robot.subsystems.Drivetrain;
import frc.robot.pidf.Gains;
import frc.robot.subsystems.BallSorter;

public class Robot extends TimedRobot {
    private Drivetrain drivetrain;
    private BallSorter sorter;

    private Vision vision;

    private Joystick joy;
    private AHRS navx;

    private IAutonomous autonomous;

    public void robotInit() {
        TalonSRX leftDrive = new TalonSRX(0);
        TalonSRX rightDrive = new TalonSRX(2);

        rightDrive.setInverted(true);

        configureDriveMotor(leftDrive);
        configureDriveMotor(rightDrive);

        VictorSPX leftFollower = new VictorSPX(1);
        configureFollowerMotor(leftFollower, leftDrive);

        VictorSPX rightFollower = new VictorSPX(3);
        configureFollowerMotor(rightFollower, rightDrive);

        drivetrain = new Drivetrain(leftDrive, rightDrive, 3.0);
        joy = new Joystick(0);
        navx = new AHRS(SPI.Port.kMXP);

        // VictorSPX sortMotor = new VictorSPX(5);
        // VictorSPX feedMotor = new VictorSPX(4);

        // sorter = new BallSorter(sortMotor, feedMotor);
        vision = new Vision(this::isEnabled);

        setPeriod(0.02);
    }

    public void autonomousInit() {
        vision.start();
        autonomous = new Turn(drivetrain, navx);
        autonomous.initialize();
    }

    public void autonomousPeriodic() {
        Vision.Color ballColor = vision.getBallColor();
        // switch (ballColor) {
        // case RED:
        // sorter.set(BallSorter.Direction.LEFT);
        // break;
        // case BLUE:
        // sorter.set(BallSorter.Direction.RIGHT);
        // break;
        // default:
        // sorter.set(BallSorter.Direction.STOP);
        // break;
        // }

        autonomous.update();
    }

    public void teleopInit() {

    }

    public void teleopPeriodic() {
        drivetrain.arcadeDrive(-joy.getY(), joy.getX());
    }

    public void disabledInit() {
        vision.stop();
    }

    private void configureDriveMotor(IMotorControllerEnhanced motor) {
        configureVoltageComp(motor);
        configurePIDMotor(motor, new Gains(0.5, 0.003, 0.0, 0.025, 0.0, 0.0));
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        motor.setSensorPhase(motor.getInverted());
    }

    private void configureFollowerMotor(IMotorController follower, IMotorController leader) {
        configureVoltageComp(follower);
        follower.follow(leader);

        follower.setInverted(leader.getInverted());
    }

    private void configureVoltageComp(IMotorController motor) {
        motor.configVoltageCompSaturation(11.0, 10);
        motor.enableVoltageCompensation(true);
        motor.configVoltageMeasurementFilter(32, 10);
    }

    private static void configurePIDMotor(IMotorControllerEnhanced motor, Gains positionGains) {
        motor.config_kF(0, positionGains.kF(), 0);
        motor.config_kP(0, positionGains.kP(), 0);
        motor.config_kI(0, positionGains.kI(), 0);
        motor.config_kD(0, positionGains.kD(), 0);
    }
}
