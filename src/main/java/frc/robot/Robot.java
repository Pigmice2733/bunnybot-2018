package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.autonomous.Autonomous;
import frc.robot.autonomous.Radius;
import frc.robot.autonomous.Test;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Utils;
import frc.robot.pidf.Gains;

// max speed : 1.165 feet / second

public class Robot extends TimedRobot {
    private Drivetrain drivetrain;
    private double sensitivity;

    private Joystick joy;
    private Joystick operatorJoy;
    private AHRS navx;

    private TalonSRX intake;
    private TalonSRX outtake;

    private Autonomous autonomous;

    private final double autoOuttakeSpeed = 0.5;
    // 0.0 - 1.0, 0.0 never, 1.0 always
    private final double autoOuttakeInterval = 0.4;
    private final double autoOuttakeOffsetDistance = 5.5; // feet

    public void robotInit() {
        TalonSRX leftDrive = new TalonSRX(0);
        TalonSRX rightDrive = new TalonSRX(2);
        intake = new TalonSRX(4);
        outtake = new TalonSRX(5);

        rightDrive.setInverted(true);

        configureDriveMotor(leftDrive);
        configureDriveMotor(rightDrive);

        leftDrive.setSensorPhase(true);
        rightDrive.setSensorPhase(true);

        VictorSPX leftFollower = new VictorSPX(1);
        configureFollowerMotor(leftFollower, leftDrive);

        VictorSPX rightFollower = new VictorSPX(3);
        configureFollowerMotor(rightFollower, rightDrive);

        drivetrain = new Drivetrain(leftDrive, rightDrive, 3.0);
        joy = new Joystick(0);
        operatorJoy = new Joystick(1);
        navx = new AHRS(SPI.Port.kMXP);

        setPeriod(0.02);
    }

    public void autonomousInit() {
        autonomous = new Test(drivetrain, navx);
        autonomous.initialize();
    }

    public void autonomousPeriodic() {
        final int state = autonomous.update();
        intake.set(ControlMode.PercentOutput, -1);
        final double distance = (Math.abs(drivetrain.getLeftSensorPosition())
                + Math.abs(drivetrain.getRightSensorPosition())) / 2;

        System.out.println(distance);

        if (state >= 4 && distance > autoOuttakeOffsetDistance && distance % 1 > autoOuttakeInterval) {
            outtake.set(ControlMode.PercentOutput, autoOuttakeSpeed);
        }
    }

    public void teleopInit() {
        sensitivity = Utils.lerp(joy.getZ(), 1.0, -1.0, 0.5, 1.0);
    }

    public void teleopPeriodic() {
        double dir = joy.getRawButton(1) ? -1.0 : 1.0;
        drivetrain.arcadeDrive(scaleControl(-dir * joy.getY(), 3.0), sensitivity * scaleControl(joy.getX(), 3.0));
        if (operatorJoy.getRawButton(3)) {
            intake.set(ControlMode.PercentOutput, -1);
        } else {
            intake.set(ControlMode.PercentOutput, 0);
        }
        if (operatorJoy.getRawButton(2)) {
            outtake.set(ControlMode.PercentOutput, 0.5);
        } else {
            outtake.set(ControlMode.PercentOutput, 0);
        }
    }

    public void disabledInit() {
    }

    private double scaleControl(double input, double factor) {
        return Math.copySign(Math.pow(input, factor), input);
    }

    private void configureDriveMotor(IMotorControllerEnhanced motor) {
        configureVoltageComp(motor);
        configurePIDMotor(motor, new Gains(0.4, 0.0015, 0.0004, 0.025, 0.0, 0.0));
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        motor.setSelectedSensorPosition(0, 0, 10);
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
