package frc.robot.autonomous;

import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Bounds;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.motion.Setpoint;
import frc.robot.motion.StaticProfile;
import frc.robot.motion.execution.StaticProfileExecutor;
import frc.robot.motion.execution.StaticSteeringController;
import frc.robot.pidf.Gains;
import frc.robot.pidf.PIDF;

public class Test implements IAutonomous {
    private final StaticProfileExecutor executor;
    private final Drivetrain drive;
    private final StaticSteeringController steering;
    private boolean finished = false;

    public Test(Drivetrain drive, AHRS navx) {
        this.drive = drive;
        StaticProfile profile = new StaticProfile(0.0, 0.0, 10.0, 5.0, 1.8, 4.5);
        executor = new StaticProfileExecutor(profile, this::driveOutput, drive::getSensorPosition, 0.05);

        Gains steeringGains = new Gains(0.0, 0.0, 0.0);
        Bounds steeringBounds = new Bounds(-0.2, 0.2);
        PIDF steeringPID = new PIDF(steeringGains, steeringBounds);
        steering = new StaticSteeringController(navx, steeringPID);
    }

    public void initialize() {
        drive.initializePID();
        // steering.initialize();
        executor.initialize();
        finished = false;
    }

    public void update() {
        if (!finished) {
            finished = executor.update();
        } else {
            drive.stop();
        }
    }

    private void driveOutput(Setpoint sp) {
        double correction = steering.correct();
        Setpoint left = new Setpoint(sp.getPosition() - correction, sp.getVelocity(), sp.getAcceleration(),
                sp.getCurvature(), sp.getHeading());
        Setpoint right = new Setpoint(sp.getPosition() + correction, sp.getVelocity(), sp.getAcceleration(),
                sp.getCurvature(), sp.getHeading());
        drive.PIDDrive(left, right);
    }
}
