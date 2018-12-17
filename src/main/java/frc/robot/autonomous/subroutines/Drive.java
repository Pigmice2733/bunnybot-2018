package frc.robot.autonomous.subroutines;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.motion.Setpoint;
import frc.robot.motion.StaticProfile;
import frc.robot.motion.execution.StaticProfileExecutor;
import frc.robot.motion.execution.StaticSteeringController;
import frc.robot.pidf.Gains;
import frc.robot.pidf.PIDF;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Bounds;

public class Drive implements ISubroutine {

    private final StaticProfileExecutor executor;
    private final Drivetrain drive;
    private final StaticSteeringController steering;
    private boolean finished = false;
    private AHRS navx;

    public Drive(Drivetrain drive, AHRS navx, double feet) {
        this.drive = drive;
        this.navx = navx;
        StaticProfile profile = new StaticProfile(0.0, 0.0, feet, 5.0, 1.8, 4.5);
        executor = new StaticProfileExecutor(profile, this::driveOutput, drive::getSensorPosition, 0.05);

        Gains steeringGains = new Gains(0.00005, 0.0, 0.0);
        Bounds steeringBounds = new Bounds(-0.2, 0.2);
        PIDF steeringPID = new PIDF(steeringGains, steeringBounds);
        steering = new StaticSteeringController(this::getAngle, steeringPID);
    }

    public void initialize() {
        drive.initializePID();
        // steering.initialize();
        executor.initialize();
        finished = false;
    }

    private double getAngle() {
        return navx.getAngle();
    }

    public boolean update() {
        if (!finished) {
            finished = executor.update();
        }
        return finished;
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
