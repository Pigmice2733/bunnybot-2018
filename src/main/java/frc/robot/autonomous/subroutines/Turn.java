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

public class Turn implements ISubroutine {
    private final StaticProfileExecutor executor;
    private final Drivetrain drive;
    private boolean finished = false;
    private double initialAngle;
    private final StaticSteeringController steering;
    private AHRS navx;

    public Turn(Drivetrain drive, AHRS navx, double degrees) {
        this.drive = drive;
        StaticProfile profile = new StaticProfile(0.0, 0.0, degrees, 50.0, 45.0, 60.0);
        executor = new StaticProfileExecutor(profile, this::driveOutput, this::getAngle, 1);

        this.navx = navx;
        initialAngle = navx.getAngle();

        Gains steeringGains = new Gains(0.00005, 0.00005, 0.0);
        Bounds steeringBounds = new Bounds(-0.2, 0.2);
        PIDF steeringPID = new PIDF(steeringGains, steeringBounds);
        steering = new StaticSteeringController(this::getAngle, steeringPID);
    }

    public void initialize() {
        drive.initializePID();
        steering.initialize();
        executor.initialize();
        finished = false;
    }

    public boolean update() {
        if (!finished) {
            finished = executor.update();
        }
        return finished;
    }

    private double getAngle() {
        return navx.getAngle() - initialAngle;
    }

    private void driveOutput(Setpoint sp) {
        sp = sp.toArcLength(drive.getTrackWidth(), false);

        double correction = steering.correct(sp.getHeading());
        sp = new Setpoint(sp.getPosition() + correction, sp.getVelocity(), sp.getAcceleration(), sp.getCurvature(),
                sp.getHeading());

        drive.PIDDrive(sp, sp.negate());
    }
}
