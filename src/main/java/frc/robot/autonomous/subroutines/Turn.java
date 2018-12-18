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
    private StaticProfileExecutor executor;
    private final StaticSteeringController steering;

    private AHRS navx;
    private final Drivetrain drive;

    private double angleOffset = 0.0;
    private double targetAngle = 0.0;
    private boolean finished = false;
    private boolean absolute = false;

    public Turn(Drivetrain drive, AHRS navx, double degrees) {
        this(drive, navx, degrees, false);
    }

    public Turn(Drivetrain drive, AHRS navx, double degrees, boolean absolute) {
        this.drive = drive;
        this.navx = navx;
        this.absolute = absolute;
        this.targetAngle = degrees;

        Gains steeringGains = new Gains(0.00005, 0.00035, 0.0);
        Bounds steeringBounds = new Bounds(-0.2, 0.2);
        PIDF steeringPID = new PIDF(steeringGains, steeringBounds);
        steering = new StaticSteeringController(this::getAngle, steeringPID);
    }

    public void initialize() {
        double initialAngle = 0.0;
        if (absolute) {
            initialAngle = targetAngle - getAngle();
            angleOffset = 0.0;
        } else {
            angleOffset = getAngle();
        }

        StaticProfile profile = new StaticProfile(0.0, initialAngle, targetAngle, 70.0, 60.0, 80.0);
        executor = new StaticProfileExecutor(profile, this::driveOutput, this::getAngle, 1);

        drive.initializePID();
        steering.initialize(initialAngle);
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
        return -navx.getAngle() - angleOffset;
    }

    private void driveOutput(Setpoint sp) {
        sp = sp.toArcLength(drive.getTrackWidth(), false);

        double correction = steering.correct(sp.getHeading());
        sp = new Setpoint(sp.getPosition() - correction, sp.getVelocity(), sp.getAcceleration(), sp.getCurvature(),
                sp.getHeading());

        drive.PIDDrive(sp, sp.negate());
    }
}
