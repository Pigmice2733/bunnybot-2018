package frc.robot.motion.execution;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.pidf.PIDF;

public class StaticSteeringController {
    private AHRS navx;
    private PIDF pid;

    private double target;
    private double startTime;

    private double bias;

    public StaticSteeringController(AHRS navx, PIDF pid) {
        this.navx = navx;
        this.pid = pid;
    }

    public void initialize() {
        target = navx.getAngle();
        startTime = Timer.getFPGATimestamp();

        pid.initialize(0.0, startTime, 0.0);

        bias = 0.0;
    }

    // Returning a wheel distance correction to steer to target heading
    public double correct() {
        double correction = pid.calculateOutput(navx.getAngle(), target, Timer.getFPGATimestamp());

        bias += correction;

        return bias;
    }

}