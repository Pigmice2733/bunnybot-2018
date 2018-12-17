package frc.robot.motion.execution;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.pidf.PIDF;

public class StaticSteeringController {
    private Input input;
    private PIDF pid;

    private double target;
    private double startTime;

    private double bias;

    public interface Input {
        double get();
    }

    public StaticSteeringController(Input input, PIDF pid) {
        this.input = input;
        this.pid = pid;
    }

    public void initialize() {
        target = input.get();
        startTime = Timer.getFPGATimestamp();

        pid.initialize(0.0, startTime, 0.0);

        bias = 0.0;
    }

    // Returning a wheel distance correction to steer to target heading
    public double correct() {
        return correct(target);
    }

    // Returning a wheel distance correction to steer to target heading
    public double correct(double target) {
        this.target = target;
        double correction = pid.calculateOutput(input.get(), target, Timer.getFPGATimestamp());

        bias += correction;

        return bias;
    }

}