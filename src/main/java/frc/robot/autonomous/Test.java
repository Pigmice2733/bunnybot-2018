package frc.robot.autonomous;

import frc.robot.subsystems.Drivetrain;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.autonomous.subroutines.Drive;
import frc.robot.autonomous.subroutines.Turn;

public class Test extends Autonomous {
    private AHRS navx;

    public Test(Drivetrain drive, AHRS navx) {
        this.navx = navx;
        this.subroutines = Arrays.asList(new Drive(drive, navx, 20.0, 0.0), new Turn(drive, navx, 90.0, true),
                new Drive(drive, navx, 8.0, 90.0), new Turn(drive, navx, 180.0, true),
                new Drive(drive, navx, 15.0, 180.0));
    }

    public void initialize() {
        navx.setAngleAdjustment(-navx.getAngle());
    }
}
