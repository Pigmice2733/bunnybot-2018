package frc.robot.autonomous;

import frc.robot.subsystems.Drivetrain;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.autonomous.subroutines.Drive;
import frc.robot.autonomous.subroutines.Turn;

public class Test extends Autonomous {
    public Test(Drivetrain drive, AHRS navx) {
        this.subroutines = Arrays.asList(new Drive(drive, navx, 20), new Turn(drive, navx, -90),
                new Drive(drive, navx, 6), new Turn(drive, navx, -90), new Drive(drive, navx, 12));
    }

    public void initialize() {
    }
}
