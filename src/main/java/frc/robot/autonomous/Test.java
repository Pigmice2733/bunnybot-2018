package frc.robot.autonomous;

import frc.robot.subsystems.Drivetrain;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.autonomous.subroutines.Drive;
import frc.robot.autonomous.subroutines.ISubroutine;
import frc.robot.autonomous.subroutines.Turn;

public class Test implements IAutonomous {
    private ISubroutine subroutine;
    private Drivetrain drive;
    private AHRS navx;
    private State currentState;

    private enum State {
        START, OUT, FIRST_TURN, ACROSS, SECOND_TURN, BACK, END;
    }

    public Test(Drivetrain drive, AHRS navx) {
        this.drive = drive;
        this.navx = navx;
        this.currentState = State.START;
    }

    public void initialize() {
    }

    public void update() {
        boolean finished;

        switch (currentState) {
        case START:
            subroutine = new Drive(drive, navx, 20);
            subroutine.initialize();
            currentState = State.OUT;
            break;
        case OUT:
            finished = subroutine.update();
            if (finished) {
                subroutine = new Turn(drive, navx, -90);
                subroutine.initialize();
                currentState = State.FIRST_TURN;
            }
            break;
        case FIRST_TURN:
            finished = subroutine.update();
            if (finished) {
                subroutine = new Drive(drive, navx, 6);
                subroutine.initialize();
                currentState = State.ACROSS;
            }
            break;
        case ACROSS:
            finished = subroutine.update();
            if (finished) {
                subroutine = new Turn(drive, navx, -90);
                subroutine.initialize();
                currentState = State.SECOND_TURN;
            }
            break;
        case SECOND_TURN:
            finished = subroutine.update();
            if (finished) {
                subroutine = new Drive(drive, navx, 12);
                subroutine.initialize();
                currentState = State.BACK;
            }
            break;
        case BACK:
            finished = subroutine.update();
            if (finished) {
                currentState = State.END;
            }
            break;
        default:
            drive.stop();
            break;
        }
    }
}
