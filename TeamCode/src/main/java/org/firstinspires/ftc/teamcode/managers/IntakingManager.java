package org.firstinspires.ftc.teamcode.managers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.global.SystemsConstants;

public class IntakingManager {
    private final ElapsedTime timer = new ElapsedTime();
    private final Intake intake;

    private enum State {
        IDLE,
        PULL,
        SHOOT,
        REVERSE
    }

    private State currentState = State.IDLE;



    public IntakingManager(Intake intake) {
        this.intake = intake;
    }

    public void togglePull() {
        currentState = (currentState == State.PULL) ? State.IDLE : State.PULL;
        timer.reset();
    }

    public void reverse(){
        currentState = State.REVERSE;
    }

    public void shoot() {
        currentState = State.SHOOT;
    }

    public void off() {
        currentState = State.IDLE;
    }

    public boolean isBusy() {
        return currentState != State.IDLE;
    }

    public void update() {

        switch (currentState) {

            case IDLE:
                intake.off();
                break;

            case PULL:

                if (intake.getCurrent() >= SystemsConstants.INTAKE_STALL_CURRENT && timer.milliseconds() > 300) {
                    intake.off();
                    currentState = State.IDLE;
                    break;
                }

                intake.pull();
                break;

            case SHOOT:
                intake.pull();
                break;

            case REVERSE:
                intake.push();
                break;
        }
    }
}
