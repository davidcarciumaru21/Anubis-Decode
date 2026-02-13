package org.firstinspires.ftc.teamcode.managers;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.global.SystemsConstants;
import org.firstinspires.ftc.teamcode.systems.Deflector;
import org.firstinspires.ftc.teamcode.systems.Indexer;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Outtake;

import org.firstinspires.ftc.teamcode.global.SystemsConstants;

public class ShootingManager {

    private final Outtake outtake;
    private final Indexer indexer;
    private final Deflector deflector;

    private final IntakingManager intakingManager;

    private enum State {
        IDLE,
        SHOOT
    }

    private State state = State.IDLE;

    private final ElapsedTime timer = new ElapsedTime();
    private double threeBallsTimeMs = SystemsConstants.THREE_BALLS_TIME_MS;

    public ShootingManager(Outtake outtake,
                           Indexer indexer,
                           Deflector deflector,
                           IntakingManager intakingManager) {
        this.outtake = outtake;
        this.indexer = indexer;
        this.deflector = deflector;

        this.intakingManager = intakingManager;
    }

    public boolean isBusy() {
        return state != State.IDLE;
    }

    public void shoot() {
        if (state == State.IDLE) {
            state = State.SHOOT;
            timer.reset();
        }
    }

    public double getTargetVelocity(double distance) {
        return 0.0227722 * Math.pow(distance, 2) + 5.57798 * distance + 2294.11435;
    }

    public double getTargetPose(double distance) {
        return (2.98921 * Math.pow(10, -7)) * Math.pow(distance, 3) - 0.00000316315 * Math.pow(distance, 2) - 0.012683 * distance + 1.20283;
    }

    private void applyTargets(double distance) {
        outtake.move(getTargetVelocity(distance));
        deflector.move(getTargetPose(distance));
    }

    public void update(double distance, double time) {

        applyTargets(distance);
        outtake.update(time);

        switch (state) {

            case IDLE:
                indexer.off();
                break;

            case SHOOT:
                if (timer.milliseconds() <= threeBallsTimeMs) {
                    intakingManager.shoot();
                    indexer.pull();
                } else {
                    intakingManager.off();
                    indexer.off();
                    state = State.IDLE;
                }
                break;
        }
    }
}
