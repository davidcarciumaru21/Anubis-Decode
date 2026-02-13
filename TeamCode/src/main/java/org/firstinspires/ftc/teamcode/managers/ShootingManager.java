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
        return 1.0;
    }

    public double getTargetPose(double distance) {
        return 1.0;
    }

    private void applyTargets(double distance) {
        outtake.move(getTargetVelocity(distance));
        deflector.move(getTargetPose(distance));
    }

    public void update(double distance) {

        applyTargets(distance);

        switch (state) {

            case IDLE:
                intakingManager.off();
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
