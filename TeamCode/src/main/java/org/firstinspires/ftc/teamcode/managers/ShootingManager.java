package org.firstinspires.ftc.teamcode.managers;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.global.ShootingConstants;
import org.firstinspires.ftc.teamcode.global.SystemsConstants;
import org.firstinspires.ftc.teamcode.systems.Deflector;
import org.firstinspires.ftc.teamcode.systems.Indexer;
import org.firstinspires.ftc.teamcode.systems.Outtake;

import org.firstinspires.ftc.teamcode.utils.MathUtils;
import org.firstinspires.ftc.teamcode.utils.Pair;

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

    public void setIdle() {
        state = State.IDLE;
    }

    public void setShoot() {
        state = State.SHOOT;
    }

    public Pair<Double, Double> getTargetAngleAndVelocity(double distance,
                                                          Vector velocityVector,
                                                          double angleToGoal) {

        double height;
        double angle;
        double g;

        if (distance < 125) {
            distance = distance - ShootingConstants.Close.PASS_THROUGH_POINT_RADIUS;
            height =  ShootingConstants.Close.SCORE_HEIGHT;
            angle = ShootingConstants.Close.SCORE_ANGLE;
            g = ShootingConstants.Close.g;
        } else {
            distance = distance - ShootingConstants.Far.PASS_THROUGH_POINT_RADIUS;
            height =  ShootingConstants.Far.SCORE_HEIGHT;
            angle = ShootingConstants.Far.SCORE_ANGLE;
            g = ShootingConstants.Far.g;
        }

        double initialAngle = MathUtils.clamp(
                        Math.atan(2 * height / distance - Math.tan(angle)),
                        SystemsConstants.MIN_HOOD_ANGLE,
                        SystemsConstants.MAX_HOOD_ANGLE
        );

        double initialFlyWheelSpeed = Math.sqrt(
                (g * Math.pow(distance, 2)) /
                        (2 * Math.pow(Math.cos(initialAngle), 2) *
                                (distance * Math.tan(initialAngle)
                                        - height))
        );

        double tetha = velocityVector.getTheta() - angleToGoal;

        double paralelComponent =
                -Math.cos(tetha) * velocityVector.getMagnitude();

        double perpendicularComponent =
                Math.sin(tetha) * velocityVector.getMagnitude();

        double yFlyWheelSpeedComponent =
                initialFlyWheelSpeed * Math.sin(initialAngle);

        double time =
                distance / (initialFlyWheelSpeed * Math.cos(initialAngle));

        double xComponentCompensation =
                distance / time + paralelComponent;

        double newXSpeed = Math.sqrt(
                Math.pow(xComponentCompensation, 2)
                        + Math.pow(perpendicularComponent, 2)
        );

        double newDistance = newXSpeed * time;

        double newAngle = MathUtils.clamp(
                Math.atan(yFlyWheelSpeedComponent / newXSpeed),
                SystemsConstants.MIN_HOOD_ANGLE,
                SystemsConstants.MAX_HOOD_ANGLE
        );

        double flyWheelSpeed = Math.sqrt(
                (g * Math.pow(newDistance, 2)) /
                        (2 * Math.pow(Math.cos(newAngle), 2) *
                                (newDistance * Math.tan(newAngle)
                                        - height))
        );

        if (Double.isNaN(newAngle) || Double.isNaN(flyWheelSpeed)) {
            return new Pair<>(SystemsConstants.MIN_HOOD_ANGLE, 0.0);
        } else {
            return new Pair<>(newAngle, flyWheelSpeed);
        }
    }

    private void applyTargets(Pair<Double, Double> velocityAndAngle) {
        deflector.moveAtAngleInRadians(velocityAndAngle.first);
        outtake.moveBallAtInchesPerSeconds(velocityAndAngle.second);
    }

    public void update(double distance, double time, Vector velocityVector, double angleToGoal) {


        if(distance < 30){
            applyTargets(getTargetAngleAndVelocity(30, velocityVector, angleToGoal));
        } else {
            applyTargets(getTargetAngleAndVelocity(distance, velocityVector, angleToGoal));
        }
        outtake.update(time);

        switch (state) {

            case IDLE:
                indexer.off();
                break;

            case SHOOT:
                if (timer.milliseconds() > threeBallsTimeMs) {
                    state = State.IDLE;
                } else {
                    intakingManager.shoot();
                    indexer.pull();
                }
                break;
        }
    }
}
