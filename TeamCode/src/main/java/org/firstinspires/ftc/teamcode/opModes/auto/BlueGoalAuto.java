package org.firstinspires.ftc.teamcode.opModes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.global.AllianceColor;
import org.firstinspires.ftc.teamcode.global.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.systems.Deflector;
import org.firstinspires.ftc.teamcode.systems.Outtake;
import org.firstinspires.ftc.teamcode.systems.Indexer;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.managers.IntakingManager;
import org.firstinspires.ftc.teamcode.managers.ShootingManager;
import java.io.FileWriter;
import java.io.IOException;
import java.io.File;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import com.google.gson.Gson;
import com.google.gson.JsonObject;

@Autonomous(name = "BlueGoalAuto", group = "Blue")
public class BlueGoalAuto extends OpMode {

    private enum States {
        START_TO_SHOOT_PRELOAD,
        FIRST_WAIT,
        SHOOT_PRELOAD,
        SHOOT_PRELAOD_TO_INTAKE_LINE1,
        INTAKE_LINE1_TO_FINISHED_INTAKE_LINE1,
        FINISHED_INTAKE_LINE1_TO_OPEN_GATE,
        OPEN_GATE_TO_SHOOT_LINE1,
        SECOND_WAIT,
        SHOOT_LINE1,
        SHOOT_LINE1_TO_INTAKE_LINE2,
        INTAKE_LINE2_TO_FINISHED_INTAKE_LINE2,
        FINISHED_INTAKE_LINE2_TO_SHOOT_LINE2,
        THIRD_WAIT,
        SHOOT_LINE2,
        SHOOT_LINE2_TO_INTAKE_LINE3,
        INTAKE_LINE3_TO_FINISHED_INTAKE_LINE3,
        FINISHED_INTAKE_LINE3_TO_SHOOT_LINE3,
        FOURTH_WAIT,
        SHOOT_LINE3,
        END
    }

    private States state = States.START_TO_SHOOT_PRELOAD;

    private Follower follower;
    private Paths paths;

    private Deflector deflector;
    private Outtake outtake;
    private Indexer indexer;
    private Intake intake;

    private ShootingManager shootingManager;
    private IntakingManager intakingManager;

    private ElapsedTime timer;
    private ElapsedTime secondTimer;

    private JsonObject json;
    private Gson gson;
    private File file;

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.393, 120.000),

                                    new Pose(48.374, 95.215)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(137))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.374, 95.215),
                                    new Pose(63.294, 83.500),
                                    new Pose(45.092, 84.495)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(45.092, 84.495),

                                    new Pose(15.205, 84.037)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.205, 84.037),
                                    new Pose(31.907, 69.243),
                                    new Pose(12.925, 80.280)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.925, 80.280),

                                    new Pose(48.271, 94.589)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(137))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.271, 94.589),
                                    new Pose(61.874, 61.528),
                                    new Pose(40.897, 60.598)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.897, 60.598),

                                    new Pose(6.907, 60.103)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(6.907, 60.103),
                                    new Pose(34.388, 45.678),
                                    new Pose(48.692, 94.879)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.692, 94.879),
                                    new Pose(58.593, 34.533),
                                    new Pose(41.822, 36.336)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.822, 36.336),

                                    new Pose(6.944, 35.542)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(6.944, 35.542),

                                    new Pose(48.449, 94.879)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))

                    .build();
        }
    }

    private void setPathState(States newState) {
        secondTimer.reset();
        state = newState;
    }

    private void run() {
        switch (state) {

            case START_TO_SHOOT_PRELOAD:
                follower.followPath(paths.Path1);
                setPathState(States.FIRST_WAIT);
                break;

            case FIRST_WAIT:
                if (secondTimer.milliseconds() > 800); {
                    setPathState(States.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    shootingManager.shoot();
                    setPathState(States.END);
                }
                break;

            case SHOOT_PRELAOD_TO_INTAKE_LINE1:
                if (!follower.isBusy() && !shootingManager.isBusy()) {
                    follower.followPath(paths.Path2);
                    setPathState(States.INTAKE_LINE1_TO_FINISHED_INTAKE_LINE1);
                }
                break;

            case INTAKE_LINE1_TO_FINISHED_INTAKE_LINE1:
                if (!follower.isBusy()) {
                    intakingManager.togglePull();
                    follower.followPath(paths.Path3, 0.5, true);
                    setPathState(States.FINISHED_INTAKE_LINE1_TO_OPEN_GATE);
                }
                break;

            case FINISHED_INTAKE_LINE1_TO_OPEN_GATE:
                if (!follower.isBusy()) {
                    intakingManager.togglePull();
                    follower.followPath(paths.Path4, 0.8, false);
                    setPathState(States.OPEN_GATE_TO_SHOOT_LINE1);
                }
                break;

            case OPEN_GATE_TO_SHOOT_LINE1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    setPathState(States.SECOND_WAIT);
                }
                break;

            case SECOND_WAIT:
                if (secondTimer.milliseconds() > 1500); {
                    setPathState(States.SHOOT_LINE1);
                }
                break;

            case SHOOT_LINE1:
                if (!follower.isBusy()) {
                    shootingManager.shoot();
                    setPathState(States.SHOOT_LINE1_TO_INTAKE_LINE2);
                }
                break;

            case SHOOT_LINE1_TO_INTAKE_LINE2:
                if (!follower.isBusy() && !shootingManager.isBusy()) {
                    follower.followPath(paths.Path6);
                    setPathState(States.INTAKE_LINE2_TO_FINISHED_INTAKE_LINE2);
                }
                break;

            case INTAKE_LINE2_TO_FINISHED_INTAKE_LINE2:
                if (!follower.isBusy()) {
                    intakingManager.togglePull();
                    follower.followPath(paths.Path7, 0.5, true);
                    setPathState(States.FINISHED_INTAKE_LINE2_TO_SHOOT_LINE2);
                }
                break;

            case FINISHED_INTAKE_LINE2_TO_SHOOT_LINE2:
                if (!follower.isBusy()) {
                    intakingManager.togglePull();
                    follower.followPath(paths.Path8);
                    setPathState(States.THIRD_WAIT);
                }
                break;

            case THIRD_WAIT:
                if (secondTimer.milliseconds() > 1000); {
                    setPathState(States.SHOOT_LINE2);
                }
                break;

            case SHOOT_LINE2:
                if (!follower.isBusy()) {
                    shootingManager.shoot();
                    setPathState(States.SHOOT_LINE2_TO_INTAKE_LINE3);
                }
                break;

            case SHOOT_LINE2_TO_INTAKE_LINE3:
                if (!follower.isBusy() && !shootingManager.isBusy()) {
                    follower.followPath(paths.Path9);
                    setPathState(States.INTAKE_LINE3_TO_FINISHED_INTAKE_LINE3);
                }
                break;

            case INTAKE_LINE3_TO_FINISHED_INTAKE_LINE3:
                if (!follower.isBusy()) {
                    intakingManager.togglePull();
                    follower.followPath(paths.Path10, 0.5, true);
                    setPathState(States.FINISHED_INTAKE_LINE3_TO_SHOOT_LINE3);
                }
                break;

            case FINISHED_INTAKE_LINE3_TO_SHOOT_LINE3:
                if (!follower.isBusy()) {
                    intakingManager.togglePull();
                    follower.followPath(paths.Path11);
                    setPathState(States.SHOOT_LINE3);
                }
                break;

            case FOURTH_WAIT:
                if (secondTimer.milliseconds() > 800); {
                    setPathState(States.SHOOT_LINE3);
                }
                break;

            case SHOOT_LINE3:
                if (!follower.isBusy()) {
                    shootingManager.shoot();
                    setPathState(States.END);
                }
                break;

            case END:
                break;
        }
    }

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(
                new Pose(
                        18.392523364485978,
                        120,
                        Math.toRadians(-37)
                )
        );
        timer = new ElapsedTime();
        secondTimer = new ElapsedTime();
        follower.update();

        deflector = new Deflector(hardwareMap);
        outtake = new Outtake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        intake = new Intake(hardwareMap);

        intakingManager = new IntakingManager(intake);
        shootingManager = new ShootingManager(outtake, indexer, deflector, intakingManager);

        paths = new Paths(follower);
    }

    @Override
    public void start() {
        timer.reset();

        telemetry.addLine("Started auto");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        shootingManager.update(
                follower.getPose().distanceFrom(Poses.blueGoalPose),
                timer.seconds(),
                follower.poseTracker.getVelocity(),
                Math.atan2((Poses.blueGoalPose.getY() - follower.getPose().getY()), (Poses.blueGoalPose.getX() - follower.getPose().getX()))
        );
        intakingManager.update();
        run();

        timer.reset();
    }

    @Override
    public void stop() {
        Pose currentPose = follower.getPose();

        json = new JsonObject();
        json.addProperty("x", currentPose.getX());
        json.addProperty("y", currentPose.getY());
        json.addProperty("heading", currentPose.getHeading());
        json.addProperty("color", AllianceColor.BLUE.toString());

        gson = new Gson();
        file = AppUtil.getInstance().getSettingsFile("RobotSettings.json");

        try (FileWriter writer = new FileWriter(file)) {
            gson.toJson(json, writer);
        } catch (IOException ignored) {}

        telemetry.addLine("Start Code #24037");
        telemetry.update();
    }
}