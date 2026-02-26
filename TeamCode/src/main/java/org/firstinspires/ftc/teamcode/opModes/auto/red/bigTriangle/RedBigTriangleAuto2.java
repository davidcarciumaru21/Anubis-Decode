package org.firstinspires.ftc.teamcode.opModes.auto.red.bigTriangle;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import static org.firstinspires.ftc.teamcode.utils.AutoUtils.mirrorHeading;

@Autonomous(name = "RedBigTriangleAuto2", group = "Red")
public class RedBigTriangleAuto2 extends OpMode {

    private enum States {
        START_TO_SHOOT_PRELOAD,
        SHOOT_PRELOAD,
        SHOOT_PRELOAD_TO_INTAKE_LINE1,
        INTAKE_LINE1_TO_FINISHED_INTAKE_LINE1,
        FINISHED_INTAKED_LINE1_TO_OPEN_GATE,
        OPEN_GATE_TO_SHOOT_LINE1,
        SHOOT_LINE1,
        SHOOT_LINE1_TO_INTAKE_LINE2,
        INTAKE_LINE2_TO_FINISHED_INTAKE_LINE2,
        FINISHED_INTAKE_LINE2_TO_SHOOT_LINE2,
        SHOOT_LINE2,
        SHOOT_LINE2_TO_INTAKE_LINE3,
        INTAKE_LINE3_TO_FINISHED_INTAKE_LINE3,
        FINISHED_INTAKE_LINE3_TO_SHOOT_LINE3,
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
                                    new Pose(20.336, 122.617).mirror(),

                                    new Pose(47.477, 95.664).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(-36)), Math.toRadians(mirrorHeading(131)))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(47.477, 95.664).mirror(),
                                    new Pose(63.182, 83.065).mirror(),
                                    new Pose(42.626, 83.963).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(131)), Math.toRadians(mirrorHeading(180)))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.626, 83.963).mirror(),

                                    new Pose(16.748, 83.692).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(180)), Math.toRadians(mirrorHeading(180)))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.748, 83.692).mirror(),
                                    new Pose(32.855, 75.963).mirror(),
                                    new Pose(17.196, 76.963).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(180)), Math.toRadians(mirrorHeading(90)))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.196, 76.963).mirror(),

                                    new Pose(47.477, 95.664).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(90)), Math.toRadians(mirrorHeading(131)))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(47.477, 95.664).mirror(),
                                    new Pose(49.201, 58.192).mirror(),
                                    new Pose(42.626, 59.710).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(131)), Math.toRadians(mirrorHeading(180)))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.626, 59.710).mirror(),

                                    new Pose(9.794, 59.682).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(180)), Math.toRadians(mirrorHeading(180)))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(9.794, 59.682).mirror(),
                                    new Pose(51.439, 57.243).mirror(),
                                    new Pose(59.140, 84.224).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(180)), Math.toRadians(mirrorHeading(135)))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.140, 84.224).mirror(),
                                    new Pose(61.093, 33.159).mirror(),
                                    new Pose(42.626, 35.888).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(135)), Math.toRadians(mirrorHeading(180)))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.626, 35.888).mirror(),

                                    new Pose(9.794, 35.888).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(180)), Math.toRadians(mirrorHeading(180)))

                    .build();

            Path11 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(9.794, 35.888).mirror(),

                                    new Pose(59.140, 84.224).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(180)), Math.toRadians(mirrorHeading(135)))

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
                setPathState(States.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()) {
                    shootingManager.shoot();
                    setPathState(States.SHOOT_PRELOAD_TO_INTAKE_LINE1);
                }
                break;

            case SHOOT_PRELOAD_TO_INTAKE_LINE1:
                if (!shootingManager.isBusy()) {
                    follower.followPath(paths.Path2);
                    setPathState(States.INTAKE_LINE1_TO_FINISHED_INTAKE_LINE1);
                }
                break;

            case INTAKE_LINE1_TO_FINISHED_INTAKE_LINE1:
                if (!follower.isBusy()) {
                    intakingManager.togglePull();
                    follower.followPath(paths.Path3, 0.4, false);
                    setPathState(States.FINISHED_INTAKED_LINE1_TO_OPEN_GATE);
                }
                break;

            case FINISHED_INTAKED_LINE1_TO_OPEN_GATE:
                if (!follower.isBusy()) {
                    intakingManager.togglePull();
                    follower.followPath(paths.Path4, 0.4, false);
                    setPathState(States.OPEN_GATE_TO_SHOOT_LINE1);
                }
                break;

            case OPEN_GATE_TO_SHOOT_LINE1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    setPathState(States.SHOOT_LINE1);
                }

            case SHOOT_LINE1:
                if (!follower.isBusy()) {
                    shootingManager.shoot();
                    setPathState(States.SHOOT_LINE1_TO_INTAKE_LINE2);
                }
                break;

            case SHOOT_LINE1_TO_INTAKE_LINE2:
                if (!shootingManager.isBusy()) {
                    follower.followPath(paths.Path6);
                    setPathState(States.INTAKE_LINE2_TO_FINISHED_INTAKE_LINE2);
                }
                break;

            case INTAKE_LINE2_TO_FINISHED_INTAKE_LINE2:
                if (!follower.isBusy()) {
                    intakingManager.togglePull();
                    follower.followPath(paths.Path7, 0.4, false);
                    setPathState(States.FINISHED_INTAKE_LINE2_TO_SHOOT_LINE2);
                }

            case FINISHED_INTAKE_LINE2_TO_SHOOT_LINE2:
                if (!follower.isBusy()) {
                    intakingManager.togglePull();
                    follower.followPath(paths.Path8);
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
                if (!shootingManager.isBusy()) {
                    follower.followPath(paths.Path9);
                    setPathState(States.INTAKE_LINE3_TO_FINISHED_INTAKE_LINE3);
                }
                break;

            case INTAKE_LINE3_TO_FINISHED_INTAKE_LINE3:
                if (!follower.isBusy()) {
                    intakingManager.togglePull();
                    follower.followPath(paths.Path10, 0.4, false);
                    setPathState(States.FINISHED_INTAKE_LINE3_TO_SHOOT_LINE3);
                }

            case FINISHED_INTAKE_LINE3_TO_SHOOT_LINE3:
                if (!follower.isBusy()) {
                    intakingManager.togglePull();
                    follower.followPath(paths.Path11);
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
                        20.336,
                        122.617,
                        Math.toRadians(-36)
                ).mirror()
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
                follower.getPose().distanceFrom(Poses.redGoalPose),
                timer.seconds(),
                follower.poseTracker.getVelocity(),
                Math.atan2((Poses.redGoalPose.getY() - follower.getPose().getY()), (Poses.redGoalPose.getX() - follower.getPose().getX()))
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
        json.addProperty("color", AllianceColor.RED.toString());

        gson = new Gson();
        file = AppUtil.getInstance().getSettingsFile("RobotSettings.json");

        try (FileWriter writer = new FileWriter(file)) {
            gson.toJson(json, writer);
        } catch (IOException ignored) {}

        telemetry.addLine("Start Code #24037");
        telemetry.update();
    }
}