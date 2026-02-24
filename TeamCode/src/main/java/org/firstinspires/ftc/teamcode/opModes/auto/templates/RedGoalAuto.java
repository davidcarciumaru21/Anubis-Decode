package org.firstinspires.ftc.teamcode.opModes.auto.templates;

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

@Autonomous(name = "RedGoalAuto", group = "Blue")
public class RedGoalAuto extends OpMode {

    private enum States {
        START_TO_SHOOT_PRELOAD,
        FIRST_WAIT,
        SHOOT_PRELOAD,
        SHOOT_PRELAOD_TO_INTAKE_LINE1,
        INTAKE_LINE1_TO_FINISHED_INTAKE_LINE1,
        SECOND_WAIT,
        FINISHED_INTAKE_LINE1_TO_SHOOT_LINE1,
        SHOOT,
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

    private JsonObject json;
    private Gson gson;
    private File file;

    private ElapsedTime secondTimer;

    private static double mirrorHeading(double deg) {
        return (180 - deg + 360);
    }

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

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18.393, 120.000).mirror(),

                                    new Pose(48.374, 95.215).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(-37)), Math.toRadians(mirrorHeading(143)))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.374, 95.215).mirror(),
                                    new Pose(63.294, 83.500).mirror(),
                                    new Pose(43.056, 84.271).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(143)), Math.toRadians(mirrorHeading(180)))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.056, 84.271).mirror(),

                                    new Pose(17.701, 83.813).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(180)), Math.toRadians(mirrorHeading(180)))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.701, 83.813).mirror(),

                                    new Pose(58.813, 84.271).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(180)), Math.toRadians(mirrorHeading(135)))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.813, 84.271).mirror(),
                                    new Pose(61.874, 58.836).mirror(),
                                    new Pose(41.346, 60.598).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(135)), Math.toRadians(mirrorHeading(180)))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.346, 60.598).mirror(),

                                    new Pose(17.579, 60.103).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(180)), Math.toRadians(mirrorHeading(180)))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.579, 60.103).mirror(),

                                    new Pose(59.009, 84.336).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(180)), Math.toRadians(mirrorHeading(135)))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.009, 84.336).mirror(),
                                    new Pose(57.472, 30.271).mirror(),
                                    new Pose(41.598, 35.888).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(135)), Math.toRadians(mirrorHeading(180)))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.598, 35.888).mirror(),

                                    new Pose(17.822, 35.991).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(180)), Math.toRadians(mirrorHeading(180)))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.822, 35.991).mirror(),

                                    new Pose(58.991, 83.664).mirror()
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(mirrorHeading(180)), Math.toRadians(mirrorHeading(135)))

                    .build();
        }
    }

    private void setPathState(States newState) {
        state = newState;
        secondTimer.reset();
    }

    private void run() {
        switch (state) {

            case START_TO_SHOOT_PRELOAD:
                follower.followPath(paths.Path1);
                setPathState(States.FIRST_WAIT);
                break;

            case FIRST_WAIT:
                if (secondTimer.milliseconds() > 2000) {
                    setPathState(States.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
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
                        125.8,
                        120,
                        Math.toRadians(-143)
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
        secondTimer.reset();

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
                Math.atan((Poses.redGoalPose.getX() - follower.getPose().getX()) / (Poses.redGoalPose.getY() - follower.getPose().getY()))
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