package org.firstinspires.ftc.teamcode.opModes.auto.blue.smallTriangle;

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

@Autonomous(name = "BlueSmallTriangleAuto1", group = "Blue")
public class BlueSmallTriangleAuto1 extends OpMode {

    private enum States {
        START_TO_SHOOT_PRELOAD,
        SHOOT_PRELOAD,
        SHOOT_PRELOAD_TO_INTAKE_HUMAN_PLAYER_BALLS,
        INTAKE_HUMAN_PLAYER_BALLS_TO_FINISHED_INTAKE_HUMAN_PLAYER_BALLS,
        FINISHED_INTAKE_HUMAN_PLAYER_BALLS_TO_SHOOT_HUMAN_PLAYER_BALLS,
        SHOOT_HUMAN_PLAYER_BALLS,
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

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(64.374, 9.645),

                                    new Pose(57.869, 23.869)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(110))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(57.869, 23.869),
                                    new Pose(32.902, 26.874),
                                    new Pose(15.729, 10.626)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.729, 10.626),

                                    new Pose(10.467, 10.626)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.467, 10.626),

                                    new Pose(57.869, 23.869)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(118))

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
                    setPathState(States.SHOOT_PRELOAD_TO_INTAKE_HUMAN_PLAYER_BALLS);
                }
                break;

            case SHOOT_PRELOAD_TO_INTAKE_HUMAN_PLAYER_BALLS:
                if (!shootingManager.isBusy()) {
                    follower.followPath(paths.Path2);
                    setPathState(States.INTAKE_HUMAN_PLAYER_BALLS_TO_FINISHED_INTAKE_HUMAN_PLAYER_BALLS);
                }
                break;

            case INTAKE_HUMAN_PLAYER_BALLS_TO_FINISHED_INTAKE_HUMAN_PLAYER_BALLS:
                if (!follower.isBusy()) {
                    intakingManager.togglePull();
                    follower.followPath(paths.Path3);
                    setPathState(States.FINISHED_INTAKE_HUMAN_PLAYER_BALLS_TO_SHOOT_HUMAN_PLAYER_BALLS);
                }
                break;

            case FINISHED_INTAKE_HUMAN_PLAYER_BALLS_TO_SHOOT_HUMAN_PLAYER_BALLS:
                if (!follower.isBusy()) {
                    intakingManager.togglePull();
                    follower.followPath(paths.Path4);
                    setPathState(States.SHOOT_HUMAN_PLAYER_BALLS);
                }
                break;

            case SHOOT_HUMAN_PLAYER_BALLS:
                if (!follower.isBusy()) {
                    shootingManager.shoot();
                    setPathState(States.SHOOT_PRELOAD_TO_INTAKE_HUMAN_PLAYER_BALLS);
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
                        64.374,
                        9.645,
                        Math.toRadians(90)
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