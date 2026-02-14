package org.firstinspires.ftc.teamcode.opModes.auto;
/*
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.systems.Deflector;
import org.firstinspires.ftc.teamcode.systems.Outtake;
import org.firstinspires.ftc.teamcode.systems.Indexer;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.managers.IntakingManager;
import org.firstinspires.ftc.teamcode.managers.ShootingManager;

@Disabled
@Autonomous(name = "BlueGoalAuto", group = "Blue")
public class BlueGoalAuto extends OpMode {

    private enum States {
        START_TO_SHOOT_PRELOAD,
        SHOOT_PRELOAD,
        SHOOT_PRELAOD_TO_INTAKE_LINE1,
        INTAKE_LINE1_TO_FINISHED_INTAKE_LINE1,
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
                                    new Pose(18.393, 120.000),

                                    new Pose(48.374, 95.215)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-37), Math.toRadians(-45))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.374, 95.215),
                                    new Pose(63.294, 83.500),
                                    new Pose(43.056, 84.271)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(43.056, 84.271),

                                    new Pose(17.701, 83.813)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.701, 83.813),

                                    new Pose(58.813, 84.271)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58.813, 84.271),
                                    new Pose(61.874, 58.836),
                                    new Pose(41.346, 60.598)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.346, 60.598),

                                    new Pose(17.579, 60.103)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.579, 60.103),

                                    new Pose(59.009, 84.336)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(59.009, 84.336),
                                    new Pose(57.472, 30.271),
                                    new Pose(41.598, 35.888)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(180))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(41.598, 35.888),

                                    new Pose(17.822, 35.991)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            Path10 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(17.822, 35.991),

                                    new Pose(58.991, 83.664)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-45))

                    .build();
        }

}

    private void setPathState(States newState) {
        state = newState;
    }

    private void run() {
        switch (state) {

            case START:
                follower.followPath(paths.Path1);
                setPathState(States.END);
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
        telemetry.addLine("Started auto");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        run();
    }

    @Override
    public void stop() {
        telemetry.addLine("Start Code #24037");
        telemetry.update();
    }
}
*/