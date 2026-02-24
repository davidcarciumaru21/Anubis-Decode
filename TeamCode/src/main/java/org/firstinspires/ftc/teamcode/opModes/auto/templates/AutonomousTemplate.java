package org.firstinspires.ftc.teamcode.opModes.auto.templates;

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
@Autonomous(name = "Autonomous template", group = "Green")
public class AutonomousTemplate extends OpMode {

    private enum States {
        START,
        END
    }

    private States state = States.START;

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

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(25.570, 127.850),
                                    new Pose(53.383, 100.262)
                            )
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(143),
                            Math.toRadians(143)
                    )
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
