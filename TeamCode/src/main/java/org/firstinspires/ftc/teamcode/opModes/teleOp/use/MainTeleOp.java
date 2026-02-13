package org.firstinspires.ftc.teamcode.opModes.teleOp.use;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.managers.IntakingManager;
import org.firstinspires.ftc.teamcode.managers.ShootingManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.systems.Deflector;
import org.firstinspires.ftc.teamcode.systems.Indexer;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Outtake;

public class MainTeleOp extends OpMode {

    private Intake intake;
    private Indexer indexer;
    private Deflector deflector;
    private Outtake outtake;

    private IntakingManager intakingManager;
    private ShootingManager shootingManager;

    private ElapsedTime timer;
    private Follower follower;

    private double forward = 0.0, strafe = 0.0, rotation = 0.0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(18.766355140186914, 36, Math.toRadians(-37));
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {

    }
}
