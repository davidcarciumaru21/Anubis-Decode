package org.firstinspires.ftc.teamcode.opModes.teleOp.dev;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.global.DrivingType;
import org.firstinspires.ftc.teamcode.global.GamepadsSettings;
import org.firstinspires.ftc.teamcode.global.Poses;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.systems.Deflector;
import org.firstinspires.ftc.teamcode.systems.Indexer;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Outtake;

@TeleOp(name = "CalibrationTeleOp", group = "dev" )
public class CalibrationTeleOp extends OpMode {

    private boolean indexerActive = true;
    private Intake intake;
    private Indexer indexer;
    private Deflector deflector;
    private Outtake outtake;
    private ElapsedTime timer;
    private Follower follower;

    private double forward = 0.0, strafe = 0.0, rotation = 0.0;
    private DrivingType drivingTypeGm1 = DrivingType.ROBOT_CENTRIC;
    private double gamepad1Coef = 1.0;

    private double targetRPM = 0.0;
    private double targetPose = 0.0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(18.392523364485978, 120, Math.toRadians(-37)));
        follower.update();

        intake = new Intake(hardwareMap);
        deflector = new Deflector(hardwareMap);
        outtake = new Outtake(hardwareMap);
        indexer = new Indexer(hardwareMap);

        timer = new ElapsedTime();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
        timer.reset();
        deflector.init(0.0);
    }

    @Override
    public void loop() {
        if (gamepad1.shareWasPressed()) {
            drivingTypeGm1 = drivingTypeGm1 == DrivingType.ROBOT_CENTRIC
                    ? DrivingType.FIELD_CENTRIC
                    : DrivingType.ROBOT_CENTRIC;
        }

        if (gamepad1.right_trigger > 0.1) gamepad1Coef = GamepadsSettings.slowCoefGm1;
        else if (gamepad1.left_trigger > 0.1) gamepad1Coef = GamepadsSettings.verySlowCoefGm1;
        else gamepad1Coef = 1.0;

        forward = -gamepad1.left_stick_y * gamepad1Coef;
        strafe = -gamepad1.left_stick_x * gamepad1Coef;
        rotation = -gamepad1.right_stick_x * gamepad1Coef;

        if (drivingTypeGm1 == DrivingType.ROBOT_CENTRIC) {
            follower.setTeleOpDrive(
                    forward,
                    strafe,
                    rotation,
                    true
                );
        } else {
            follower.setTeleOpDrive(
                    forward,
                    strafe,
                    rotation,
                    false
                );
        }

        if (gamepad1.dpadUpWasPressed()) targetRPM += 50;
        if (gamepad1.dpadDownWasPressed()) targetRPM -= 50;

        if (gamepad1.dpadRightWasPressed()) targetPose += 0.01;
        if (gamepad1.dpadLeftWasPressed()) targetPose -= 0.01;

        if(gamepad1.xWasPressed()) indexerActive = !indexerActive;
        if(indexerActive){
            indexer.pull();
        }
        else{
            indexer.off();
        }
        outtake.moveFlyWheelAtRPM(targetRPM);
        outtake.update(timer.seconds());
        intake.pull();
        deflector.move(targetPose);
        indexer.pull();

        follower.update();
        timer.reset();

        telemetry.addData("TargetRPM", targetRPM);
        telemetry.addData("TargetPose", targetPose);
        telemetry.addData("Distance", follower.getPose().distanceFrom(Poses.blueGoalPose));
        follower.update();
        telemetry.update();
    }
}
