package org.firstinspires.ftc.teamcode.opModes.teleOp.use;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.global.DrivingType;
import org.firstinspires.ftc.teamcode.global.Poses;
import org.firstinspires.ftc.teamcode.managers.IntakingManager;
import org.firstinspires.ftc.teamcode.managers.ShootingManager;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.systems.Deflector;
import org.firstinspires.ftc.teamcode.systems.Indexer;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Outtake;
import org.firstinspires.ftc.teamcode.global.GamepadsSettings;

@TeleOp(name = "MainTeleOp", group = "use")
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

    private boolean gamepad1IsActive = false;
    private DrivingType drivingTypeGm1 = DrivingType.ROBOT_CENTRIC;
    private DrivingType drivingTypeGm2 = DrivingType.ROBOT_CENTRIC;
    private double gamepad1Coef = 1.0;
    private double gamepad2Coef = 1.0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(18.392523364485978, 120, Math.toRadians(-37)));
        follower.update();

        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        deflector = new Deflector(hardwareMap);
        outtake = new Outtake(hardwareMap);

        intakingManager = new IntakingManager(intake);
        shootingManager = new ShootingManager(outtake, indexer, deflector, intakingManager);

        timer = new ElapsedTime();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
        timer.reset();
    }

    @Override
    public void loop() {

        gamepad1IsActive = Math.abs(gamepad1.left_stick_y) > GamepadsSettings.gamepadTrashold ||
                Math.abs(gamepad1.left_stick_x) > GamepadsSettings.gamepadTrashold ||
                Math.abs(gamepad1.right_stick_x) > GamepadsSettings.gamepadTrashold;

        if (gamepad1.shareWasPressed()) {
            drivingTypeGm1 = drivingTypeGm1 == DrivingType.ROBOT_CENTRIC
                    ? DrivingType.FIELD_CENTRIC
                    : DrivingType.ROBOT_CENTRIC;
        }

        if (gamepad2.shareWasPressed()) {
            drivingTypeGm2 = drivingTypeGm2 == DrivingType.ROBOT_CENTRIC
                    ? DrivingType.FIELD_CENTRIC
                    : DrivingType.ROBOT_CENTRIC;
        }

        if (gamepad1.right_trigger > 0.1) gamepad1Coef = GamepadsSettings.slowCoefGm1;
        else if (gamepad1.left_trigger > 0.1) gamepad1Coef = GamepadsSettings.verySlowCoefGm1;
        else gamepad1Coef = 1.0;

        if (gamepad2.right_trigger > 0.1) gamepad2Coef = GamepadsSettings.slowCoefGm2;
        else if (gamepad2.left_trigger > 0.1) gamepad2Coef = GamepadsSettings.verySlowCoefGm2;
        else gamepad2Coef = 1.0;

        if (gamepad1IsActive) {
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
        } else {
            forward = gamepad2.left_stick_y * gamepad2Coef;
            strafe = gamepad2.left_stick_x * gamepad2Coef;
            rotation = gamepad2.right_stick_x * gamepad2Coef;

            if (drivingTypeGm2 == DrivingType.ROBOT_CENTRIC) {
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
        }

        if (gamepad1.rightBumperWasPressed()) shootingManager.shoot();
        if (gamepad1.xWasPressed()) intakingManager.togglePull();

        follower.update();
        shootingManager.update(follower.getPose().distanceFrom(Poses.blueGoalPose), timer.milliseconds());
        intakingManager.update();
        timer.reset();
    }

    @Override
    public void stop() {
        telemetry.addLine("Start Code #24037");
        telemetry.update();
    }
}
