package org.firstinspires.ftc.teamcode.opModes.teleOp.use;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Deflector;
import org.firstinspires.ftc.teamcode.systems.Outtake;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Indexer;

@TeleOp(name = "ManualRobotTeleOp", group = "use")
public class ManualRobotTeleOp extends OpMode {

    private TelemetryManager panelsTelemetry;

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private double x = 0.0, y = 0.0, rx = 0.0;
    double frontLeftPower = 0.0, backLeftPower = 0.0, frontRightPower = 0.0, backRightPower = 0.0;
    private double denominator = 1.0;

    private Outtake outtake;
    private Intake intake;
    private Indexer indexer;
    private Deflector deflector;

    private double rpm = 0.12, pose = 0.0;

    private ElapsedTime timer;

    private double speedCoefficient = 1.0;

    @Override
    public void init() {
        timer = new ElapsedTime();

        frontLeftMotor = hardwareMap.dcMotor.get("FrontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("BackLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("FrontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("BackRightMotor");

        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        deflector = new Deflector(hardwareMap);


        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {

        if (gamepad1.right_trigger >= 0.3) speedCoefficient = 0.5;
        else if (gamepad1.left_trigger >= 0.3) speedCoefficient = 0.25;
        else speedCoefficient = 1.0;

        x = gamepad1.left_stick_x * speedCoefficient;
        y = -gamepad1.left_stick_y  * speedCoefficient;
        rx = gamepad1.right_stick_x  * speedCoefficient;

        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = (y + x + rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        if (gamepad1.dpadUpWasPressed()) rpm += 50;
        if (gamepad1.dpadDownWasPressed()) rpm -= 50;

        if (gamepad1.dpadRightWasPressed()) pose += 0.01;
        if (gamepad1.dpadLeftWasPressed()) pose -= 0.01;

        outtake.moveFlyWheelAtRPM(rpm);
        outtake.update(timer.milliseconds());
        deflector.move(pose);
        intake.pull();
        if (gamepad1.bWasPressed()) indexer.off();
        if (gamepad1.aWasPressed()) indexer.pull();
        timer.reset();

        telemetry.addData("rpm", rpm);
        telemetry.addData("pose", pose);
        telemetry.addData("outtake speed", outtake.getRPM());
        telemetry.update();

        panelsTelemetry.addData("current rpm", outtake.getRPM());;
        panelsTelemetry.addData("target rpm", rpm);
        panelsTelemetry.addData("power", outtake.getPower());
        panelsTelemetry.update();
    }
}