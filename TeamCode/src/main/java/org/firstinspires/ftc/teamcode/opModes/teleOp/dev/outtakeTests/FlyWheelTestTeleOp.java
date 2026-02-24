package org.firstinspires.ftc.teamcode.opModes.teleOp.dev.outtakeTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Deflector;
import org.firstinspires.ftc.teamcode.systems.Indexer;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Outtake;

@TeleOp(name = "FlyWheelTestTeleOp", group = "Dev")
public class FlyWheelTestTeleOp extends OpMode {

    private Outtake outtake;
    private Deflector deflector;
    private Intake intake;
    private Indexer indexer;

    private ElapsedTime timer;

    private double rpm = 0, angle = 40;

    @Override
    public void init() {
        timer = new ElapsedTime();

        outtake = new Outtake(hardwareMap);
        deflector = new Deflector(hardwareMap);
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadUpWasPressed()) rpm += 50;
        if (gamepad1.dpadDownWasPressed()) rpm -= 50;

        if (gamepad1.dpadRightWasPressed()) angle += 1;
        if (gamepad1.dpadLeftWasPressed()) angle -= 1;

        outtake.moveFlyWheelAtRPM(rpm);
        outtake.update(timer.seconds());
        deflector.moveAtAngleInDegrees(angle);
        intake.pull();
        if (gamepad1.bWasPressed()) indexer.off();
        if (gamepad1.aWasPressed()) indexer.pull();
        timer.reset();

        telemetry.addData("rpm", rpm);
        telemetry.addData("pose", angle);
    }
}
