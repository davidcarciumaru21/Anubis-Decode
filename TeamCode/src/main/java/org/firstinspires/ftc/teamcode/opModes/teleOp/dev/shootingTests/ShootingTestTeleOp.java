package org.firstinspires.ftc.teamcode.opModes.teleOp.dev.shootingTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Deflector;
import org.firstinspires.ftc.teamcode.systems.Indexer;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Outtake;

@TeleOp(name = "ShootingTestTeleOp", group = "Dev")
public class ShootingTestTeleOp extends OpMode {

    private Outtake outtake;
    private Deflector deflector;
    private Intake intake;
    private Indexer indexer;

    private ElapsedTime timer;

    private double rpm = 0, pose = 0.12;

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

        if (gamepad1.dpadRightWasPressed()) pose += 0.01;
        if (gamepad1.dpadLeftWasPressed()) pose -= 0.01;

        outtake.moveFlyWheelAtRPM(rpm);
        outtake.update(timer.seconds());
        deflector.move(pose);
        intake.pull();
        if (gamepad1.bWasPressed()) indexer.off();
        if (gamepad1.aWasPressed()) indexer.pull();
        timer.reset();

        telemetry.addData("rpm", rpm);
        telemetry.addData("pose", pose);
    }
}
