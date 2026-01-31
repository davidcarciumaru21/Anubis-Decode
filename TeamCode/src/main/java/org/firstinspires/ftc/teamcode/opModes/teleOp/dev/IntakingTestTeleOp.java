package org.firstinspires.ftc.teamcode.opModes.teleOp.dev;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.systems.Indexer;
import org.firstinspires.ftc.teamcode.systems.Intake;

@TeleOp(name = "IntakingTestTeleOp", group = "Dev")
public class IntakingTestTeleOp extends OpMode {

    private Intake intake;
    private Indexer indexer;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
    }

    @Override
    public void loop() {
        intake.pull();
        indexer.pull();
    }
}
