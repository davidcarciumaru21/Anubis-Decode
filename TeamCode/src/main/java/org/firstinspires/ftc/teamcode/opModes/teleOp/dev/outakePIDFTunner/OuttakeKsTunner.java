package org.firstinspires.ftc.teamcode.opModes.teleOp.dev.outakePIDFTunner;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.systems.Outtake;

@TeleOp(name = "OuttakeKsTuner", group = "Dev")
public class OuttakeKsTunner extends OpMode {

    private Outtake outtake;
    private double power = 0.0;
    private VoltageSensor voltageSensor;

    @Override
    public void init() {
        outtake = new Outtake(hardwareMap);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void loop() {

        if (gamepad1.dpadUpWasPressed()) {
            power += 0.01;
        }

        if (gamepad1.dpadDownWasPressed()) {
            power -= 0.01;
        }

        power = Math.max(0.0, Math.min(power, 1.0));

        outtake.setPower(power);

        double batteryVoltage = voltageSensor.getVoltage();
        double ksVolts = power * batteryVoltage;

        telemetry.addData("Ks", ksVolts);
        telemetry.update();
    }
}
