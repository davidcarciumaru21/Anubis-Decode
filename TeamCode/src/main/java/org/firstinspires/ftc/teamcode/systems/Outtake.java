package org.firstinspires.ftc.teamcode.systems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class Outtake {

    private final DcMotorEx outtakeMotor1;
    private final DcMotorEx outtakeMotor2;
    private final VoltageSensor voltageSensor;

    private static final double TICKS_PER_REV = 28.0;

    public static double kS = 0.92;
    public static double kV = 0.0021;
    public static double kP = 0.23;
    public static double kI = 0;

    public static double MAX_ACCEL_RPM_PER_SEC = 3000;
    public static double I_ENABLE_ERROR = 0;

    private double targetRPM = 0;
    private double rampedRPM = 0;
    private double integral = 0;

    private double filteredVoltage = 13.0;
    private static final double alpha = 0.01;

    public Outtake(HardwareMap hardwareMap) {
        outtakeMotor1  = hardwareMap.get(DcMotorEx.class, "OuttakeMotor1");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "OuttakeMotor2");

        outtakeMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        outtakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        outtakeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtakeMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void move(double rpm) {
        targetRPM = rpm;
    }

    public void stop() {
        targetRPM = 0;
        rampedRPM = 0;
        integral = 0;
        setPower(0);
    }

    public double getRPM() {
        double ticksPerSec1 = outtakeMotor1.getVelocity();
        double ticksPerSec2 = outtakeMotor2.getVelocity();
        return ((ticksPerSec1 + ticksPerSec2) / 2.0 * 60.0) / TICKS_PER_REV;
    }

    public double getPower() {
        return outtakeMotor1.getPower();
    }

    public void setPower(double power) {
        outtakeMotor1.setPower(power);
        outtakeMotor2.setPower(power);
    }

    public void update(double deltaTime) {

        double currentRPM = getRPM();

        double maxStep = MAX_ACCEL_RPM_PER_SEC * deltaTime;
        double diff = targetRPM - rampedRPM;
        rampedRPM += Range.clip(diff, -maxStep, maxStep);

        double ff = kS * Math.signum(rampedRPM)
                + kV * rampedRPM;

        double error = rampedRPM - currentRPM;
        double p = kP * error;

        if (Math.abs(error) < I_ENABLE_ERROR) {
            integral += error * deltaTime;
        }
        double i = kI * integral;

        double measuredVoltage = voltageSensor.getVoltage();
        filteredVoltage += alpha * (measuredVoltage - filteredVoltage);

        double power = (ff + p + i) / filteredVoltage;
        power = Range.clip(power, 0.0, 1.0);

        setPower(power);
    }

    public void changePI(double p, double i) {
        kP = p;
        kI = i;
    }

    public void changeKs(double s) {
        kS = s;
    }

    public void changeKv(double v) {
        kV = v;
    }

    public double getKs() {
        return kS;
    }

    public double getKv() {
        return kV;
    }
}