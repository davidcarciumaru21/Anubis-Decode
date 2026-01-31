package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

public class Outtake {

    private final DcMotorEx outtake;
    private final VoltageSensor voltageSensor;

    private static final double TICKS_PER_REV = 28.0;

    public static double kS = 0.618;

    public static double kV = 0.0026;
    public static double kP = 0.01;
    public static double kI = 0;

    public static double MAX_ACCEL_RPM_PER_SEC = 3000;
    public static double I_ENABLE_ERROR = 200;

    private double targetRPM = 0;
    private double rampedRPM = 0;
    private double integral = 0;

    private double filteredVoltage = 13.0;
    private static final double alpha = 0.01;

    public Outtake(HardwareMap hardwareMap) {
        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void move(double rpm) {
        targetRPM = rpm;
    }

    public void stop() {
        targetRPM = 0;
        rampedRPM = 0;
        integral = 0;
        outtake.setPower(0);
    }

    public double getRPM() {
        double ticksPerSec = outtake.getVelocity();
        return (ticksPerSec * 60.0) / TICKS_PER_REV;
    }

    public double getPower() {
        return outtake.getPower();
    }

    public void setPower(double power) {
        outtake.setPower(power);
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

        outtake.setPower(power);
    }

    public void changePI(double p, double i) {
        kP = p;
        kI = i;
    }
}