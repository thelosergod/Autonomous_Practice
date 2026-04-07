package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MethodHolder {
    private DcMotor lf, rf, lr, rr;
    private Pinpoint pinpoint;
    private double targetX;
    private double targetY;
    private double targetHeading;
    private boolean turningOnly = false;

    public MethodHolder(DcMotor lf, DcMotor rf, DcMotor lr, DcMotor rr,
                        Pinpoint pinpoint) {
        this.lf = lf;
        this.rf = rf;
        this.lr = lr;
        this.rr = rr;
        this.pinpoint = pinpoint;
        targetX = pinpoint.getX();
        targetY = pinpoint.getY();
        targetHeading = pinpoint.getHeading();
    }

    // =========================
    // COMMANDS
    // =========================
    public void moveForward(double inches) {
        targetX = pinpoint.getX();
        targetY = pinpoint.getY() + inches;
        targetHeading = pinpoint.getHeading();
        turningOnly = false;
    }

    public void moveBackward(double inches) {
        moveForward(-inches);
    }

    public void strafeRight(double inches) {
        targetX = pinpoint.getX() + inches;
        targetY = pinpoint.getY();
        targetHeading = pinpoint.getHeading();
        turningOnly = false;
    }

    public void strafeLeft(double inches) {
        strafeRight(-inches);
    }

    public void turnRight(double radians) {
        targetHeading = pinpoint.getHeading() + radians;
        turningOnly = true;
    }

    public void turnLeft(double radians) {
        turnRight(-radians);
    }

    // =========================
    // IS BUSY CHECK
    // =========================
    public boolean isBusy() {
        double errorX = targetX - pinpoint.getX();
        double errorY = targetY - pinpoint.getY();
        double errorHeading = angleWrap(targetHeading - pinpoint.getHeading());

        if (turningOnly) {
            return Math.abs(errorHeading) > 0.05; // ~3 degrees tolerance
        }
        return Math.abs(errorX) > 0.5 || Math.abs(errorY) > 0.5; // ~0.5 inch tolerance
    }

    // =========================
    // UPDATE LOOP
    // =========================
    public void update() {
        double currentX = pinpoint.getX();
        double currentY = pinpoint.getY();
        double currentHeading = pinpoint.getHeading();

        double errorX = targetX - currentX;
        double errorY = targetY - currentY;
        double errorHeading = angleWrap(targetHeading - currentHeading);

        double kpXY = 0.05;
        double kpTurn = 0.8;

        double strafe = clamp(errorX * kpXY, -0.6, 0.6);
        double forward = clamp(errorY * kpXY, -0.6, 0.6);
        double turn = clamp(errorHeading * kpTurn, -0.5, 0.5);

        if (turningOnly) {
            forward = 0;
            strafe = 0;
        }

        double lfPower = forward + strafe + turn;
        double rfPower = forward - strafe - turn;
        double lrPower = forward - strafe + turn;
        double rrPower = forward + strafe - turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(lfPower),
                Math.max(Math.abs(rfPower),
                Math.max(Math.abs(lrPower), Math.abs(rrPower)))));

        lf.setPower(lfPower / max);
        rf.setPower(rfPower / max);
        lr.setPower(lrPower / max);
        rr.setPower(rrPower / max);
    }

    // =========================
    // STOP
    // =========================
    public void stop() {
        lf.setPower(0);
        rf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);
    }

    // =========================
    // HELPERS
    // =========================
    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
