package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MethodHolder {
    private DcMotor left_front, right_front, left_rear, right_rear, xOdometry, yOdometry;

    private double Distance, Direction;

    public MethodHolder(DcMotor left_front, DcMotor right_front, DcMotor left_rear, DcMotor right_rear, DcMotor xOdometry, DcMotor yOdometry,
                        double Distance, double Direction) {

        this.left_front = left_front;
        this.right_front = right_front;
        this.left_rear = left_rear;
        this.right_rear = right_rear;

        this.xOdometry = xOdometry;
        this.yOdometry = yOdometry;

        this.Distance = Distance;
        this.Direction = Direction;
    }

    public void update() {

        double Distance2 = Distance * 505.316944317;

        double currentDistance = yOdometry.getCurrentPosition();
        double currentDirection = (xOdometry.getCurrentPosition() - yOdometry.getCurrentPosition()) / 1000.0;

        double distanceError = Distance2 - currentDistance;
        double headingError = Direction - currentDirection;

        double kpDistance = 0.005;
        double kpHeading = 0.01;

        double forward = distanceError * kpDistance;
        double turn = headingError * kpHeading;

        turn = Math.max(-0.3, Math.min(0.3, turn));
        forward = Math.max(-0.5, Math.min(0.5, forward));

        double leftFrontPower = forward + turn;
        double leftRearPower = forward + turn;
        double rightFrontPower = forward - turn;
        double rightRearPower = forward - turn;

        left_front.setPower(leftFrontPower);
        right_front.setPower(rightFrontPower);
        left_rear.setPower(leftRearPower);
        right_rear.setPower(rightRearPower);
    }
}

