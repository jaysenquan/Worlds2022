package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Commands {
    HardwareMap hardwareMap;
    private Robot robot;
    private BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .5, correction, rotation;
    private int turnError = 8;
    private double liftTime = 0, lowerTime = 0;
    PIDController pidRotate, pidDrive;
    private static final double COUNTS_PER_MOTOR_REV = 537.7; //Ticks per rotation for the GoBilda 5202 PLanetary Motor
    private static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_INCHES = 3.54331;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    public Commands(Robot r) {
        robot = r;
    }
    public void init(HardwareMap hwMap) {
        hardwareMap = hwMap;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        pidRotate = new PIDController(.003, .00003, 0);
        pidDrive = new PIDController(.065, 0, 0);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void moveToPosition(double inches, double power, double direction, boolean lift, boolean lower) {
        double targetPosition = inches*COUNTS_PER_INCH;
        robot.callibrateRunToPosition();
        double currentTime = System.currentTimeMillis();
        if (direction == 1) {
            int frontLeftOriginalTarget = robot.frontLeft.getTargetPosition();
            int rearLeftOriginalTarget = robot.rearLeft.getTargetPosition();
            int frontRightOriginalTarget = robot.frontRight.getTargetPosition();
            int rearRightOriginalTarget = robot.rearRight.getTargetPosition();
            robot.addToPosition((int)targetPosition, (int)targetPosition, (int)targetPosition, (int)targetPosition);
            while(robot.motorsAreBusy()) {
                int frontLeftCurrentPosition = robot.frontLeft.getCurrentPosition();
                int rearLeftCurrentPosition = robot.rearLeft.getCurrentPosition();
                int frontRightCurrentPosition = robot.frontRight.getCurrentPosition();
                int rearRightCurrentPosition = robot.rearRight.getCurrentPosition();

                if(Math.abs(frontLeftCurrentPosition-frontLeftOriginalTarget) < targetPosition ||
                        Math.abs(rearLeftCurrentPosition-rearLeftOriginalTarget) < targetPosition ||
                        Math.abs(frontRightCurrentPosition-frontRightOriginalTarget) < targetPosition ||
                        Math.abs(rearRightCurrentPosition-rearRightOriginalTarget) < targetPosition) {
                    if(System.currentTimeMillis() - currentTime <= liftTime && lift == true) {
                        robot.liftMotor.setPower(1);
                    } else if (System.currentTimeMillis() - currentTime <= lowerTime && lower == true) {
                        robot.liftMotor.setPower(-1);
                    } else {
                        robot.liftMotor.setPower(0);
                    }
                    correction = pidDrive.performPID(getAngle());
                    robot.frontLeft.setPower(power-correction);
                    robot.rearLeft.setPower(power-correction);
                    robot.frontRight.setPower(power+correction);
                    robot.rearRight.setPower(power+correction);
                } else {
                    robot.liftMotor.setPower(0);
                    robot.setMotorPower(0);
                }
            }
        } else if (direction == -1) {
            int frontLeftOriginalTarget = robot.frontLeft.getTargetPosition();
            int rearLeftOriginalTarget = robot.rearLeft.getTargetPosition();
            int frontRightOriginalTarget = robot.frontRight.getTargetPosition();
            int rearRightOriginalTarget = robot.rearRight.getTargetPosition();
            robot.addToPosition((int)-targetPosition, (int)-targetPosition, (int)-targetPosition, (int)-targetPosition);

            while(robot.motorsAreBusy()) {
                int frontLeftCurrentPosition = robot.frontLeft.getCurrentPosition();
                int rearLeftCurrentPosition = robot.rearLeft.getCurrentPosition();
                int frontRightCurrentPosition = robot.frontRight.getCurrentPosition();
                int rearRightCurrentPosition = robot.rearRight.getCurrentPosition();

                if(Math.abs(frontLeftCurrentPosition-frontLeftOriginalTarget) < targetPosition ||
                        Math.abs(rearLeftCurrentPosition-rearLeftOriginalTarget) < targetPosition ||
                        Math.abs(frontRightCurrentPosition-frontRightOriginalTarget) < targetPosition ||
                        Math.abs(rearRightCurrentPosition-rearRightOriginalTarget) < targetPosition) {
                    if(System.currentTimeMillis() - currentTime <= liftTime && lift == true) {
                        robot.liftMotor.setPower(1);
                    } else if (System.currentTimeMillis() - currentTime <= lowerTime && lower == true) {
                        robot.liftMotor.setPower(-1);
                    } else {
                        robot.liftMotor.setPower(0);
                    }
                    correction = pidDrive.performPID(getAngle());
                    robot.frontLeft.setPower(-power-correction);
                    robot.rearLeft.setPower(-power-correction);
                    robot.frontRight.setPower(-power+correction);
                    robot.rearRight.setPower(-power+correction);
                } else {
                    robot.setMotorPower(0);
                    robot.liftMotor.setPower(0);
                }
            }
        }
        correction = 0;
    }
    // + = left, - = right
    public void strafeToPosition(double inches, double power, double direction) {
        double targetPosition = inches * COUNTS_PER_INCH;
        robot.callibrateRunToPosition();

        if(direction == 1) {
            int frontLeftOriginalTarget = robot.frontLeft.getTargetPosition();
            int rearLeftOriginalTarget = robot.rearLeft.getTargetPosition();
            int frontRightOriginalTarget = robot.frontRight.getTargetPosition();
            int rearRightOriginalTarget = robot.rearRight.getTargetPosition();
            robot.addToPosition((int)targetPosition*-1, (int)targetPosition, (int)targetPosition, (int)targetPosition *-1);

            while(robot.motorsAreBusy()) {
                int frontLeftCurrentPosition = robot.frontLeft.getCurrentPosition();
                int rearLeftCurrentPosition = robot.rearLeft.getCurrentPosition();
                int frontRightCurrentPosition = robot.frontRight.getCurrentPosition();
                int rearRightCurrentPosition = robot.rearRight.getCurrentPosition();

                if(Math.abs(frontLeftCurrentPosition-frontLeftOriginalTarget) < targetPosition ||
                        Math.abs(rearLeftCurrentPosition-rearLeftOriginalTarget) < targetPosition ||
                        Math.abs(frontRightCurrentPosition-frontRightOriginalTarget) < targetPosition ||
                        Math.abs(rearRightCurrentPosition-rearRightOriginalTarget) < targetPosition) {
                    robot.frontLeft.setPower(-power);
                    robot.rearLeft.setPower(power);
                    robot.frontRight.setPower(power);
                    robot.rearRight.setPower(-power);
                } else {
                    robot.setMotorPower(0);
                }
            }
        } else if (direction == -1) {
            int frontLeftOriginalTarget = robot.frontLeft.getTargetPosition();
            int rearLeftOriginalTarget = robot.rearLeft.getTargetPosition();
            int frontRightOriginalTarget = robot.frontRight.getTargetPosition();
            int rearRightOriginalTarget = robot.rearRight.getTargetPosition();
            robot.addToPosition((int)targetPosition, (int)targetPosition*-1, (int)targetPosition*-1, (int)targetPosition);

            while(robot.motorsAreBusy()) {
                int frontLeftCurrentPosition = robot.frontLeft.getCurrentPosition();
                int rearLeftCurrentPosition = robot.rearLeft.getCurrentPosition();
                int frontRightCurrentPosition = robot.frontRight.getCurrentPosition();
                int rearRightCurrentPosition = robot.rearRight.getCurrentPosition();

                if(Math.abs(frontLeftCurrentPosition-frontLeftOriginalTarget) < targetPosition ||
                        Math.abs(rearLeftCurrentPosition-rearLeftOriginalTarget) < targetPosition ||
                        Math.abs(frontRightCurrentPosition-frontRightOriginalTarget) < targetPosition ||
                        Math.abs(rearRightCurrentPosition-rearRightOriginalTarget) < targetPosition) {
                    robot.frontLeft.setPower(power);
                    robot.rearLeft.setPower(-power);
                    robot.frontRight.setPower(-power);
                    robot.rearRight.setPower(power);
                } else {
                    robot.setMotorPower(0);
                }
            }
        }
    }
    public void intake() {
        double currentTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - currentTime <= 1000) {
            robot.intakeMotor.setPower(-1);
            robot.setMotorPower(.1);
        }
        robot.intakeMotor.setPower(0);
    }
    public void sleep(double time) {
        double currentTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - currentTime <= time) {
            //do nothing
        }
        return;
    }
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }
    public void turnCarousel(int side) {
        //red = 0, blue = 1
        if(side == 0) {
            robot.carouselServo.setPower(-1);
        } else if (side == 1) {
            robot.carouselServo.setPower(1);
        }
        sleep(7500);
        robot.carouselServo.setPower(0);
    }
    public void lowClose() {
        robot.liftMotor.setPower(.5);
        sleep(250);
        robot.output.setPosition(0);
        sleep(250);
        robot.liftMotor.setPower(-1);
        sleep(700);
    }
    public void setLiftTime(double time) {
        liftTime = time;
    }
    public void setLowerTime(double time) {
        lowerTime = time;
    }
    /*
    private void liftSlide(int height) {
        double currentTime = System.currentTimeMillis();
        if (height == 0) {
            while(System.currentTimeMillis() - currentTime <= 3000) {
                robot.liftMotor.setPower(1);
            }
            robot.liftMotor.setPower(0);
        }
        else if (height == 1) {
            while(getRuntime() - currentTime <= 1100) {
                robot.liftMotor.setPower(1);
            }
            robot.liftMotor.setPower(0);
        }
        else if (height == 2) {
            while(getRuntime() - currentTime <= 500) {
                robot.liftMotor.setPower(1);
            }
            robot.liftMotor.setPower(0);
        }
    }
    private void lowerSlide(int height) {
        double currentTime = getRuntime();
        if(height == 0) {
            while(getRuntime() - currentTime <= 2000) {
                robot.liftMotor.setPower(1);
            }
            robot.liftMotor.setPower(0);
        } else if (height == 1) {
            while(getRuntime() - currentTime <= 1000) {
                robot.liftMotor.setPower(-1);
            }
            robot.liftMotor.setPower(0);
        } else if (height == 2) {
            while(getRuntime() - currentTime <= 500) {
                robot.liftMotor.setPower(-1);
            }
            robot.liftMotor.setPower(0);
        }
    }
     */
    public void close() {
        robot.output.setPosition(0);
        sleep(500);
    }
    public void release() {
        robot.output.setPosition(1);
        sleep(1000);
    }
}
