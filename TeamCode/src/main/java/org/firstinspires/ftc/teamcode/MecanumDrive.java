package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;

class MecanumDrive {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;

    DcMotor shooterFront;
    DcMotor shooterBack;
    DcMotor intake;
    DcMotor conveyor;

    Servo grabber;
    Servo wobbleArm;
    Servo lifter;
    Servo pusher;

    private double fast = 1.0; // Limit motor power to this value for Andymark RUN_USING_ENCODER mode
    private double medium = 0.7; // medium speed
    private double slow = 0.2; // slow speed

    public static double GEAR_RATIO = 1.0; // for simulator - ours should be 0.5f;
    public static double WHEEL_RADIUS = 5.0;  // 5 cm
    public static double TICKS_PER_ROTATION = 1120.0;  // From NeveRest (for simulator)  GoBilda should be 383.6f
    public static double CM_PER_TICK = (2 * Math.PI * GEAR_RATIO * WHEEL_RADIUS) / TICKS_PER_ROTATION;

    /** This is for encoder **/
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private double maxSpeed = 1.0;

    // drive motor position variables
    private int flPos; private int frPos; private int blPos; private int brPos;


    private MatrixF conversion;
    private GeneralMatrixF encoderMatrix = new GeneralMatrixF(3, 1);

    private int frontLeftOffset;
    private int frontRightOffset;
    private int backRightOffset;
    private int backLeftOffset;

    ElapsedTime runtime = new ElapsedTime();


    MecanumDrive() {

        float[] data = {1.0f, 1.0f, 1.0f,
                1.0f, -1.0f, -1.0f,
                1.0f, -1.0f, 1.0f};
        conversion = new GeneralMatrixF(3, 3, data);
        conversion = conversion.inverted();
    }

    void init(HardwareMap hwMap) {
        frontLeft = hwMap.get(DcMotor.class, "front_left_motor");
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight = hwMap.get(DcMotor.class, "front_right_motor");
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft = hwMap.get(DcMotor.class, "back_left_motor");
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight = hwMap.get(DcMotor.class, "back_right_motor");
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //grabber = hwMap.get(Servo.class, "left_hand");
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
    }

    void initServo(HardwareMap hwMap){
        grabber = hwMap.get(Servo.class, "grabber");//0 - control hub
        wobbleArm = hwMap.get(Servo.class, "wobble_arm");//4 - control hub
        lifter = hwMap.get(Servo.class, "lifter");//5 - control hub
        pusher = hwMap.get(Servo.class, "pusher");//3 - ext hub

    }
    void initShooterMotors(HardwareMap hwMap){
        shooterFront = hwMap.get(DcMotor.class, "shooter_front");//
        shooterBack = hwMap.get(DcMotor.class, "shooter_back");
    }

    void initIntakeAndConveyor(HardwareMap hwMap){
        intake = hwMap.get(DcMotor.class, "intake");
        conveyor = hwMap.get(DcMotor.class, "conveyor");
    }


    void setAllMotorsToRunToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void setAllMotorsToRunUsingEncoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        shooterBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void setSpeeds(double flSpeed, double frSpeed, double blSpeed, double brSpeed) {
        double largest = maxSpeed;
        /*
        largest = Math.max(largest, Math.abs(flSpeed));
        largest = Math.max(largest, Math.abs(frSpeed));
        largest = Math.max(largest, Math.abs(blSpeed));
        largest = Math.max(largest, Math.abs(brSpeed));

        frontLeft.setPower(flSpeed / largest);
        frontRight.setPower(frSpeed / largest);
        backLeft.setPower(blSpeed / largest);
        backRight.setPower(brSpeed / largest);
        */


        frontLeft.setPower(flSpeed);
        frontRight.setPower(frSpeed);
        backLeft.setPower(blSpeed);
        backRight.setPower(brSpeed);
    }

    void driveMecanum(double forward, double strafe, double rotate) {
        //watch following site to get more details on different options on how to move motors
        //https://stemrobotics.cs.pdx.edu/node/4746
        double frontLeftSpeed = forward + strafe + rotate;
        double frontRightSpeed = forward - strafe - rotate;
        double backLeftSpeed = forward - strafe + rotate;
        double backRightSpeed = forward + strafe - rotate;

        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    double[] getDistanceCm() {
        double[] distances = {0.0, 0.0};

        encoderMatrix.put(0, 0, (float) ((frontLeft.getCurrentPosition() - frontLeftOffset) * CM_PER_TICK));
        encoderMatrix.put(1, 0, (float) ((frontRight.getCurrentPosition() - frontRightOffset) * CM_PER_TICK));
        encoderMatrix.put(2, 0, (float) ((backLeft.getCurrentPosition() - backLeftOffset) * CM_PER_TICK));

        MatrixF distanceMatrix = conversion.multiplied(encoderMatrix);
        distances[0] = distanceMatrix.get(0, 0);
        distances[1] = distanceMatrix.get(1, 0);

        return distances;
    }

    void setMaxSpeed(double speed) {
        maxSpeed = Math.min(speed, 1.0);
    }

    public double getMaxSpeed() {
        return maxSpeed;
    }

    public void setEncoderOffsets() {
        frontRightOffset = frontRight.getCurrentPosition();
        frontLeftOffset = frontLeft.getCurrentPosition();
        backLeftOffset = backLeft.getCurrentPosition();
        backRightOffset = backRight.getCurrentPosition();
    }

    public void setAllWheelsToTargetPosition(int distance) {
        frontLeft.setTargetPosition((int) (distance*COUNTS_PER_INCH));
        frontRight.setTargetPosition((int) (distance*COUNTS_PER_INCH));
        backLeft.setTargetPosition((int) (distance*COUNTS_PER_INCH));
        backRight.setTargetPosition((int) (distance*COUNTS_PER_INCH));
    }

    public void moveForward(int distanceInInches, boolean isOpModeActive, int timeoutS, double speed, Telemetry telemetry){

        //get current position for all motors so we can start from there
        flPos = frontLeft.getCurrentPosition();
        frPos = frontRight.getCurrentPosition();
        blPos = backLeft.getCurrentPosition();
        brPos = backRight.getCurrentPosition();

        // calculate new targets
        flPos += distanceInInches * COUNTS_PER_INCH;
        frPos += distanceInInches * COUNTS_PER_INCH;
        blPos += distanceInInches * COUNTS_PER_INCH;
        brPos += distanceInInches * COUNTS_PER_INCH;

        //now since we have right positions for all motors, set target positions for all motors
        frontLeft.setTargetPosition(flPos);
        frontRight.setTargetPosition(frPos);
        backLeft.setTargetPosition(blPos);
        backRight.setTargetPosition(brPos);

        setAllMotorsToRunToPosition();
        runtime.reset();
        setSpeeds(speed, speed,speed,speed);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() && frontRight.isBusy())) {
            //wait or print something in telemetry
            telemetry.addLine("Moving forward...");
            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
            telemetry.addData("Actual", "%7d :%7d", frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition(), backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());
            telemetry.update();
        }
        setSpeeds(0,0,0,0);


    }

    public void moveBackward(int distanceInInches, boolean isOpModeActive, int timeoutS, double speed, Telemetry telemetry){

        //get current position for all motors so we can start from there
        flPos = frontLeft.getCurrentPosition();
        frPos = frontRight.getCurrentPosition();
        blPos = backLeft.getCurrentPosition();
        brPos = backRight.getCurrentPosition();

        // calculate new targets
        flPos -= distanceInInches * COUNTS_PER_INCH;
        frPos -= distanceInInches * COUNTS_PER_INCH;
        blPos -= distanceInInches * COUNTS_PER_INCH;
        brPos -= distanceInInches * COUNTS_PER_INCH;

        //now since we have right positions for all motors, set target positions for all motors
        frontLeft.setTargetPosition(flPos);
        frontRight.setTargetPosition(frPos);
        backLeft.setTargetPosition(blPos);
        backRight.setTargetPosition(brPos);

        setAllMotorsToRunToPosition();
        runtime.reset();
        setSpeeds(speed, speed,speed,speed);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() && frontRight.isBusy())) {
            //wait or print something in telemetry
            telemetry.addLine("Moving backward...");
            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
            telemetry.addData("Actual", "%7d :%7d", frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition(), backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());
            telemetry.update();
        }
        setSpeeds(0,0,0,0);

    }

    public void strafeLeft(int distanceInInches, boolean isOpModeActive, int timeoutS, double speed, Telemetry telemetry){

        //get current position for all motors so we can start from there
        flPos = frontLeft.getCurrentPosition();
        frPos = frontRight.getCurrentPosition();
        blPos = backLeft.getCurrentPosition();
        brPos = backRight.getCurrentPosition();

        // calculate new targets
        flPos -= distanceInInches * COUNTS_PER_INCH;
        frPos += distanceInInches * COUNTS_PER_INCH;
        blPos += distanceInInches * COUNTS_PER_INCH;
        brPos -= distanceInInches * COUNTS_PER_INCH;

        //now since we have right positions for all motors, set target positions for all motors
        frontLeft.setTargetPosition(flPos);
        frontRight.setTargetPosition(frPos);
        backLeft.setTargetPosition(blPos);
        backRight.setTargetPosition(brPos);

        setAllMotorsToRunToPosition();
        runtime.reset();
        setSpeeds(speed, speed,speed,speed);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() && frontRight.isBusy())) {
            //wait or print something in telemetry
            telemetry.addLine("Strafing left..");
            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
            telemetry.addData("Actual", "%7d :%7d", frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition(), backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());
            telemetry.update();
        }
        setSpeeds(0,0,0,0);

    }

    public void strafeRight(int distanceInInches, boolean isOpModeActive, int timeoutS, double speed, Telemetry telemetry){

        //get current position for all motors so we can start from there
        flPos = frontLeft.getCurrentPosition();
        frPos = frontRight.getCurrentPosition();
        blPos = backLeft.getCurrentPosition();
        brPos = backRight.getCurrentPosition();

        // calculate new targets
        flPos += distanceInInches * COUNTS_PER_INCH;
        frPos -= distanceInInches * COUNTS_PER_INCH;
        blPos -= distanceInInches * COUNTS_PER_INCH;
        brPos += distanceInInches * COUNTS_PER_INCH;

        //now since we have right positions for all motors, set target positions for all motors
        frontLeft.setTargetPosition(flPos);
        frontRight.setTargetPosition(frPos);
        backLeft.setTargetPosition(blPos);
        backRight.setTargetPosition(brPos);

        setAllMotorsToRunToPosition();
        runtime.reset();
        setSpeeds(speed, speed,speed,speed);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {
            //wait or print something in telemetry
            telemetry.addLine("Strafing right...");
            telemetry.addData("runtime seconds ",runtime.seconds());
            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
            telemetry.addData("Actual", "%7d :%7d", frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition(), backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());
            telemetry.update();
        }
        setSpeeds(0,0,0,0);

    }

    public void rotateLeft(int distanceInInches, boolean isOpModeActive, int timeoutS, double speed, Telemetry telemetry){

        //get current position for all motors so we can start from there
        flPos = frontLeft.getCurrentPosition();
        frPos = frontRight.getCurrentPosition();
        blPos = backLeft.getCurrentPosition();
        brPos = backRight.getCurrentPosition();

        // calculate new targets
        flPos -= distanceInInches * COUNTS_PER_INCH;
        frPos += distanceInInches * COUNTS_PER_INCH;
        blPos -= distanceInInches * COUNTS_PER_INCH;
        brPos += distanceInInches * COUNTS_PER_INCH;

        //now since we have right positions for all motors, set target positions for all motors
        frontLeft.setTargetPosition(flPos);
        frontRight.setTargetPosition(frPos);
        backLeft.setTargetPosition(blPos);
        backRight.setTargetPosition(brPos);

        setAllMotorsToRunToPosition();
        runtime.reset();
        setSpeeds(speed, speed,speed,speed);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() && frontRight.isBusy())) {
            //wait or print something in telemetry
            telemetry.addLine("Rotating left..");
            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
            telemetry.addData("Actual", "%7d :%7d", frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition(), backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());
            telemetry.update();
        }
        setSpeeds(0,0,0,0);

    }

    public void rotateRight(int distanceInInches, boolean isOpModeActive, int timeoutS, double speed, Telemetry telemetry){

        //get current position for all motors so we can start from there
        flPos = frontLeft.getCurrentPosition();
        frPos = frontRight.getCurrentPosition();
        blPos = backLeft.getCurrentPosition();
        brPos = backRight.getCurrentPosition();

        // calculate new targets
        flPos += distanceInInches * COUNTS_PER_INCH;
        frPos -= distanceInInches * COUNTS_PER_INCH;
        blPos += distanceInInches * COUNTS_PER_INCH;
        brPos -= distanceInInches * COUNTS_PER_INCH;

        //now since we have right positions for all motors, set target positions for all motors
        frontLeft.setTargetPosition(flPos);
        frontRight.setTargetPosition(frPos);
        backLeft.setTargetPosition(blPos);
        backRight.setTargetPosition(brPos);

        setAllMotorsToRunToPosition();
        runtime.reset();
        setSpeeds(speed, speed,speed,speed);
        while (runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() && frontRight.isBusy())) {
            //wait or print something in telemetry
            telemetry.addLine("Rotating right...");
            telemetry.addData("Target", "%7d :%7d", flPos, frPos, blPos, brPos);
            telemetry.addData("Actual", "%7d :%7d", frontLeft.getCurrentPosition(),
                    frontRight.getCurrentPosition(), backLeft.getCurrentPosition(),
                    backRight.getCurrentPosition());
            telemetry.update();
        }
        setSpeeds(0,0,0,0);

    }


    public void moveBasedOnTotalRings(int totalRings, Telemetry telemetry) {
        if(totalRings == 0){
            //Strafe right
            //strafeRight(20, true, 5, fast, telemetry);
            //Move forward to A
            //moveForward(20, true, 5, fast, telemetry);

        }else if(totalRings == 1){
            //Strafe right
            //rotateRight(3, true, 5, slow, telemetry);
            //Move forward to A
            moveBackward(2, true, 5, slow, telemetry);
            //Strafe left to B
            //strafeLeft(10, true, 5, fast, telemetry);

        }else if(totalRings == 4){
            //Strafe right
            strafeRight(1, true, 5, fast, telemetry);
            //Move forward to A
            //moveForward(24, true, 5, fast, telemetry);

        }
    }

    public void grabWobble(){
        grabber.setPosition(0);
    }

    public void releaseWobble(){
        grabber.setPosition(1.0);
    }

    public void runIntake(double intakePower){
        intake.setPower(intakePower);
    }

    public void runConveyor(double conveyorPower){
        conveyor.setPower(conveyorPower);
    }

    public void runShooterFront(double shooterFrontPower){
        shooterFront.setPower(shooterFrontPower);
    }

    public void runShooterBack(double shooterBackPower){
        shooterBack.setPower(shooterBackPower);
    }

    public void moveWobbleArmUp() {
        wobbleArm.setPosition(0);
    }
    public void moveWobbleArmDown() {
        wobbleArm.setPosition(1.0);
    }
    public void moveLifterUp() {
        lifter.setPosition(0);
    }
    public void moveLifterDown() {
        lifter.setPosition(1.0);
    }


    public void liftUp() {
        lifter.setDirection(Servo.Direction.FORWARD);
        lifter.setPosition(0.5);
    }
    public void liftDown() {
        lifter.setDirection(Servo.Direction.FORWARD);
        lifter.setPosition(0.5);
    }

    /**
     * This method puts the current thread to sleep for the given time in msec.
     * It handles InterruptException where it recalculates the remaining time
     * and calls sleep again repeatedly until the specified sleep time has past.
     *
     * @param sleepTime specifies sleep time in msec.
     */
    public static void sleep(long sleepTime){
        long wakeupTime = System.currentTimeMillis() + sleepTime;

        while (sleepTime > 0){
            try{
                Thread.sleep(sleepTime);
            }catch (InterruptedException e){

            }
            sleepTime = wakeupTime - System.currentTimeMillis();
        }
    }


}