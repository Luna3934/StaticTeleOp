package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Localization.PositionTracker;

import java.util.concurrent.TimeUnit;

public class GamerOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, intake, shooter, wobbleArm, rampIntake, leftDead, rightDead, perpDead; //deadwheels will probably be set to a specific motor that's already being used
    private Servo wobbleClaw, loader, retractor;
    private boolean precision, direction, precisionChanged;
    private boolean useOneGamepad=false;
    private double baseParallelRightPosition, baseParallelLeftPosition, basePerpendicularPosition,baseClawPosition;
    private boolean clawState=false; //claw position- false= open  true=closed
    private boolean clawStateChanged=false;
    private boolean retractorState=false;//retractor position    false=up    true=down
    private boolean retractorStateChanged=false;
    private final int deadWheelTicks = 4096; //is this per revolution?
    private final double WHEEL_CIRCUMFERENCE_IN = Math.PI*3.05; //circumference of parallel deadwheel
    private final double DEADWHEEL_INCHES_OVER_TICKS = WHEEL_CIRCUMFERENCE_IN/deadWheelTicks;
    private PositionTracker positionTracker = new PositionTracker(0, 0, 0);

    @Override
    public void init() {
        //after driver hits init

        setUpDriveTrain();
        setUpIntake();
        setUpServos();
        setUpMotors();
        lowerRetractor();

        precision = false;
        direction = false;
        useOneGamepad = false;
        precisionChanged = false;

        telemetry.addData("Status", "Initialized");

        baseParallelLeftPosition = leftDead.getCurrentPosition();
        baseParallelRightPosition = rightDead.getCurrentPosition(); //deadwheels, but name prob needs changing when plugged into hub
        basePerpendicularPosition = perpDead.getCurrentPosition();

        baseClawPosition=wobbleArm.getCurrentPosition();
    }

    public void setUpDriveTrain() {
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("rightBack");
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Right and left motors facing opposite direction so right motors set to reverse
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setUpIntake() {
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rampIntake = (DcMotorEx) hardwareMap.dcMotor.get("rampIntake");
        rampIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rampIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void setUpServos() {
        wobbleClaw = hardwareMap.servo.get("wobbleClaw");
        retractor = hardwareMap.servo.get("retractor");
        loader = hardwareMap.servo.get("loader");

        //INIT SERVOS IN BEGINNING
        wobbleClaw.setPosition(0);
        retractor.setPosition(0);
        loader.setPosition(0);
    }

    public void setUpMotors() {
        shooter = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wobbleArm = (DcMotorEx) hardwareMap.dcMotor.get("wobbleArm");
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    //what runs in between robot being initialized and before it plays
    public void init_loop() {    }

    //once driver hits play
    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {
        telemetry.update();

        //useEncoders();
        driveBot();
        intake();
        wobbler();
        useEncoders();
        shooting();


        useOneGamepad();
    }

    public void useEncoders() {
        double parallelLeftTicks = (leftDead.getCurrentPosition() - baseParallelLeftPosition);
        double parallelRightTicks = rightDead.getCurrentPosition() - baseParallelRightPosition;
        double perpendicularTicks = perpDead.getCurrentPosition() - basePerpendicularPosition;

        positionTracker.updateTicks(parallelLeftTicks, parallelRightTicks, perpendicularTicks);

        if(Math.abs(gamepad1.left_stick_x) < Math.cos(Math.toRadians(5)) || Math.sqrt(Math.pow(gamepad1.left_stick_x,2) + Math.pow(gamepad1.left_stick_y,2)) < .1 ) {
            positionTracker.updateLocationAndPose(telemetry, "");
        }
        else {
            positionTracker.updateLocationAndPose(telemetry, "");
        }

        telemetry.addData("parallel left ticks", parallelLeftTicks);
        telemetry.addData("parallel right ticks", parallelRightTicks);
        telemetry.addData("perpendicular ticks", perpendicularTicks);
        telemetry.addData("Distance traveled", (parallelLeftTicks + parallelRightTicks)*DEADWHEEL_INCHES_OVER_TICKS/2);
        telemetry.addData("X Position", positionTracker.getCurrentX());
        telemetry.addData("Y Position", positionTracker.getCurrentY());
        telemetry.update();
    }

    public void driveBot() {
        if (gamepad1.right_bumper && !precisionChanged) {
            precision = !precision;
            precisionChanged = true;
        }
        else if (!gamepad1.right_bumper) {
            precisionChanged = false;
        }



        double xMagnitude = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double xLinear = direction ? xMagnitude : -xMagnitude;

        double joystickAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + 3*Math.PI/4;
        double xTurn = gamepad1.right_stick_x;


        double leftFrontPower = xLinear*Math.sin(joystickAngle) - xTurn;
        double rightFrontPower = xLinear*Math.cos(joystickAngle) + xTurn;
        double leftBackPower = xLinear*Math.cos(joystickAngle) - xTurn;
        double rightBackPower = xLinear*Math.sin(joystickAngle) + xTurn;

        double[] motorPowers = new double[]{leftFrontPower, rightFrontPower, leftBackPower, rightBackPower};
        motorPowers = convertMotorPowers(motorPowers, xLinear, xTurn);

        leftFront.setPower(precision ? 0.4 * motorPowers[0] : motorPowers[0]); //if precision is true, motor power = power*0.4
        rightFront.setPower(precision ? 0.4 * motorPowers[1] : motorPowers[1]);
        leftBack.setPower(precision ? 0.4 * motorPowers[2] : motorPowers[2]);
        rightBack.setPower(precision ? 0.4 * motorPowers[3] : motorPowers[3]);

//        telemetry.addData("Front Motors", "Left Front (%.2f), Right Front (%.2f)", leftFront.getPower(), rightFront.getPower());
//        telemetry.addData("joystickAngle", joystickAngle);
//        telemetry.addData("Rear Motors", "Left Rear (%.2f), Right Rear (%.2f)", leftBack.getPower(), rightBack.getPower());
    }

    public double[] convertMotorPowers(double[] motorPowers, double xLinear, double xTurn) {
        double maxPower = getMaxMagnitude(motorPowers);

        double conversion = Math.abs(Math.sqrt((Math.pow(xLinear,2) + Math.pow(xTurn, 2))/*/2*/)/maxPower);

//        telemetry.addData("maxPower", maxPower);
//        telemetry.addData("conversion", conversion);
//        telemetry.update();

        for (int i = 0; i < motorPowers.length; i++) {
            motorPowers[i] *= conversion;
        }

        return motorPowers;
    }

    public double getMaxMagnitude(double[] arr) {
        double max = Math.abs(arr[0]);

        for (int i = 1; i < arr.length; i++) {
            if (Math.abs(arr[i]) > max) {
                max = arr[i];
            }
        }

        return max;
    }

    public void wobbler() { //find correct motor power for wobblearm
        if(gamepad2.x && !clawState && !clawStateChanged || (useOneGamepad && gamepad1.x && !clawState && !clawStateChanged)) { //claw is a boolean stating whether claw is open or closed
            wobbleClaw.setPosition(1);
            clawState=!clawState;
            clawStateChanged=true;
        }
        else if(gamepad2.x && clawState && clawStateChanged || (useOneGamepad && gamepad1.x && clawState && clawStateChanged)) {
            wobbleClaw.setPosition(0);
            clawState=!clawState;
            clawStateChanged=false;
        }

        if(gamepad2.dpad_right &&  wobbleArm.getCurrentPosition()>baseClawPosition && wobbleArm.getCurrentPosition()<1000|| (useOneGamepad && gamepad1.dpad_right)) //this 1000 needs to be changed to the MAX position the wobble Arm can be at OR change it to not rely on a max. This code also sets the initial position of the arm to the minimum position
            wobbleArm.setPower(.05);//this power needs to be tweaked depending on preferred speed of the motor
        else if(gamepad2.dpad_left || (useOneGamepad && gamepad1.dpad_left))
            wobbleArm.setPower(-.05);
        else
            wobbleArm.setPower(0);
    }


    public void intake() { //figure out what the best power for intake is
        if (gamepad2.dpad_up || (useOneGamepad && gamepad1.dpad_up)) {
            intake.setPower(1);
            rampIntake.setPower(1);
        }
        else if (gamepad2.dpad_down || (useOneGamepad && gamepad1.dpad_down)) {
            intake.setPower(-1);
            rampIntake.setPower(-1);
        }
        else if(gamepad2.y || (useOneGamepad && gamepad1.y)){
            intake.setPower(0);
            rampIntake.setPower(-1);
        }
        else if(gamepad2.a || (useOneGamepad && gamepad1.a)){
            intake.setPower(-1);
            rampIntake.setPower(0);
        }
        else {
            intake.setPower(0);
            rampIntake.setPower(0);
        }

        if((gamepad2.b && !retractorState && !retractorStateChanged)||(useOneGamepad && gamepad1.b && !retractorState && !retractorStateChanged)){
            retractor.setPosition(1);
            retractorState=!retractorState;
            retractorStateChanged=true;
        }
        else if((gamepad2.b && retractorState && !retractorStateChanged)||(useOneGamepad && gamepad1.b && retractorState && !retractorStateChanged)){
            retractor.setPosition(0);
            retractorState=!retractorState;
        }
        else if((!gamepad2.b && retractorStateChanged) ||(useOneGamepad && !gamepad1.b && retractorStateChanged)){
            retractorStateChanged=false;
        }



    }

    public void shooting(){
        if(gamepad1.left_trigger==1) {//change the length of time to build power
            while(gamepad1.left_trigger==1) {
                runtime.reset();
                if (runtime.time(TimeUnit.MILLISECONDS) < 1000) //1000 milliseconds=1 seconds
                    shooter.setPower(runtime.time(TimeUnit.MILLISECONDS) / 1000.0);
                else if (runtime.time(TimeUnit.MILLISECONDS) >= 1000)
                    shooter.setPower(1);
            }
        }
        else
            shooter.setPower(0);
        if(gamepad1.right_trigger==1) {
            loader.setPosition(1);
            //might need a small pause here
            loader.setPosition(0);
        }
    }

    public void lowerRetractor(){
        retractor.setPosition(1);
        retractorState=!retractorState;
    }


    public void useOneGamepad() {
        if ((gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.left_trigger == 1 && gamepad1.right_trigger == 1) || (gamepad2.right_bumper && gamepad2.left_bumper && gamepad2.left_trigger == 1 && gamepad2.right_trigger == 1)) {
            useOneGamepad = !useOneGamepad;
        }
    }


    @Override
    public void stop() {

    }
}