package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Localization.PositionTracker;

import java.util.concurrent.TimeUnit;

import static android.os.SystemClock.sleep;

public class GamerOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, intake, shooter, wobbleArm, rampIntake, leftDead, rightDead, perpDead; //DeadWheels will probably be set to a specific motor that's already being used
    private Servo wobbleClaw, loader;
    private boolean precision, direction, precisionChanged;
    private boolean useOneGamePad=false;
    private double baseParallelRightPosition, baseParallelLeftPosition, basePerpendicularPosition,baseClawPosition;
    private boolean clawState=false; // claw position - false=open  true=closed
    private boolean clawStateChanged=false;
    //private final int deadWheelTicks = 4096; // DeadWheel Stuff
    //private final double WHEEL_CIRCUMFERENCE_IN=Math.PI*3.05; // circumference of parallel DeadWheel
    //private final double DEADWHEEL_INCHES_OVER_TICKS = WHEEL_CIRCUMFERENCE_IN/deadWheelTicks; // DeadWheel Stuff
    private PositionTracker positionTracker = new PositionTracker(0, 0, 0);

    @Override
    public void init() { // after driver hits init
        //setUpServos
        wobbleClaw = hardwareMap.servo.get("wobbleClaw");
        wobbleClaw.setPosition(0);
        loader = hardwareMap.servo.get("loader");
        loader.setPosition(0);
        //driveTrain
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
        rightFront.setDirection(DcMotor.Direction.REVERSE); //Right and left motors facing opposite direction so right motors set to reverse
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        //setUpIntake
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rampIntake = (DcMotorEx) hardwareMap.dcMotor.get("rampIntake");
        rampIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rampIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //setUpMotors
        shooter = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wobbleArm = (DcMotorEx) hardwareMap.dcMotor.get("wobbleArm");
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        precision = false;
        direction = false;
        useOneGamePad = false;
        precisionChanged = false;
        telemetry.addData("Status", "Initialized");
        baseClawPosition = wobbleClaw.getPosition();
//        baseParallelLeftPosition = leftDead.getCurrentPosition();
//        baseParallelRightPosition = rightDead.getCurrentPosition(); // DeadWheels, but name prob needs changing when plugged into hub
//        basePerpendicularPosition = perpDead.getCurrentPosition();
    }

    @Override
    // what runs in between robot being initialized and before it plays
    public void init_loop() { init(); }

    @Override
    public void start() {runtime.reset();} //runtime.reset();

    @Override
    public void loop() { // runner
        telemetry.update();
        //useEncoders(); DeadWheels method
        driveBot();
        intake();
        wobbler();
        //useEncoders(); DeadWheels method
        shooting();
        useOneGamePad();
    }

    public void driveBot() {
        if ((gamepad1.left_trigger>.5 || (useOneGamePad && gamepad1.right_bumper)) && !precisionChanged) {
            precision = !precision;
            precisionChanged = true; }
        else if (!(gamepad1.left_trigger >.5) || (useOneGamePad && gamepad1.right_bumper)) precisionChanged = false;

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
//        telemetry.addData("joystickAngle", joystickAngle); // Information for the phone for testing
//        telemetry.addData("Rear Motors", "Left Rear (%.2f), Right Rear (%.2f)", leftBack.getPower(), rightBack.getPower());
    }

    public double[] convertMotorPowers(double[] motorPowers, double xLinear, double xTurn) {
        double maxPower = getMaxMagnitude(motorPowers);
        double conversion = Math.abs(Math.sqrt((Math.pow(xLinear,2) + Math.pow(xTurn, 2))/*/2*/)/maxPower);
        telemetry.addData("Wobble Claw Position: ",wobbleClaw.getPosition());
        telemetry.update();
//        telemetry.addData("maxPower", maxPower); // more data onto the phone
//        telemetry.addData("conversion", conversion);
//        telemetry.update(); Information for the phone
        for (int i = 0; i < motorPowers.length; i++)  motorPowers[i] *= conversion;
        return motorPowers; }

    public double getMaxMagnitude(double[] arr) {
        double max = Math.abs(arr[0]);
        for (int i = 1; i < arr.length; i++)
            if (Math.abs(arr[i]) > max) max = arr[i];
        return max; }

    public void wobbler() { //find correct motor power for wobble arm
        // if(gamepad2.dpad_left /*&& !clawState && !clawStateChanged || (useOneGamePad && gamepad1.x && !clawState && !clawStateChanged*/ || (useOneGamePad && gamepad1.dpad_left)) { // claw is a boolean stating whether claw is open or closed
        //wobbleArm.setPower(-.8);
        //wobbleArm.setPower(.7); // test, clawState question, ask about how armTravel works on Rogue
        //clawState=!clawState;
        //clawStateChanged=true;
        // }
        //else if(gamepad2.x && clawState && clawStateChanged || (useOneGamePad && gamepad1.x && clawState && clawStateChanged)) {
        //  wobbleClaw.setPosition(0);
        //  clawState=!clawState;
        //  clawStateChanged=false; }
        //if(gamepad2.dpad_right /*&&  wobbleArm.getCurrentPosition()>baseClawPosition && wobbleArm.getCurrentPosition()<1000*/|| (useOneGamePad && gamepad1.dpad_right)) //this 1000 needs to be changed to the MAX position the wobble Arm can be at OR change it to not rely on a max. This code also sets the initial position of the arm to the minimum position
        //   wobbleArm.setPower(.1); // this power needs to be tweaked depending on preferred speed of the motor
        //else if(gamepad2.dpad_left || (useOneGamePad && gamepad1.dpad_left)) wobbleArm.setPower(.5);
        //else wobbleArm.setPower(0);
        if(gamepad2.dpad_left || (useOneGamePad && gamepad1.dpad_left)) wobbleArm.setPower(-.75); // Moves arm C.C (Counter-Clockwise)
        if(gamepad2.dpad_right || (useOneGamePad && gamepad1.dpad_right)) wobbleArm.setPower(.5); // Moves arm C (Clockwise)
        if(gamepad2.dpad_up || (useOneGamePad && gamepad1.dpad_up)) wobbleClaw.setPosition(1); // Opens Claw
        if(gamepad2.dpad_down || (useOneGamePad && gamepad1.dpad_down)) wobbleClaw.setPosition(baseClawPosition); // Closes Claw
        else wobbleArm.setPower(0); // When we don't wanna use the wobbler arm
        // wobbleClaw.setPosition(baseClawPosition);
    }

    public void intake() { // figure out what the best power for intake is
        if (gamepad2.b|| (useOneGamePad && gamepad1.left_bumper)) { // Spit Function: spits out (a) ring(s) if it's stuck
            intake.setPower(1);
            rampIntake.setPower(1); }
        else if (gamepad2.a|| (useOneGamePad && gamepad1.left_trigger > .5)) { // Intake Function: Intakes rings for shooting
            intake.setPower(-1);
            rampIntake.setPower(-1); }
        //else if(gamepad2.y || (useOneGamePad && gamepad1.y)){ // sus scenario #1
        //    intake.setPower(0);
        //    rampIntake.setPower(-1);
        //}
        // else if(gamepad2.a || (useOneGamePad && gamepad1.a)){ // sus scenario #2
        //    intake.setPower(-1);
        //    rampIntake.setPower(0);
        //}
        else {intake.setPower(0); // intakes stay zero when no one presses intake
            rampIntake.setPower(0);}
    }

    public void shooting(){
        /*if(gamepad1.left_trigger==1.0) { // change the length of time to build power
            while(gamepad1.left_trigger > .5) {
                runtime.reset();
                if (runtime.time(TimeUnit.MILLISECONDS) < 1000) //1000 milliseconds=1 seconds
                    shooter.setPower(runtime.time(TimeUnit.MILLISECONDS) / 1000.0);
                else if (runtime.time(TimeUnit.MILLISECONDS) >= 1000)
                    shooter.setPower(1);
            }
        } */ // Made a better method, giving the user the responsibility of the time when shooting the ring from the bot
        if(gamepad1.left_bumper || (useOneGamePad && gamepad1.right_trigger>.5)) shooter.setPower(-1); // sets the power for the motor - wires were flipped, so now set to negative (Can also be solved by setting motor to reverse in the initialization)
        else shooter.setPower(0); // when we don't ready up the motor
        if(gamepad1.right_trigger>.5 || (useOneGamePad && gamepad1.y)) loader.setPosition(1); // button for shooting the ring
        else loader.setPosition(0); // when we don't shoot the ring
    }
    // seb did beginning shit and was very sus.
    public void useOneGamePad() {
        if ((gamepad1.x && gamepad1.a && gamepad1.b) || (gamepad2.x && gamepad2.a && gamepad2.b))
            useOneGamePad = !useOneGamePad; } // test changes to this to make this smaller and more understandable

    /*public void useEncoders() {
        double parallelLeftTicks = leftDead.getCurrentPosition() - baseParallelLeftPosition;
        double parallelRightTicks = rightDead.getCurrentPosition() - baseParallelRightPosition;
        double perpendicularTicks = perpDead.getCurrentPosition() - basePerpendicularPosition;
        positionTracker.updateTicks(parallelLeftTicks, parallelRightTicks, perpendicularTicks);
        positionTracker.updateLocationAndPose(telemetry, "");
        telemetry.addData("parallel left ticks", parallelLeftTicks);
        telemetry.addData("parallel right ticks", parallelRightTicks);
        telemetry.addData("perpendicular ticks", perpendicularTicks);
        telemetry.addData("Distance traveled", (parallelLeftTicks + parallelRightTicks) * DEADWHEEL_INCHES_OVER_TICKS / 2);
        telemetry.addData("X Position", positionTracker.getCurrentX());
        telemetry.addData("Y Position", positionTracker.getCurrentY());
        telemetry.update();
    }*/

    @Override
    public void stop() { } // stops the bot
}