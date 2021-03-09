package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;


@Autonomous(name = "BluePegs", group = "Autonomous") // The main auto I have at the moment
public class BluePegs extends Auto {

    private double xRingScan[], xWobbleX[], xWobblyY[], xIntake[], xShoot1[], xShoot2[], xShoot3[];
    private double yRingScan[], yWobbleX[], yWobblyY[], yIntake[], yShoot1[], yShoot2[], yShoot3[];


    private String layout;


    //identify layout
    //move to the x position for that layout
    //move to the y position for that layout
    //move to peg shooting position (IF RINGS ARE ALREADY IN THE ROBOT)
    //intake any available rings (based on layout) - find a way to unstack?
    //shoot other rings
    //park


    public void runOpMode() throws InterruptedException {
        layout="";
        initialize();
        waitForStart();
        moveByWheelEncoders(0, 84, .5, "straight"); //the 84 inches might need to be tweaked.
        //move forward
//        PIDTurn(60,.75); //turn to rings
//        PIDTurn(0,.75);//turn back
//        try {
//            if (layout.equals(RING_COUNT.ZERO)) {
//                goZero();
//            } else if (layout.equals(RING_COUNT.ONE)) {
//                //goOne()
//            } else {
//                //goFour()
//            }
//        } catch (InterruptedException e) {
//
//        }
    }

    public void goZero() throws InterruptedException {
        //newMove(new Waypoint(0,0), new Waypoint(-1, -26.4), 0, .4, .9, 0, true, 5, 1.7);//wut

    }



}