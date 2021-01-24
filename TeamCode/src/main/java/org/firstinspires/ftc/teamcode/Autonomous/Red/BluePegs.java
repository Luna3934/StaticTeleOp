package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Autonomous.Auto;
import org.firstinspires.ftc.teamcode.Autonomous.Spline.Waypoint;
import org.firstinspires.ftc.teamcode.Autonomous.TensorFlow.TensorFlow;

import java.util.List;


@Autonomous(name = "BluePegs", group = "Autonomous")
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
        //move forward
        PIDTurn(60,.75); //turn to rings
        runTensorFlow();//scan for rings and return layout
        try {
            if (layout.equals(RING_COUNT.ZERO)) {
                goZero();
            } else if (layout.equals(RING_COUNT.ONE)) {
                //goOne()
            } else {
                //goFour()
            }
        } catch (InterruptedException e) {

        }
    }

    public void goZero() throws InterruptedException {
        //newMove(new Waypoint(0,0), new Waypoint(-1, -26.4), 0, .4, .9, 0, true, 5, 1.7);//wut

    }



}