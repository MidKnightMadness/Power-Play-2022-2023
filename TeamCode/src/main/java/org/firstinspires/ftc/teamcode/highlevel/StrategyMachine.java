package org.firstinspires.ftc.teamcode.highlevel;

// Project imports
import org.firstinspires.ftc.teamcode.common.Timer;
import org.firstinspires.ftc.teamcode.odometry.Odometry;

// Auxillary imports


public class StrategyMachine {
    /* Class description:

    This class contains tunable functions to iterate the

    */
    public int [][] scored;
    public double [] baselineIncentives = {0.0, 0.0, 0.0}; // Not static since needs to vary with conditions
    boolean autonomous;
    double timeRemaining;
    private Odometry odometry;


    public StrategyMachine(boolean autonomous, double timeLimit, int currentScore, Odometry odometry){
        // Copy over numerical data
        for(int i = 0; i < scored.length; i++){ // ith row
            for(int j = 0; ){

            }
        }

    }

    public StrategyMachine(boolean autonomous, double timeLimit, int currentScore, int [][] scored, double [] baselineIncentives, Odometry odometry){
        // Copy over numerical data

        // Copy over arrays
    }

}

class StrategyTimer implements Runnable{
    org.firstinspires.ftc.teamcode.common.Timer timer;

    @Override
    public void run() {
        // Start timer
        timer = new Timer();


    }
}
