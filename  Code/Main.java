package iBug;
import java.io.IOException;
import simbad.gui.*;
import javax.vecmath.*;
import simbad.sim.*;


import simbad.gui.Simbad;
import simbad.sim.EnvironmentDescription;
// import subsumption.BehaviorBasedAgent;

import javax.vecmath.Vector3d;

public class Main {

    public Main() {

    }

    public static void main(String[] args) {
        // Create the environment
        EnvironmentDescription environment = new Env();

        // Sets the robot under the light source in order to get the luminosity there.
        // This is a must and has to implemented first of all so as to be able to 
        // know later when to stop the robot, i.e. when it has reached the goal
        // MyRobot myRobot = new MyRobot(new Vector3d(6, 0, 6), "Robot", goal);

        // Initialize the robot in a random position and add it to the environment
        MyRobot myRobot = new MyRobot(new Vector3d(-9, 0, -6), "Robot");
        environment.add(myRobot);

        // Create a Simbad object
        Simbad frame = new Simbad(environment, false);
    }

}

