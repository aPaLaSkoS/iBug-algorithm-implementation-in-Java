package iBug;

import javax.vecmath.*;
import simbad.sim.*;


public class Env extends EnvironmentDescription {
    public Env(){
        createEnvironment();
    }

    public void createEnvironment() {

        // Set cherry agent's position and adds it to the environment
        Vector3d cherryAgentPos = new Vector3d(6, 0, 6); 
        addCherryAgent(cherryAgentPos, "myCherryAgent", 0.1f);

        // Adds 3 boxes of different sizes and in different positions into the environment
        addBox(-2., 0., 2., 4, 1, 5);
        addBox(3., 0., 5., 2, 1, 2);
        addBox(-6., 0., -4., 1, 1, 8);

        // Adds a light source into the environment at 2m height
        addLightSource(6, 2, 6);
        
    }

    // Adds a box into the environment
    public void addBox(double x, double y, double z, int xSize, int ySize, int zSize) {
        add(new Box(new Vector3d(x, y, z), new Vector3f(xSize, ySize, zSize), this));
    } 

    // Adds a light source into the environment
    public void addLightSource(double light_x, double light_y, double light_z) {
        light1SetPosition(light_x, light_y, light_z);
    }

    // Adds a cherry agent into the environment
    public void addCherryAgent(Vector3d pos, String name, float radius) {
        add(new CherryAgent(pos, name, radius));
    }
}