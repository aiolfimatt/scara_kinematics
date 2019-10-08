/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package scara_kinematics;

import java.util.ArrayList;
import java.util.List;

/**
 *
 * @author cbelettr
 */
public class MotionObject {
    
    List parameters;
    List values;
    
    List trajectory;
    List motor1_points;
    List motor2_points;
    
    List graph_points;
    List graph_segments;
    List aux1_graph;
    List aux2_graph;
    List aux3_graph;
    List aux4_graph;
    List aux5_graph;
    
    String errorString;
    int n_points;
    double time_interval;
    
    String unit_traj;
    //
    String unit_motor1;
    String unit_motor2;
    //
   
    
    public MotionObject()
    {
        parameters =new ArrayList<String>();
        values = new ArrayList<Double>();
        trajectory = new ArrayList<Point>();
        graph_points = new ArrayList<Point>();
        graph_segments = new ArrayList<Segment>();
        aux1_graph = new ArrayList<Double>();
        aux2_graph = new ArrayList<Double>();
        aux3_graph = new ArrayList<Double>();
        aux4_graph = new ArrayList<Double>();
        aux5_graph = new ArrayList<Double>();
        motor1_points = new ArrayList<Double>();
        motor2_points = new ArrayList<Double>();
        unit_traj   = "";
        unit_motor1 = "";
        unit_motor2 = "";
        errorString = "";
        n_points = 0;
        time_interval = 1;
    }
    public MotionObject(List parameters, List values, int n_points, double time_interval, String traj_unit, String motor1_unit, String motor2_unit)
    {
        this.parameters = parameters;
        this.values = values;
        this.n_points = n_points;
        this.time_interval = time_interval;
        this.unit_traj = traj_unit;
        this.unit_motor1 = motor1_unit;
        this.unit_motor2 = motor2_unit;
    }
    public boolean generateTrajectory()
    {
        return false;
    }
    public boolean generateGraphPoints(int n_point, int B_WIDTH, int B_HEIGHT)
    {
        return false;
    }
    public void addParameter(String name, Double value)
    {
        parameters.add(name);
        values.add(value);
    }
    public void setNPoints(int n)
    {
        n_points = n;
    }
    public void setTimeInterval(Double t)
    {
        time_interval = t;
    }
}
