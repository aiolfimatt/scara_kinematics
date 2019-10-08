/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package scara_kinematics;

/**
 *
 * @author cbelettr
 */
public class Point {
    double x;
    double y;
    String description;
    
    public Point()
    {
        x=0;
        y=0;
    }
    
    public Point(double x, double y, String desc)
    {
        this.x = x;
        this.y = y;
        this.description = desc;
    }
    public Point(double x, double y)
    {
        this.x = x;
        this.y = y;
        this.description = "";
    }
    
    public double getX()
    {
        return x;
    }
    
    public double getY()
    {
        return y;
    }
    
    public String getDescription()
    {
        return description;
    }
    
    public void setX(float x)
    {
        this.x = x;
    }
    
    public void setY(float y)
    {
        this.y = y;
    }
    
    public void setDescription (String desc)
    {
        this.description = desc;
    }
}
