/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package scara_kinematics;

/**
 *
 * @author cbelettr
 */
public class Segment {
    int start_point_index;
    int end_point_index;
    String description;
    public Segment()
    {
        start_point_index = 0;
        end_point_index = 0;
        description = "";
    }
    
    public Segment(int s_point, int e_point, String desc)
    {
        start_point_index = s_point;
        end_point_index = e_point;
        description = desc;
    }
    
    public void setStartPoint(int i)
    {
        start_point_index = i;
    }
    
    public void setEndPoint(int i)
    {
        end_point_index = i;
    }
    
    public int getStartPoint()
    {
        return start_point_index;
    }
    
    public int getEndPoint()
    {
        return end_point_index;
    }
    
    public String getDescription()
    {  
        return description;
    }
    public void setDescription(String desc)
    {
        description = desc;
    }
}
