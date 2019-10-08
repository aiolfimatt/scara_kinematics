/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package scara_kinematics;

import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Image;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import javax.swing.ImageIcon;
import javax.swing.JPanel;
import javax.swing.Timer;

public class Board extends JPanel implements ActionListener
{
    
    MotionObject mFather;
    
    public Board(MotionObject mf) {
        initBoard();
        mFather = mf;
    }

    private void initBoard() {
        setBackground(Color.WHITE);
        setPreferredSize(new Dimension(350, 350));
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        drawPoints(g);
    }

    private void drawPoints(Graphics g) {
        for(int i=0; i<mFather.graph_points.size(); i++)
        {
            Point p = (Point)mFather.graph_points.get(i);
            g.fillOval((int)p.getX(), (int)p.getY(), 5, 5);
        }
        for(int i=0;i<mFather.graph_segments.size();i++)
        {
            Segment s = (Segment)mFather.graph_segments.get(i);
            Point p1 = (Point)mFather.graph_points.get(s.getStartPoint());
            Point p2 = (Point)mFather.graph_points.get(s.getEndPoint());
            g.drawLine((int)p1.getX(), (int)p1.getY(), (int)p2.getX(), (int)p2.getY());
        }
        Toolkit.getDefaultToolkit().sync();
    }

    public void actionPerformed(ActionEvent e) {
        repaint();
    }

}
