/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package scara_kinematics;

import java.util.ArrayList;
import javax.swing.JOptionPane;

/**
 *
 * @author cbelettr
 */
public class Scara_cam extends MotionObject {

    public Scara_cam(boolean fill) {

        super();
        if(fill)
        {
            n_points = 500;
            //
            parameters.add("Arm L1 (mm)");
            parameters.add("Aem L2 (mm)");
            parameters.add("Inernal area reduction K (mm)");
            parameters.add("X1 start (mm)");
            parameters.add("Y1 start (mm)");
            parameters.add("X2 stop (mm)");
            parameters.add("Y2 stop (mm)");
            parameters.add("Acceleration (%)");
            parameters.add("Constant speed (%)");
            parameters.add("Deceleration (%)");
            parameters.add("Arm angle inversion 1");
            parameters.add("Arm angle inversion 2");
            //
            values.add(200.0);
            values.add(150.0);
            values.add(46.0);
            values.add(200.0);
            values.add(-100.0);
            values.add(100.0);
            values.add(120.0);
            values.add(30.0);
            values.add(40.0);
            values.add(30.0);
            values.add(0.0);
            values.add(1.0);
            //
            
            //
        }
        //
        graph_points.add(new Point(0, 0, "Origine"));
        graph_points.add(new Point(20, 20, "Fine L1"));
        graph_points.add(new Point(20, 40, "Fine L2"));
        //
        graph_segments.add(new Segment(0, 1, "L1"));
        graph_segments.add(new Segment(1, 2, "L2"));
        //
        unit_motor1 = "Deg";
        unit_motor2 = "Deg";
        //
        unit_traj = "mm";
    }
    
    public boolean generateGraphPoints(int n_point,int B_WIDTH,int B_HEIGTH)
    {
        Point pxy = (Point)trajectory.get(n_point);
        double x = pxy.getX();
        double y = pxy.getY();
        double L1 = (Double)values.get(0);
        double L2 = (Double)values.get(1);
        double lTot = L1 + L2;
        double scaleX = B_WIDTH/(2*lTot);
        double scaleY = B_HEIGTH/(2*lTot);
        double offsetX = (lTot*scaleX);
        double offsetY = (lTot*scaleY);
        try
        {   
            Point p0 = new Point(offsetX, offsetY);
            graph_points.set(0,p0);
            double alfa = ((Double)motor1_points.get(n_point)/360*(2*Math.PI));
            // end point L1
            Point pM = new Point( (L1*Math.cos(alfa) * scaleX) + offsetX,((L1*Math.sin(alfa)*scaleY)+offsetY)); 
            graph_points.set(1, pM);
            //end point L2
            Point pB = new Point((x*scaleX)+offsetX,(y*scaleY)+offsetY);
            graph_points.set(2,pB); 
        }
        catch(Exception e)
        {
            e.printStackTrace();
            return false;
        }
        return true;
    }

    public boolean generateTrajectory() {
        
        
        double L1        =    (Double) values.get(0);
        double L2        =    (Double) values.get(1);
        double K         =    (Double) values.get(2);
        double X1        =    (Double) values.get(3);
        double Y1        =    (Double) values.get(4);
        double X2        =    (Double) values.get(5);
        double Y2        =    (Double) values.get(6);
        double perc_acc  =    (Double) values.get(7);
        double perc_cost =    (Double) values.get(8);
        double perc_dec  =    (Double) values.get(9);
        double inv_ang_1 =    (Double) values.get(10);
        double inv_ang_2 =    (Double) values.get(11);
        double raggio_inizio=0;
        double raggio_fine = 0;
        errorString = "";
        
        //calcolo traiettoria scara
        try {
            raggio_inizio = Math.sqrt(Math.pow(X1,2)+Math.pow(Y1,2));
            raggio_fine = Math.sqrt(Math.pow(X2,2)+Math.pow(Y2,2));
            
            if ((perc_acc + perc_cost + perc_dec) != 100) {
                errorString = "Errore calcolo: percentuali movimentazione";
                return false;
            }
            if((L1 <= 0) || (L2<= 0) || (K <= 0) )
            {
                errorString = "Errore calcolo: parametro nullo o <0";
                return false;
            }
            if((L1 - L2) < 0)
            {
               errorString = "Errore calcolo: (L1-L2) < 0 ";
                return false; 
            }
            
            if (((L1+L2) < raggio_inizio) || ((L1-L2+K) > raggio_inizio)) {
                errorString = "Errore calcolo: Posizione iniziale irraggiungibile";
                return false;
            }
            if (((L1+L2) < raggio_fine) || ((L1-L2+K) > raggio_fine)) {
                errorString = "Errore calcolo: Posizione finale irraggiungibile";
                return false;
            }
            if ( (X2 == X1) && (Y2 == Y1)) {
                errorString = "Errore calcolo: punti iniziale e finale uguali";
                return false;
            }
            
            //calcolo angoli alfa1, beta1, alfa2, beta2
            double alfa1, beta1, alfa2, beta2;
            double int_1, int_2;
            //
            int_1 = Math.acos((Math.pow(X1,2)+Math.pow(Y1,2)-Math.pow(L1,2)-Math.pow(L2,2))/(2*L1*L2));
            if(inv_ang_1 == 1)
            {
                int_1 = -int_1;
            }
            int_2 = Math.atan(Y1/X1) - Math.atan(L2*Math.sin(int_1)/(L1+L2*Math.cos(int_1)));
            if (X1<=0)
            {
                int_2 = int_2 - Math.PI;
            }
            beta1 = ((int_1)/(2*Math.PI)*360);
            alfa1 = ((int_2)/(2*Math.PI)*360);
            //
            int_1 = Math.acos((Math.pow(X2,2)+Math.pow(Y2,2)-Math.pow(L1,2)-Math.pow(L2,2))/(2*L1*L2));
            if(inv_ang_2 == 1)
            {
                int_1 = -int_1;
            }
            int_2 = Math.atan(Y2/X2) - Math.atan(L2*Math.sin(int_1)/(L1+L2*Math.cos(int_1)));
            if (X2<=0)
            {
                int_2 = int_2 - Math.PI;
            }
            beta2 = ((int_1)/(2*Math.PI)*360);
            alfa2 = ((int_2)/(2*Math.PI)*360);
            //
            // calcolo camma alfa
            //
            double corsa_indietro = alfa1;
            double corsa_avanti = alfa2;
            double dA, dC, dD;
            double V0 = 0;
            double V2 = 0;
            double acc, dec;
            double alzata = corsa_avanti - (corsa_indietro);
            dA = n_points / 100 * perc_acc;
            dC = n_points / 100 * perc_cost;
            dD = n_points / 100 * perc_dec;
            double V1 = (2 * alzata - V0 * dA - V2 * dD) / (dA + 2 * dC + dD);
            double CAM_T1_1A = corsa_indietro + (V1 - V0) * dA / 2 + V0 * dA;
            double CAM_T1_2A = CAM_T1_1A + V1 * dC;
            acc = (V1 - V0) / dA;
            dec = (V2 - V1) / dD;
            ///acc trapezio
            double A1_0 = dA / 8;
            double A3_0 = A1_0;
            double A2_0 = A1_0 * 6;
            double accA1_0 = 9.142857 * acc / dA;
            double accR_0 = 1.142857 * acc;
            double accA3_0 = -accA1_0;
            ///dec trapezio
            double A1_2 = dD / 8;
            double A3_2 = A1_2;
            double A2_2 = A1_2 * 6;
            double accA2_2 = 9.142857 * dec / dD;
            double accR_2 = 1.142857 * dec;
            double accA3_2 = -accA2_2;
            int current_section = 0; ///0 acc1 1 acc 2 acc3 3 velcost 4 dec1 5 dec2 6 dec3
            double offset = 0;
            double ain = 0, vin = 0, ain1 = 0, vin1 = 0;
            //calcolo sequenza punti
            motor1_points = new ArrayList<Double>();
            for (int k = 0; k < n_points; k++) {
                double xx = (k - offset);
                switch (current_section) {
                    case 0:
                        if (xx > A1_0) {
                            current_section++;
                            offset += A1_0;
                            ain = ((corsa_indietro + (accA1_0 * A1_0 * A1_0 * A1_0) / 6 + V0 * A1_0));
                            vin = ((V0 + (accA1_0 * A1_0 * A1_0) / 2));
                            xx = (k - offset);
                        }
                        break;
                    case 1:
                        if (xx > A2_0) {
                            current_section++;
                            offset += A2_0;
                            ain1 = ((ain + (accR_0 * A2_0 * A2_0) / 2 + vin * A2_0));
                            vin1 = ((vin + (accR_0 * A2_0)));
                            xx = (k - offset);
                        }
                        break;
                    case 2:
                        if (xx > A3_0) {
                            current_section++;
                            offset += A3_0;
                            xx = (k - offset);
                        }
                        break;
                    case 3:
                        if (xx > dC) {
                            current_section++;
                            offset += dC;
                            xx = (k - offset);
                        }
                        break;
                    case 4:
                        if (xx > A1_2) {
                            current_section++;
                            offset += A1_2;
                            ain = ((CAM_T1_2A + (accA2_2 * A1_2 * A1_2 * A1_2) / 6 + V1 * A1_2));
                            vin = ((V1 + (accA2_2 * A1_2 * A1_2) / 2));
                            xx = (k - offset);
                        }
                        break;
                    case 5:
                        if (xx > A2_2) {
                            current_section++;
                            offset += A2_2;
                            ain1 = (float) ((ain + (accR_2 * A2_2 * A2_2) / 2 + vin * A2_2));
                            vin1 = (float) ((vin + (accR_2 * A2_2)));
                            xx = (k - offset);
                        }
                        break;
                    case 6: {
                        if (xx >= A3_2) {
                            offset += A3_2;
                            xx = A3_2;
                        }
                    }
                    break;
                }
                switch (current_section) {
                    case 0: {
                        double toAdd = ((corsa_indietro + (accA1_0 * xx * xx * xx) / 6 + V0 * xx));
                        motor1_points.add(toAdd);
                    }
                    break;
                    case 1: {
                        double toAdd = ((ain + (accR_0 * xx * xx) / 2 + vin * xx));
                        motor1_points.add(toAdd);
                    }
                    break;
                    case 2: {
                        double toAdd = ((ain1 + (accR_0 * xx * xx) / 2 + vin1 * xx + (accA3_0 * xx * xx * xx) / 6));
                        motor1_points.add(toAdd);
                    }
                    break;
                    case 3: {
                        double toAdd = ((CAM_T1_1A + V1 * xx));
                        motor1_points.add(toAdd);
                    }
                    break;
                    case 4: {
                        double toAdd = ((CAM_T1_2A + (accA2_2 * xx * xx * xx) / 6 + V1 * xx));
                        motor1_points.add(toAdd);
                    }
                    break;
                    case 5: {
                        double toAdd = ((ain + (accR_2 * xx * xx) / 2 + vin * xx));
                        motor1_points.add(toAdd);
                    }
                    break;
                    case 6: {
                        double toAdd = ((ain1 + (accR_2 * xx * xx) / 2 + vin1 * xx + (accA3_2 * xx * xx * xx) / 6));
                        motor1_points.add(toAdd);
                    }
                    break;
                }
            }
            
            
            //
            // calcolo camma alfa
            corsa_indietro = beta1;
            corsa_avanti = beta2;
            alzata = corsa_avanti - (corsa_indietro);
            dA = n_points / 100 * perc_acc;
            dC = n_points / 100 * perc_cost;
            dD = n_points / 100 * perc_dec;
            V1 = (2 * alzata - V0 * dA - V2 * dD) / (dA + 2 * dC + dD);
            CAM_T1_1A = corsa_indietro + (V1 - V0) * dA / 2 + V0 * dA;
            CAM_T1_2A = CAM_T1_1A + V1 * dC;
            acc = (V1 - V0) / dA;
            dec = (V2 - V1) / dD;
            ///acc trapezio
            A1_0 = dA / 8;
            A3_0 = A1_0;
            A2_0 = A1_0 * 6;
            accA1_0 = 9.142857 * acc / dA;
            accR_0 = 1.142857 * acc;
            accA3_0 = -accA1_0;
            ///dec trapezio
            A1_2 = dD / 8;
            A3_2 = A1_2;
            A2_2 = A1_2 * 6;
            accA2_2 = 9.142857 * dec / dD;
            accR_2 = 1.142857 * dec;
            accA3_2 = -accA2_2;
            current_section = 0; ///0 acc1 1 acc 2 acc3 3 velcost 4 dec1 5 dec2 6 dec3
            offset = 0;
            ain = 0;
            vin = 0;
            ain1 = 0;
            vin1 = 0;
            //calcolo sequenza punti
            motor2_points = new ArrayList<Double>();
            for (int k = 0; k < n_points; k++) {
                double xx = (k - offset);
                switch (current_section) {
                    case 0:
                        if (xx > A1_0) {
                            current_section++;
                            offset += A1_0;
                            ain = ((corsa_indietro + (accA1_0 * A1_0 * A1_0 * A1_0) / 6 + V0 * A1_0));
                            vin = ((V0 + (accA1_0 * A1_0 * A1_0) / 2));
                            xx = (k - offset);
                        }
                        break;
                    case 1:
                        if (xx > A2_0) {
                            current_section++;
                            offset += A2_0;
                            ain1 = ((ain + (accR_0 * A2_0 * A2_0) / 2 + vin * A2_0));
                            vin1 = ((vin + (accR_0 * A2_0)));
                            xx = (k - offset);
                        }
                        break;
                    case 2:
                        if (xx > A3_0) {
                            current_section++;
                            offset += A3_0;
                            xx = (k - offset);
                        }
                        break;
                    case 3:
                        if (xx > dC) {
                            current_section++;
                            offset += dC;
                            xx = (k - offset);
                        }
                        break;
                    case 4:
                        if (xx > A1_2) {
                            current_section++;
                            offset += A1_2;
                            ain = ((CAM_T1_2A + (accA2_2 * A1_2 * A1_2 * A1_2) / 6 + V1 * A1_2));
                            vin = ((V1 + (accA2_2 * A1_2 * A1_2) / 2));
                            xx = (k - offset);
                        }
                        break;
                    case 5:
                        if (xx > A2_2) {
                            current_section++;
                            offset += A2_2;
                            ain1 = (float) ((ain + (accR_2 * A2_2 * A2_2) / 2 + vin * A2_2));
                            vin1 = (float) ((vin + (accR_2 * A2_2)));
                            xx = (k - offset);
                        }
                        break;
                    case 6: {
                        if (xx >= A3_2) {
                            offset += A3_2;
                            xx = A3_2;
                        }
                    }
                    break;
                }
                switch (current_section) {
                    case 0: {
                        double toAdd = ((corsa_indietro + (accA1_0 * xx * xx * xx) / 6 + V0 * xx));
                        motor2_points.add(toAdd);
                    }
                    break;
                    case 1: {
                        double toAdd = ((ain + (accR_0 * xx * xx) / 2 + vin * xx));
                        motor2_points.add(toAdd);
                    }
                    break;
                    case 2: {
                        double toAdd = ((ain1 + (accR_0 * xx * xx) / 2 + vin1 * xx + (accA3_0 * xx * xx * xx) / 6));
                        motor2_points.add(toAdd);
                    }
                    break;
                    case 3: {
                        double toAdd = ((CAM_T1_1A + V1 * xx));
                        motor2_points.add(toAdd);
                    }
                    break;
                    case 4: {
                        double toAdd = ((CAM_T1_2A + (accA2_2 * xx * xx * xx) / 6 + V1 * xx));
                        motor2_points.add(toAdd);
                    }
                    break;
                    case 5: {
                        double toAdd = ((ain + (accR_2 * xx * xx) / 2 + vin * xx));
                        motor2_points.add(toAdd);
                    }
                    break;
                    case 6: {
                        double toAdd = ((ain1 + (accR_2 * xx * xx) / 2 + vin1 * xx + (accA3_2 * xx * xx * xx) / 6));
                        motor2_points.add(toAdd);
                    }
                    break;
                }
            }
            //calcolo punti traiettoria
            trajectory = new ArrayList<Point>();
            aux1_graph = new ArrayList<Double>();
            aux2_graph = new ArrayList<Double>();
            aux3_graph = new ArrayList<Double>();
            aux4_graph = new ArrayList<Double>();
            aux5_graph = new ArrayList<Double>();
            
            for(int k=0;k<motor1_points.size();k++)
            {
                double alfa = (Double)motor1_points.get(k);
                double beta = (Double)motor2_points.get(k);
                
                double x,y;
                
                alfa = alfa /360 *(2*Math.PI);
                beta = beta/360 * (2*Math.PI);
                x = L1 * Math.cos(alfa) + L2 * Math.cos(alfa + beta); 
                y = L1 * Math.sin(alfa) +  L2 * Math.sin(alfa +beta);
                
                trajectory.add (new Point(x,y));
                //
            }
        } catch (Exception e) {
            errorString = "Errore: "+e.getMessage();
            e.printStackTrace();
            return false;
        }
        return true;
        
    }
}
