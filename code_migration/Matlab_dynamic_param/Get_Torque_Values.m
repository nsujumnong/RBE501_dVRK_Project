function [Torque] = Get_Torque_Values(List, q_array)
%Get_Torque_Vales returns the 7 torque values for the given parameters
% returns a 7 x 1 array of torque values

g = List(1);
cm1_x = List(2);
cm1_y = List(3);
cm1_z = List(4);
cm2_x = List(5);
cm2_y = List(6);
cm2_z = List(7);
cm3_x = List(8);
cm3_y = List(9);
cm3_z = List(10);
cm4_x = List(11);
cm4_y = List(12);
cm4_z = List(13);
cm5_x = List(14);
cm5_y = List(15);
cm5_z = List(16);
cm6_x = List(17);
cm6_y = List(18);
cm6_z = List(19);
cm7_x = List(20);
cm7_y = List(21);
cm7_z = List(22);
m1 = List(23);
m2 = List(24);
m3 = List(25);
m4 = List(26);
m5 = List(27);
m6 = List(28);
m7 = List(29);
L2 = List(30);
L3 = List(31);
L4_z0 = List(32);
q1 = q_array(1);
q2 = q_array(2);
q3 = q_array(3);
q4 = q_array(4);
q5 = q_array(5);
q6 = q_array(6);
q7 = q_array(7);

Torque = zeros(7,1);

Torque(1)=0;
Torque(2)= L2*g*m2*sin(q2) + L2*g*m3*sin(q2) + L2*g*m4*sin(q2) + L2*g*m5*sin(q2) + L2*g*m6*sin(q2) + L2*g*m7*sin(q2) - cm2_y*g*m2*cos(q2) + cm2_x*g*m2*sin(q2) + L3*g*m3*cos(q2)*cos(q3) + L3*g*m4*cos(q2)*cos(q3) + L3*g*m5*cos(q2)*cos(q3) + L3*g*m6*cos(q2)*cos(q3) + L3*g*m7*cos(q2)*cos(q3) - L4_z0*g*m4*cos(q2)*sin(q3) - L4_z0*g*m4*cos(q3)*sin(q2) - L4_z0*g*m5*cos(q2)*sin(q3) - L4_z0*g*m5*cos(q3)*sin(q2) - L4_z0*g*m6*cos(q2)*sin(q3) - L4_z0*g*m6*cos(q3)*sin(q2) - L4_z0*g*m7*cos(q2)*sin(q3) - L4_z0*g*m7*cos(q3)*sin(q2) + cm3_x*g*m3*cos(q2)*cos(q3) - L3*g*m3*sin(q2)*sin(q3) - L3*g*m4*sin(q2)*sin(q3) - L3*g*m5*sin(q2)*sin(q3) - L3*g*m6*sin(q2)*sin(q3) - L3*g*m7*sin(q2)*sin(q3) - cm3_z*g*m3*cos(q2)*sin(q3) - cm3_z*g*m3*cos(q3)*sin(q2) + cm4_y*g*m4*cos(q2)*sin(q3) + cm4_y*g*m4*cos(q3)*sin(q2) - cm3_x*g*m3*sin(q2)*sin(q3) + cm4_x*g*m4*cos(q2)*cos(q3)*cos(q4) - cm4_z*g*m4*cos(q2)*cos(q3)*sin(q4) + cm5_y*g*m5*cos(q2)*cos(q3)*sin(q4) - cm5_z*g*m5*cos(q2)*cos(q5)*sin(q3) - cm5_z*g*m5*cos(q3)*cos(q5)*sin(q2) - cm6_y*g*m6*cos(q2)*cos(q5)*sin(q3) - cm6_y*g*m6*cos(q3)*cos(q5)*sin(q2) - cm4_x*g*m4*cos(q4)*sin(q2)*sin(q3) + cm5_x*g*m5*cos(q2)*sin(q3)*sin(q5) + cm5_x*g*m5*cos(q3)*sin(q2)*sin(q5) + cm4_z*g*m4*sin(q2)*sin(q3)*sin(q4) - cm5_y*g*m5*sin(q2)*sin(q3)*sin(q4) + cm5_x*g*m5*cos(q4)*cos(q5)*sin(q2)*sin(q3) - cm6_z*g*m6*cos(q2)*cos(q3)*sin(q4)*sin(q6) - cm6_z*g*m6*cos(q2)*cos(q6)*sin(q3)*sin(q5) - cm6_z*g*m6*cos(q3)*cos(q6)*sin(q2)*sin(q5) + cm7_x*g*m7*cos(q2)*cos(q5)*sin(q3)*sin(q7) + cm7_x*g*m7*cos(q3)*cos(q5)*sin(q2)*sin(q7) - cm7_z*g*m7*cos(q2)*cos(q3)*sin(q4)*sin(q6) - cm7_z*g*m7*cos(q2)*cos(q6)*sin(q3)*sin(q5) - cm7_z*g*m7*cos(q3)*cos(q6)*sin(q2)*sin(q5) + cm5_z*g*m5*cos(q4)*sin(q2)*sin(q3)*sin(q5) + cm6_x*g*m6*cos(q6)*sin(q2)*sin(q3)*sin(q4) + cm6_y*g*m6*cos(q4)*sin(q2)*sin(q3)*sin(q5) + cm6_x*g*m6*cos(q2)*sin(q3)*sin(q5)*sin(q6) + cm6_x*g*m6*cos(q3)*sin(q2)*sin(q5)*sin(q6) + cm6_z*g*m6*sin(q2)*sin(q3)*sin(q4)*sin(q6) + cm7_z*g*m7*sin(q2)*sin(q3)*sin(q4)*sin(q6) - cm5_x*g*m5*cos(q2)*cos(q3)*cos(q4)*cos(q5) - cm5_z*g*m5*cos(q2)*cos(q3)*cos(q4)*sin(q5) - cm6_x*g*m6*cos(q2)*cos(q3)*cos(q6)*sin(q4) - cm6_y*g*m6*cos(q2)*cos(q3)*cos(q4)*sin(q5) - cm7_y*g*m7*cos(q2)*cos(q5)*cos(q7)*sin(q3) - cm7_y*g*m7*cos(q3)*cos(q5)*cos(q7)*sin(q2) + cm6_z*g*m6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) + cm7_z*g*m7*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) - cm6_x*g*m6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) - cm7_x*g*m7*cos(q2)*cos(q3)*cos(q6)*cos(q7)*sin(q4) - cm7_y*g*m7*cos(q2)*cos(q3)*cos(q4)*cos(q7)*sin(q5) - cm6_z*g*m6*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) + cm7_x*g*m7*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q7) - cm7_z*g*m7*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) - cm7_y*g*m7*cos(q2)*cos(q3)*cos(q6)*sin(q4)*sin(q7) + cm6_x*g*m6*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) + cm7_x*g*m7*cos(q6)*cos(q7)*sin(q2)*sin(q3)*sin(q4) + cm7_y*g*m7*cos(q4)*cos(q7)*sin(q2)*sin(q3)*sin(q5) + cm7_x*g*m7*cos(q2)*cos(q7)*sin(q3)*sin(q5)*sin(q6) + cm7_x*g*m7*cos(q3)*cos(q7)*sin(q2)*sin(q5)*sin(q6) - cm7_x*g*m7*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q7) + cm7_y*g*m7*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q7) + cm7_y*g*m7*cos(q2)*sin(q3)*sin(q5)*sin(q6)*sin(q7) + cm7_y*g*m7*cos(q3)*sin(q2)*sin(q5)*sin(q6)*sin(q7) - cm7_x*g*m7*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q7)*sin(q6) - cm7_y*g*m7*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6)*sin(q7) + cm7_x*g*m7*cos(q4)*cos(q5)*cos(q7)*sin(q2)*sin(q3)*sin(q6) + cm7_y*g*m7*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6)*sin(q7);
Torque(3)=L3*g*m3*cos(q2)*cos(q3) + L3*g*m4*cos(q2)*cos(q3) + L3*g*m5*cos(q2)*cos(q3) + L3*g*m6*cos(q2)*cos(q3) + L3*g*m7*cos(q2)*cos(q3) - L4_z0*g*m4*cos(q2)*sin(q3) - L4_z0*g*m4*cos(q3)*sin(q2) - L4_z0*g*m5*cos(q2)*sin(q3) - L4_z0*g*m5*cos(q3)*sin(q2) - L4_z0*g*m6*cos(q2)*sin(q3) - L4_z0*g*m6*cos(q3)*sin(q2) - L4_z0*g*m7*cos(q2)*sin(q3) - L4_z0*g*m7*cos(q3)*sin(q2) + cm3_x*g*m3*cos(q2)*cos(q3) - L3*g*m3*sin(q2)*sin(q3) - L3*g*m4*sin(q2)*sin(q3) - L3*g*m5*sin(q2)*sin(q3) - L3*g*m6*sin(q2)*sin(q3) - L3*g*m7*sin(q2)*sin(q3) - cm3_z*g*m3*cos(q2)*sin(q3) - cm3_z*g*m3*cos(q3)*sin(q2) + cm4_y*g*m4*cos(q2)*sin(q3) + cm4_y*g*m4*cos(q3)*sin(q2) - cm3_x*g*m3*sin(q2)*sin(q3) + cm4_x*g*m4*cos(q2)*cos(q3)*cos(q4) - cm4_z*g*m4*cos(q2)*cos(q3)*sin(q4) + cm5_y*g*m5*cos(q2)*cos(q3)*sin(q4) - cm5_z*g*m5*cos(q2)*cos(q5)*sin(q3) - cm5_z*g*m5*cos(q3)*cos(q5)*sin(q2) - cm6_y*g*m6*cos(q2)*cos(q5)*sin(q3) - cm6_y*g*m6*cos(q3)*cos(q5)*sin(q2) - cm4_x*g*m4*cos(q4)*sin(q2)*sin(q3) + cm5_x*g*m5*cos(q2)*sin(q3)*sin(q5) + cm5_x*g*m5*cos(q3)*sin(q2)*sin(q5) + cm4_z*g*m4*sin(q2)*sin(q3)*sin(q4) - cm5_y*g*m5*sin(q2)*sin(q3)*sin(q4) + cm5_x*g*m5*cos(q4)*cos(q5)*sin(q2)*sin(q3) - cm6_z*g*m6*cos(q2)*cos(q3)*sin(q4)*sin(q6) - cm6_z*g*m6*cos(q2)*cos(q6)*sin(q3)*sin(q5) - cm6_z*g*m6*cos(q3)*cos(q6)*sin(q2)*sin(q5) + cm7_x*g*m7*cos(q2)*cos(q5)*sin(q3)*sin(q7) + cm7_x*g*m7*cos(q3)*cos(q5)*sin(q2)*sin(q7) - cm7_z*g*m7*cos(q2)*cos(q3)*sin(q4)*sin(q6) - cm7_z*g*m7*cos(q2)*cos(q6)*sin(q3)*sin(q5) - cm7_z*g*m7*cos(q3)*cos(q6)*sin(q2)*sin(q5) + cm5_z*g*m5*cos(q4)*sin(q2)*sin(q3)*sin(q5) + cm6_x*g*m6*cos(q6)*sin(q2)*sin(q3)*sin(q4) + cm6_y*g*m6*cos(q4)*sin(q2)*sin(q3)*sin(q5) + cm6_x*g*m6*cos(q2)*sin(q3)*sin(q5)*sin(q6) + cm6_x*g*m6*cos(q3)*sin(q2)*sin(q5)*sin(q6) + cm6_z*g*m6*sin(q2)*sin(q3)*sin(q4)*sin(q6) + cm7_z*g*m7*sin(q2)*sin(q3)*sin(q4)*sin(q6) - cm5_x*g*m5*cos(q2)*cos(q3)*cos(q4)*cos(q5) - cm5_z*g*m5*cos(q2)*cos(q3)*cos(q4)*sin(q5) - cm6_x*g*m6*cos(q2)*cos(q3)*cos(q6)*sin(q4) - cm6_y*g*m6*cos(q2)*cos(q3)*cos(q4)*sin(q5) - cm7_y*g*m7*cos(q2)*cos(q5)*cos(q7)*sin(q3) - cm7_y*g*m7*cos(q3)*cos(q5)*cos(q7)*sin(q2) + cm6_z*g*m6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) + cm7_z*g*m7*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) - cm6_x*g*m6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) - cm7_x*g*m7*cos(q2)*cos(q3)*cos(q6)*cos(q7)*sin(q4) - cm7_y*g*m7*cos(q2)*cos(q3)*cos(q4)*cos(q7)*sin(q5) - cm6_z*g*m6*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) + cm7_x*g*m7*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q7) - cm7_z*g*m7*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) - cm7_y*g*m7*cos(q2)*cos(q3)*cos(q6)*sin(q4)*sin(q7) + cm6_x*g*m6*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) + cm7_x*g*m7*cos(q6)*cos(q7)*sin(q2)*sin(q3)*sin(q4) + cm7_y*g*m7*cos(q4)*cos(q7)*sin(q2)*sin(q3)*sin(q5) + cm7_x*g*m7*cos(q2)*cos(q7)*sin(q3)*sin(q5)*sin(q6) + cm7_x*g*m7*cos(q3)*cos(q7)*sin(q2)*sin(q5)*sin(q6) - cm7_x*g*m7*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q7) + cm7_y*g*m7*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q7) + cm7_y*g*m7*cos(q2)*sin(q3)*sin(q5)*sin(q6)*sin(q7) + cm7_y*g*m7*cos(q3)*sin(q2)*sin(q5)*sin(q6)*sin(q7) - cm7_x*g*m7*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q7)*sin(q6) - cm7_y*g*m7*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6)*sin(q7) + cm7_x*g*m7*cos(q4)*cos(q5)*cos(q7)*sin(q2)*sin(q3)*sin(q6) + cm7_y*g*m7*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6)*sin(q7);
Torque(4)=-g*sin(q2 + q3)*(cm4_z*m4*cos(q4) - cm5_y*m5*cos(q4) + cm4_x*m4*sin(q4) + cm6_x*m6*cos(q4)*cos(q6) - cm5_x*m5*cos(q5)*sin(q4) + cm6_z*m6*cos(q4)*sin(q6) + cm7_z*m7*cos(q4)*sin(q6) - cm5_z*m5*sin(q4)*sin(q5) - cm6_y*m6*sin(q4)*sin(q5) + cm7_x*m7*cos(q4)*cos(q6)*cos(q7) + cm6_z*m6*cos(q5)*cos(q6)*sin(q4) + cm7_z*m7*cos(q5)*cos(q6)*sin(q4) + cm7_y*m7*cos(q4)*cos(q6)*sin(q7) - cm6_x*m6*cos(q5)*sin(q4)*sin(q6) - cm7_y*m7*cos(q7)*sin(q4)*sin(q5) + cm7_x*m7*sin(q4)*sin(q5)*sin(q7) - cm7_x*m7*cos(q5)*cos(q7)*sin(q4)*sin(q6) - cm7_y*m7*cos(q5)*sin(q4)*sin(q6)*sin(q7));
Torque(5)=g*m7*(cm7_z*cos(q2)*cos(q3)*cos(q5)*cos(q6) - cm7_y*cos(q2)*cos(q3)*cos(q7)*sin(q5) + cm7_x*cos(q2)*cos(q3)*sin(q5)*sin(q7) - cm7_z*cos(q5)*cos(q6)*sin(q2)*sin(q3) + cm7_y*cos(q7)*sin(q2)*sin(q3)*sin(q5) - cm7_x*sin(q2)*sin(q3)*sin(q5)*sin(q7) + cm7_x*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q7) + cm7_x*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q7) - cm7_z*cos(q2)*cos(q4)*cos(q6)*sin(q3)*sin(q5) - cm7_z*cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5) - cm7_y*cos(q2)*cos(q3)*cos(q5)*sin(q6)*sin(q7) + cm7_x*cos(q5)*cos(q7)*sin(q2)*sin(q3)*sin(q6) + cm7_y*cos(q5)*sin(q2)*sin(q3)*sin(q6)*sin(q7) - cm7_y*cos(q2)*cos(q4)*cos(q5)*cos(q7)*sin(q3) - cm7_y*cos(q3)*cos(q4)*cos(q5)*cos(q7)*sin(q2) - cm7_x*cos(q2)*cos(q3)*cos(q5)*cos(q7)*sin(q6) + cm7_x*cos(q2)*cos(q4)*cos(q7)*sin(q3)*sin(q5)*sin(q6) + cm7_x*cos(q3)*cos(q4)*cos(q7)*sin(q2)*sin(q5)*sin(q6) + cm7_y*cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q6)*sin(q7) + cm7_y*cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6)*sin(q7)) - g*m5*(cm5_x*cos(q2)*cos(q3)*cos(q5) + cm5_z*cos(q2)*cos(q3)*sin(q5) - cm5_x*cos(q5)*sin(q2)*sin(q3) - cm5_z*sin(q2)*sin(q3)*sin(q5) + cm5_z*cos(q2)*cos(q4)*cos(q5)*sin(q3) + cm5_z*cos(q3)*cos(q4)*cos(q5)*sin(q2) - cm5_x*cos(q2)*cos(q4)*sin(q3)*sin(q5) - cm5_x*cos(q3)*cos(q4)*sin(q2)*sin(q5)) - g*m6*(cm6_y*cos(q2)*cos(q3)*sin(q5) - cm6_y*sin(q2)*sin(q3)*sin(q5) - cm6_z*cos(q2)*cos(q3)*cos(q5)*cos(q6) + cm6_y*cos(q2)*cos(q4)*cos(q5)*sin(q3) + cm6_y*cos(q3)*cos(q4)*cos(q5)*sin(q2) + cm6_x*cos(q2)*cos(q3)*cos(q5)*sin(q6) + cm6_z*cos(q5)*cos(q6)*sin(q2)*sin(q3) - cm6_x*cos(q5)*sin(q2)*sin(q3)*sin(q6) + cm6_z*cos(q2)*cos(q4)*cos(q6)*sin(q3)*sin(q5) + cm6_z*cos(q3)*cos(q4)*cos(q6)*sin(q2)*sin(q5) - cm6_x*cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q6) - cm6_x*cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q6));
Torque(6)=- g*m6*(cm6_x*cos(q2)*cos(q3)*cos(q6)*sin(q5) + cm6_z*cos(q2)*cos(q6)*sin(q3)*sin(q4) + cm6_z*cos(q3)*cos(q6)*sin(q2)*sin(q4) + cm6_z*cos(q2)*cos(q3)*sin(q5)*sin(q6) - cm6_x*cos(q2)*sin(q3)*sin(q4)*sin(q6) - cm6_x*cos(q3)*sin(q2)*sin(q4)*sin(q6) - cm6_x*cos(q6)*sin(q2)*sin(q3)*sin(q5) - cm6_z*sin(q2)*sin(q3)*sin(q5)*sin(q6) + cm6_z*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + cm6_z*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + cm6_x*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + cm6_x*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)) - g*m7*(cm7_z*cos(q2)*cos(q6)*sin(q3)*sin(q4) + cm7_z*cos(q3)*cos(q6)*sin(q2)*sin(q4) + cm7_z*cos(q2)*cos(q3)*sin(q5)*sin(q6) - cm7_z*sin(q2)*sin(q3)*sin(q5)*sin(q6) + cm7_z*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + cm7_z*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) + cm7_y*cos(q2)*cos(q3)*cos(q6)*sin(q5)*sin(q7) - cm7_x*cos(q2)*cos(q7)*sin(q3)*sin(q4)*sin(q6) - cm7_x*cos(q3)*cos(q7)*sin(q2)*sin(q4)*sin(q6) - cm7_x*cos(q6)*cos(q7)*sin(q2)*sin(q3)*sin(q5) - cm7_y*cos(q2)*sin(q3)*sin(q4)*sin(q6)*sin(q7) - cm7_y*cos(q3)*sin(q2)*sin(q4)*sin(q6)*sin(q7) - cm7_y*cos(q6)*sin(q2)*sin(q3)*sin(q5)*sin(q7) + cm7_x*cos(q2)*cos(q3)*cos(q6)*cos(q7)*sin(q5) + cm7_x*cos(q2)*cos(q4)*cos(q5)*cos(q6)*cos(q7)*sin(q3) + cm7_x*cos(q3)*cos(q4)*cos(q5)*cos(q6)*cos(q7)*sin(q2) + cm7_y*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3)*sin(q7) + cm7_y*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q7));
Torque(7)=g*m7*(cm7_x*cos(q5)*cos(q7)*sin(q2)*sin(q3) - cm7_y*cos(q2)*cos(q3)*cos(q5)*sin(q7) - cm7_x*cos(q2)*cos(q3)*cos(q5)*cos(q7) + cm7_y*cos(q5)*sin(q2)*sin(q3)*sin(q7) + cm7_x*cos(q2)*cos(q4)*cos(q7)*sin(q3)*sin(q5) + cm7_x*cos(q3)*cos(q4)*cos(q7)*sin(q2)*sin(q5) - cm7_y*cos(q2)*cos(q6)*cos(q7)*sin(q3)*sin(q4) - cm7_y*cos(q3)*cos(q6)*cos(q7)*sin(q2)*sin(q4) - cm7_y*cos(q2)*cos(q3)*cos(q7)*sin(q5)*sin(q6) + cm7_x*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q7) + cm7_x*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q7) + cm7_y*cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q7) + cm7_y*cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q7) + cm7_x*cos(q2)*cos(q3)*sin(q5)*sin(q6)*sin(q7) + cm7_y*cos(q7)*sin(q2)*sin(q3)*sin(q5)*sin(q6) - cm7_x*sin(q2)*sin(q3)*sin(q5)*sin(q6)*sin(q7) - cm7_y*cos(q2)*cos(q4)*cos(q5)*cos(q7)*sin(q3)*sin(q6) - cm7_y*cos(q3)*cos(q4)*cos(q5)*cos(q7)*sin(q2)*sin(q6) + cm7_x*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6)*sin(q7) + cm7_x*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)*sin(q7));

Torque;

end