cm1_x = 0;
cm1_y = .195/2;
cm1_z = 0;
cm2_x = -.285/2;
cm2_y = 0;
cm2_z = 0;
cm3_x = -.37/2;
cm3_y = 0;
cm3_z = 0;
cm4_x = 0;
cm4_y = (3/4)*.10;
cm4_z = (1/4)*-.095;
cm5_x = 0;
cm5_y = (3/4)*-.06;
cm5_z = (1/4)*.06;
cm6_x = 0;
cm6_y = (3/4)*-.045;
cm6_z = (1/4)*.045;
cm7_x = 0;
cm7_y = 0;
cm7_z = 0;
g = 9.81;
m1 = .8;
m2 = .10;
m3 = .10;
m4 = .05;
m5 = .05;
m6 = .05;
m7 = 0;
q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;
q5 = 0;
q6 = 0;
q7 = 0;

Torque = zeros(7,1);

error = [0];

while error(i-1)<error(i) 
    Torque(1) = (g*(200*cm1_x*m1*cos(q1) - 200*cm1_z*m1*sin(q1) - 200*cm2_z*m2*sin(q1) - 200*cm3_y*m3*sin(q1) - 57*m2*cos(q1)*sin(q2) - 57*m3*cos(q1)*sin(q2) - 57*m4*cos(q1)*sin(q2) - 57*m5*cos(q1)*sin(q2) - 57*m6*cos(q1)*sin(q2) + 19*m5*cos(q4)*sin(q1) - 57*m7*cos(q1)*sin(q2) + 19*m6*cos(q4)*sin(q1) + 19*m7*cos(q4)*sin(q1) - 74*m3*cos(q1)*cos(q2)*cos(q3) - 74*m4*cos(q1)*cos(q2)*cos(q3) - 74*m5*cos(q1)*cos(q2)*cos(q3) - 74*m6*cos(q1)*cos(q2)*cos(q3) - 74*m7*cos(q1)*cos(q2)*cos(q3) + 20*m4*cos(q1)*cos(q2)*sin(q3) + 20*m4*cos(q1)*cos(q3)*sin(q2) + 20*m5*cos(q1)*cos(q2)*sin(q3) + 20*m5*cos(q1)*cos(q3)*sin(q2) + 20*m6*cos(q1)*cos(q2)*sin(q3) + 20*m6*cos(q1)*cos(q3)*sin(q2) + 20*m7*cos(q1)*cos(q2)*sin(q3) + 20*m7*cos(q1)*cos(q3)*sin(q2) + 74*m3*cos(q1)*sin(q2)*sin(q3) + 74*m4*cos(q1)*sin(q2)*sin(q3) + 74*m5*cos(q1)*sin(q2)*sin(q3) + 74*m6*cos(q1)*sin(q2)*sin(q3) + 74*m7*cos(q1)*sin(q2)*sin(q3) - 9*m7*cos(q4)*sin(q1)*sin(q6) + 9*m6*sin(q1)*sin(q4)*sin(q5) + 9*m7*sin(q1)*sin(q4)*sin(q5) + 200*cm2_y*m2*cos(q1)*cos(q2) - 200*cm2_x*m2*cos(q1)*sin(q2) - 200*cm4_z*m4*cos(q4)*sin(q1) + 200*cm5_y*m5*cos(q4)*sin(q1) - 200*cm4_x*m4*sin(q1)*sin(q4) - 19*m5*cos(q1)*cos(q2)*cos(q3)*sin(q4) - 19*m6*cos(q1)*cos(q2)*cos(q3)*sin(q4) + 9*m6*cos(q1)*cos(q2)*cos(q5)*sin(q3) + 9*m6*cos(q1)*cos(q3)*cos(q5)*sin(q2) - 19*m7*cos(q1)*cos(q2)*cos(q3)*sin(q4) + 9*m7*cos(q1)*cos(q2)*cos(q5)*sin(q3) + 9*m7*cos(q1)*cos(q3)*cos(q5)*sin(q2) + 9*m7*cos(q5)*cos(q6)*sin(q1)*sin(q4) + 19*m5*cos(q1)*sin(q2)*sin(q3)*sin(q4) + 19*m6*cos(q1)*sin(q2)*sin(q3)*sin(q4) + 19*m7*cos(q1)*sin(q2)*sin(q3)*sin(q4) - 200*cm3_x*m3*cos(q1)*cos(q2)*cos(q3) + 200*cm3_z*m3*cos(q1)*cos(q2)*sin(q3) + 200*cm3_z*m3*cos(q1)*cos(q3)*sin(q2) - 200*cm4_y*m4*cos(q1)*cos(q2)*sin(q3) - 200*cm4_y*m4*cos(q1)*cos(q3)*sin(q2) - 200*cm6_x*m6*cos(q4)*cos(q6)*sin(q1) + 200*cm3_x*m3*cos(q1)*sin(q2)*sin(q3) + 200*cm5_x*m5*cos(q5)*sin(q1)*sin(q4) + 200*cm6_z*m6*cos(q4)*sin(q1)*sin(q6) + 200*cm7_z*m7*cos(q4)*sin(q1)*sin(q6) + 200*cm5_z*m5*sin(q1)*sin(q4)*sin(q5) + 200*cm6_y*m6*sin(q1)*sin(q4)*sin(q5) + 9*m6*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + 9*m7*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + 9*m7*cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q6) - 9*m7*cos(q1)*cos(q2)*cos(q6)*sin(q3)*sin(q5) - 9*m7*cos(q1)*cos(q3)*cos(q6)*sin(q2)*sin(q5) - 9*m6*cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - 9*m7*cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - 9*m7*cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - 200*cm4_x*m4*cos(q1)*cos(q2)*cos(q3)*cos(q4) + 200*cm4_z*m4*cos(q1)*cos(q2)*cos(q3)*sin(q4) - 200*cm5_y*m5*cos(q1)*cos(q2)*cos(q3)*sin(q4) + 200*cm5_z*m5*cos(q1)*cos(q2)*cos(q5)*sin(q3) + 200*cm5_z*m5*cos(q1)*cos(q3)*cos(q5)*sin(q2) + 200*cm6_y*m6*cos(q1)*cos(q2)*cos(q5)*sin(q3) + 200*cm6_y*m6*cos(q1)*cos(q3)*cos(q5)*sin(q2) - 200*cm7_x*m7*cos(q4)*cos(q6)*cos(q7)*sin(q1) + 200*cm4_x*m4*cos(q1)*cos(q4)*sin(q2)*sin(q3) - 200*cm5_x*m5*cos(q1)*cos(q2)*sin(q3)*sin(q5) - 200*cm5_x*m5*cos(q1)*cos(q3)*sin(q2)*sin(q5) - 200*cm6_z*m6*cos(q5)*cos(q6)*sin(q1)*sin(q4) - 200*cm7_z*m7*cos(q5)*cos(q6)*sin(q1)*sin(q4) + 200*cm7_y*m7*cos(q4)*cos(q6)*sin(q1)*sin(q7) - 200*cm4_z*m4*cos(q1)*sin(q2)*sin(q3)*sin(q4) + 200*cm5_y*m5*cos(q1)*sin(q2)*sin(q3)*sin(q4) - 200*cm6_x*m6*cos(q5)*sin(q1)*sin(q4)*sin(q6) + 200*cm7_y*m7*cos(q7)*sin(q1)*sin(q4)*sin(q5) + 200*cm7_x*m7*sin(q1)*sin(q4)*sin(q5)*sin(q7) + 200*cm5_x*m5*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5) + 200*cm5_z*m5*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + 200*cm6_x*m6*cos(q1)*cos(q2)*cos(q3)*cos(q6)*sin(q4) + 200*cm6_y*m6*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5) + 200*cm7_y*m7*cos(q1)*cos(q2)*cos(q5)*cos(q7)*sin(q3) + 200*cm7_y*m7*cos(q1)*cos(q3)*cos(q5)*cos(q7)*sin(q2) - 200*cm5_x*m5*cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3) - 200*cm6_z*m6*cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q6) + 200*cm6_z*m6*cos(q1)*cos(q2)*cos(q6)*sin(q3)*sin(q5) + 200*cm6_z*m6*cos(q1)*cos(q3)*cos(q6)*sin(q2)*sin(q5) + 200*cm7_x*m7*cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q7) + 200*cm7_x*m7*cos(q1)*cos(q3)*cos(q5)*sin(q2)*sin(q7) - 200*cm7_z*m7*cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q6) + 200*cm7_z*m7*cos(q1)*cos(q2)*cos(q6)*sin(q3)*sin(q5) + 200*cm7_z*m7*cos(q1)*cos(q3)*cos(q6)*sin(q2)*sin(q5) - 200*cm5_z*m5*cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5) - 200*cm6_x*m6*cos(q1)*cos(q6)*sin(q2)*sin(q3)*sin(q4) - 200*cm6_y*m6*cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5) + 200*cm6_x*m6*cos(q1)*cos(q2)*sin(q3)*sin(q5)*sin(q6) + 200*cm6_x*m6*cos(q1)*cos(q3)*sin(q2)*sin(q5)*sin(q6) - 200*cm7_x*m7*cos(q5)*cos(q7)*sin(q1)*sin(q4)*sin(q6) + 200*cm6_z*m6*cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + 200*cm7_z*m7*cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + 200*cm7_y*m7*cos(q5)*sin(q1)*sin(q4)*sin(q6)*sin(q7) + 9*m7*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) - 9*m7*cos(q1)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) - 200*cm6_z*m6*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) - 200*cm7_z*m7*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6) - 200*cm6_x*m6*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6) + 200*cm7_x*m7*cos(q1)*cos(q2)*cos(q3)*cos(q6)*cos(q7)*sin(q4) + 200*cm7_y*m7*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q7)*sin(q5) + 200*cm6_z*m6*cos(q1)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) + 200*cm7_x*m7*cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5)*sin(q7) + 200*cm7_z*m7*cos(q1)*cos(q4)*cos(q5)*cos(q6)*sin(q2)*sin(q3) - 200*cm7_y*m7*cos(q1)*cos(q2)*cos(q3)*cos(q6)*sin(q4)*sin(q7) + 200*cm6_x*m6*cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6) - 200*cm7_x*m7*cos(q1)*cos(q6)*cos(q7)*sin(q2)*sin(q3)*sin(q4) - 200*cm7_y*m7*cos(q1)*cos(q4)*cos(q7)*sin(q2)*sin(q3)*sin(q5) + 200*cm7_x*m7*cos(q1)*cos(q2)*cos(q7)*sin(q3)*sin(q5)*sin(q6) + 200*cm7_x*m7*cos(q1)*cos(q3)*cos(q7)*sin(q2)*sin(q5)*sin(q6) - 200*cm7_x*m7*cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5)*sin(q7) + 200*cm7_y*m7*cos(q1)*cos(q6)*sin(q2)*sin(q3)*sin(q4)*sin(q7) - 200*cm7_y*m7*cos(q1)*cos(q2)*sin(q3)*sin(q5)*sin(q6)*sin(q7) - 200*cm7_y*m7*cos(q1)*cos(q3)*sin(q2)*sin(q5)*sin(q6)*sin(q7) - 200*cm7_x*m7*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q7)*sin(q6) + 200*cm7_y*m7*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q6)*sin(q7) + 200*cm7_x*m7*cos(q1)*cos(q4)*cos(q5)*cos(q7)*sin(q2)*sin(q3)*sin(q6) - 200*cm7_y*m7*cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*sin(q6)*sin(q7)))/200;
    Torque(2) = (g*sin(q1)*(20*m4*cos(q2)*cos(q3) - 57*m3*cos(q2) - 57*m4*cos(q2) - 57*m5*cos(q2) - 57*m6*cos(q2) - 57*m7*cos(q2) - 200*cm2_x*m2*cos(q2) - 200*cm2_y*m2*sin(q2) - 57*m2*cos(q2) + 20*m5*cos(q2)*cos(q3) + 20*m6*cos(q2)*cos(q3) + 20*m7*cos(q2)*cos(q3) + 74*m3*cos(q2)*sin(q3) + 74*m3*cos(q3)*sin(q2) + 74*m4*cos(q2)*sin(q3) + 74*m4*cos(q3)*sin(q2) + 74*m5*cos(q2)*sin(q3) + 74*m5*cos(q3)*sin(q2) + 74*m6*cos(q2)*sin(q3) + 74*m6*cos(q3)*sin(q2) + 74*m7*cos(q2)*sin(q3) + 74*m7*cos(q3)*sin(q2) - 20*m4*sin(q2)*sin(q3) - 20*m5*sin(q2)*sin(q3) - 20*m6*sin(q2)*sin(q3) - 20*m7*sin(q2)*sin(q3) + 9*m6*cos(q2)*cos(q3)*cos(q5) + 9*m7*cos(q2)*cos(q3)*cos(q5) + 19*m5*cos(q2)*sin(q3)*sin(q4) + 19*m5*cos(q3)*sin(q2)*sin(q4) + 19*m6*cos(q2)*sin(q3)*sin(q4) + 19*m6*cos(q3)*sin(q2)*sin(q4) - 9*m6*cos(q5)*sin(q2)*sin(q3) + 19*m7*cos(q2)*sin(q3)*sin(q4) + 19*m7*cos(q3)*sin(q2)*sin(q4) - 9*m7*cos(q5)*sin(q2)*sin(q3) + 200*cm3_z*m3*cos(q2)*cos(q3) - 200*cm4_y*m4*cos(q2)*cos(q3) + 200*cm3_x*m3*cos(q2)*sin(q3) + 200*cm3_x*m3*cos(q3)*sin(q2) - 200*cm3_z*m3*sin(q2)*sin(q3) + 200*cm4_y*m4*sin(q2)*sin(q3) - 9*m7*cos(q2)*cos(q3)*cos(q6)*sin(q5) - 9*m6*cos(q2)*cos(q4)*sin(q3)*sin(q5) - 9*m6*cos(q3)*cos(q4)*sin(q2)*sin(q5) - 9*m7*cos(q2)*cos(q4)*sin(q3)*sin(q5) - 9*m7*cos(q3)*cos(q4)*sin(q2)*sin(q5) - 9*m7*cos(q2)*sin(q3)*sin(q4)*sin(q6) - 9*m7*cos(q3)*sin(q2)*sin(q4)*sin(q6) + 9*m7*cos(q6)*sin(q2)*sin(q3)*sin(q5) + 200*cm5_z*m5*cos(q2)*cos(q3)*cos(q5) + 200*cm6_y*m6*cos(q2)*cos(q3)*cos(q5) + 200*cm4_x*m4*cos(q2)*cos(q4)*sin(q3) + 200*cm4_x*m4*cos(q3)*cos(q4)*sin(q2) - 200*cm5_x*m5*cos(q2)*cos(q3)*sin(q5) - 200*cm4_z*m4*cos(q2)*sin(q3)*sin(q4) - 200*cm4_z*m4*cos(q3)*sin(q2)*sin(q4) + 200*cm5_y*m5*cos(q2)*sin(q3)*sin(q4) + 200*cm5_y*m5*cos(q3)*sin(q2)*sin(q4) - 200*cm5_z*m5*cos(q5)*sin(q2)*sin(q3) - 200*cm6_y*m6*cos(q5)*sin(q2)*sin(q3) + 200*cm5_x*m5*sin(q2)*sin(q3)*sin(q5) - 9*m7*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) - 9*m7*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + 200*cm7_y*m7*cos(q2)*cos(q3)*cos(q5)*cos(q7) - 200*cm5_x*m5*cos(q2)*cos(q4)*cos(q5)*sin(q3) - 200*cm5_x*m5*cos(q3)*cos(q4)*cos(q5)*sin(q2) + 200*cm6_z*m6*cos(q2)*cos(q3)*cos(q6)*sin(q5) + 200*cm7_x*m7*cos(q2)*cos(q3)*cos(q5)*sin(q7) + 200*cm7_z*m7*cos(q2)*cos(q3)*cos(q6)*sin(q5) - 200*cm5_z*m5*cos(q2)*cos(q4)*sin(q3)*sin(q5) - 200*cm5_z*m5*cos(q3)*cos(q4)*sin(q2)*sin(q5) - 200*cm6_x*m6*cos(q2)*cos(q6)*sin(q3)*sin(q4) - 200*cm6_x*m6*cos(q3)*cos(q6)*sin(q2)*sin(q4) - 200*cm6_y*m6*cos(q2)*cos(q4)*sin(q3)*sin(q5) - 200*cm6_y*m6*cos(q3)*cos(q4)*sin(q2)*sin(q5) + 200*cm6_x*m6*cos(q2)*cos(q3)*sin(q5)*sin(q6) - 200*cm7_y*m7*cos(q5)*cos(q7)*sin(q2)*sin(q3) + 200*cm6_z*m6*cos(q2)*sin(q3)*sin(q4)*sin(q6) + 200*cm6_z*m6*cos(q3)*sin(q2)*sin(q4)*sin(q6) - 200*cm6_z*m6*cos(q6)*sin(q2)*sin(q3)*sin(q5) - 200*cm7_x*m7*cos(q5)*sin(q2)*sin(q3)*sin(q7) + 200*cm7_z*m7*cos(q2)*sin(q3)*sin(q4)*sin(q6) + 200*cm7_z*m7*cos(q3)*sin(q2)*sin(q4)*sin(q6) - 200*cm7_z*m7*cos(q6)*sin(q2)*sin(q3)*sin(q5) - 200*cm6_x*m6*sin(q2)*sin(q3)*sin(q5)*sin(q6) + 200*cm6_z*m6*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + 200*cm6_z*m6*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + 200*cm7_z*m7*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + 200*cm7_z*m7*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + 200*cm6_x*m6*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + 200*cm6_x*m6*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) - 200*cm7_x*m7*cos(q2)*cos(q6)*cos(q7)*sin(q3)*sin(q4) - 200*cm7_x*m7*cos(q3)*cos(q6)*cos(q7)*sin(q2)*sin(q4) - 200*cm7_y*m7*cos(q2)*cos(q4)*cos(q7)*sin(q3)*sin(q5) - 200*cm7_y*m7*cos(q3)*cos(q4)*cos(q7)*sin(q2)*sin(q5) + 200*cm7_x*m7*cos(q2)*cos(q3)*cos(q7)*sin(q5)*sin(q6) - 200*cm7_x*m7*cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q7) - 200*cm7_x*m7*cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q7) + 200*cm7_y*m7*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q7) + 200*cm7_y*m7*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q7) - 200*cm7_y*m7*cos(q2)*cos(q3)*sin(q5)*sin(q6)*sin(q7) - 200*cm7_x*m7*cos(q7)*sin(q2)*sin(q3)*sin(q5)*sin(q6) + 200*cm7_y*m7*sin(q2)*sin(q3)*sin(q5)*sin(q6)*sin(q7) + 200*cm7_x*m7*cos(q2)*cos(q4)*cos(q5)*cos(q7)*sin(q3)*sin(q6) + 200*cm7_x*m7*cos(q3)*cos(q4)*cos(q5)*cos(q7)*sin(q2)*sin(q6) - 200*cm7_y*m7*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6)*sin(q7) - 200*cm7_y*m7*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)*sin(q7)))/200;
    Torque(3) = (g*sin(q1)*(20*m4*cos(q2)*cos(q3) + 20*m5*cos(q2)*cos(q3) + 20*m6*cos(q2)*cos(q3) + 20*m7*cos(q2)*cos(q3) + 74*m3*cos(q2)*sin(q3) + 74*m3*cos(q3)*sin(q2) + 74*m4*cos(q2)*sin(q3) + 74*m4*cos(q3)*sin(q2) + 74*m5*cos(q2)*sin(q3) + 74*m5*cos(q3)*sin(q2) + 74*m6*cos(q2)*sin(q3) + 74*m6*cos(q3)*sin(q2) + 74*m7*cos(q2)*sin(q3) + 74*m7*cos(q3)*sin(q2) - 20*m4*sin(q2)*sin(q3) - 20*m5*sin(q2)*sin(q3) - 20*m6*sin(q2)*sin(q3) - 20*m7*sin(q2)*sin(q3) + 9*m6*cos(q2)*cos(q3)*cos(q5) + 9*m7*cos(q2)*cos(q3)*cos(q5) + 19*m5*cos(q2)*sin(q3)*sin(q4) + 19*m5*cos(q3)*sin(q2)*sin(q4) + 19*m6*cos(q2)*sin(q3)*sin(q4) + 19*m6*cos(q3)*sin(q2)*sin(q4) - 9*m6*cos(q5)*sin(q2)*sin(q3) + 19*m7*cos(q2)*sin(q3)*sin(q4) + 19*m7*cos(q3)*sin(q2)*sin(q4) - 9*m7*cos(q5)*sin(q2)*sin(q3) + 200*cm3_z*m3*cos(q2)*cos(q3) - 200*cm4_y*m4*cos(q2)*cos(q3) + 200*cm3_x*m3*cos(q2)*sin(q3) + 200*cm3_x*m3*cos(q3)*sin(q2) - 200*cm3_z*m3*sin(q2)*sin(q3) + 200*cm4_y*m4*sin(q2)*sin(q3) - 9*m7*cos(q2)*cos(q3)*cos(q6)*sin(q5) - 9*m6*cos(q2)*cos(q4)*sin(q3)*sin(q5) - 9*m6*cos(q3)*cos(q4)*sin(q2)*sin(q5) - 9*m7*cos(q2)*cos(q4)*sin(q3)*sin(q5) - 9*m7*cos(q3)*cos(q4)*sin(q2)*sin(q5) - 9*m7*cos(q2)*sin(q3)*sin(q4)*sin(q6) - 9*m7*cos(q3)*sin(q2)*sin(q4)*sin(q6) + 9*m7*cos(q6)*sin(q2)*sin(q3)*sin(q5) + 200*cm5_z*m5*cos(q2)*cos(q3)*cos(q5) + 200*cm6_y*m6*cos(q2)*cos(q3)*cos(q5) + 200*cm4_x*m4*cos(q2)*cos(q4)*sin(q3) + 200*cm4_x*m4*cos(q3)*cos(q4)*sin(q2) - 200*cm5_x*m5*cos(q2)*cos(q3)*sin(q5) - 200*cm4_z*m4*cos(q2)*sin(q3)*sin(q4) - 200*cm4_z*m4*cos(q3)*sin(q2)*sin(q4) + 200*cm5_y*m5*cos(q2)*sin(q3)*sin(q4) + 200*cm5_y*m5*cos(q3)*sin(q2)*sin(q4) - 200*cm5_z*m5*cos(q5)*sin(q2)*sin(q3) - 200*cm6_y*m6*cos(q5)*sin(q2)*sin(q3) + 200*cm5_x*m5*sin(q2)*sin(q3)*sin(q5) - 9*m7*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) - 9*m7*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + 200*cm7_y*m7*cos(q2)*cos(q3)*cos(q5)*cos(q7) - 200*cm5_x*m5*cos(q2)*cos(q4)*cos(q5)*sin(q3) - 200*cm5_x*m5*cos(q3)*cos(q4)*cos(q5)*sin(q2) + 200*cm6_z*m6*cos(q2)*cos(q3)*cos(q6)*sin(q5) + 200*cm7_x*m7*cos(q2)*cos(q3)*cos(q5)*sin(q7) + 200*cm7_z*m7*cos(q2)*cos(q3)*cos(q6)*sin(q5) - 200*cm5_z*m5*cos(q2)*cos(q4)*sin(q3)*sin(q5) - 200*cm5_z*m5*cos(q3)*cos(q4)*sin(q2)*sin(q5) - 200*cm6_x*m6*cos(q2)*cos(q6)*sin(q3)*sin(q4) - 200*cm6_x*m6*cos(q3)*cos(q6)*sin(q2)*sin(q4) - 200*cm6_y*m6*cos(q2)*cos(q4)*sin(q3)*sin(q5) - 200*cm6_y*m6*cos(q3)*cos(q4)*sin(q2)*sin(q5) + 200*cm6_x*m6*cos(q2)*cos(q3)*sin(q5)*sin(q6) - 200*cm7_y*m7*cos(q5)*cos(q7)*sin(q2)*sin(q3) + 200*cm6_z*m6*cos(q2)*sin(q3)*sin(q4)*sin(q6) + 200*cm6_z*m6*cos(q3)*sin(q2)*sin(q4)*sin(q6) - 200*cm6_z*m6*cos(q6)*sin(q2)*sin(q3)*sin(q5) - 200*cm7_x*m7*cos(q5)*sin(q2)*sin(q3)*sin(q7) + 200*cm7_z*m7*cos(q2)*sin(q3)*sin(q4)*sin(q6) + 200*cm7_z*m7*cos(q3)*sin(q2)*sin(q4)*sin(q6) - 200*cm7_z*m7*cos(q6)*sin(q2)*sin(q3)*sin(q5) - 200*cm6_x*m6*sin(q2)*sin(q3)*sin(q5)*sin(q6) + 200*cm6_z*m6*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + 200*cm6_z*m6*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + 200*cm7_z*m7*cos(q2)*cos(q4)*cos(q5)*cos(q6)*sin(q3) + 200*cm7_z*m7*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q2) + 200*cm6_x*m6*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6) + 200*cm6_x*m6*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6) - 200*cm7_x*m7*cos(q2)*cos(q6)*cos(q7)*sin(q3)*sin(q4) - 200*cm7_x*m7*cos(q3)*cos(q6)*cos(q7)*sin(q2)*sin(q4) - 200*cm7_y*m7*cos(q2)*cos(q4)*cos(q7)*sin(q3)*sin(q5) - 200*cm7_y*m7*cos(q3)*cos(q4)*cos(q7)*sin(q2)*sin(q5) + 200*cm7_x*m7*cos(q2)*cos(q3)*cos(q7)*sin(q5)*sin(q6) - 200*cm7_x*m7*cos(q2)*cos(q4)*sin(q3)*sin(q5)*sin(q7) - 200*cm7_x*m7*cos(q3)*cos(q4)*sin(q2)*sin(q5)*sin(q7) + 200*cm7_y*m7*cos(q2)*cos(q6)*sin(q3)*sin(q4)*sin(q7) + 200*cm7_y*m7*cos(q3)*cos(q6)*sin(q2)*sin(q4)*sin(q7) - 200*cm7_y*m7*cos(q2)*cos(q3)*sin(q5)*sin(q6)*sin(q7) - 200*cm7_x*m7*cos(q7)*sin(q2)*sin(q3)*sin(q5)*sin(q6) + 200*cm7_y*m7*sin(q2)*sin(q3)*sin(q5)*sin(q6)*sin(q7) + 200*cm7_x*m7*cos(q2)*cos(q4)*cos(q5)*cos(q7)*sin(q3)*sin(q6) + 200*cm7_x*m7*cos(q3)*cos(q4)*cos(q5)*cos(q7)*sin(q2)*sin(q6) - 200*cm7_y*m7*cos(q2)*cos(q4)*cos(q5)*sin(q3)*sin(q6)*sin(q7) - 200*cm7_y*m7*cos(q3)*cos(q4)*cos(q5)*sin(q2)*sin(q6)*sin(q7)))/200;
    Torque(4) = -(g*(9*m6*cos(q1)*cos(q4)*sin(q5) - 19*m6*cos(q1)*sin(q4) - 19*m7*cos(q1)*sin(q4) - 19*m5*cos(q1)*sin(q4) + 9*m7*cos(q1)*cos(q4)*sin(q5) + 9*m7*cos(q1)*sin(q4)*sin(q6) - 200*cm4_x*m4*cos(q1)*cos(q4) + 200*cm4_z*m4*cos(q1)*sin(q4) - 200*cm5_y*m5*cos(q1)*sin(q4) + 9*m7*cos(q1)*cos(q4)*cos(q5)*cos(q6) + 19*m5*cos(q2)*cos(q3)*cos(q4)*sin(q1) + 19*m6*cos(q2)*cos(q3)*cos(q4)*sin(q1) + 19*m7*cos(q2)*cos(q3)*cos(q4)*sin(q1) - 19*m5*cos(q4)*sin(q1)*sin(q2)*sin(q3) - 19*m6*cos(q4)*sin(q1)*sin(q2)*sin(q3) - 19*m7*cos(q4)*sin(q1)*sin(q2)*sin(q3) + 200*cm5_x*m5*cos(q1)*cos(q4)*cos(q5) + 200*cm5_z*m5*cos(q1)*cos(q4)*sin(q5) + 200*cm6_x*m6*cos(q1)*cos(q6)*sin(q4) + 200*cm6_y*m6*cos(q1)*cos(q4)*sin(q5) - 200*cm6_z*m6*cos(q1)*sin(q4)*sin(q6) - 200*cm7_z*m7*cos(q1)*sin(q4)*sin(q6) - 9*m7*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q6) + 9*m6*cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q5) + 9*m7*cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q5) + 9*m7*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q6) - 9*m6*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 9*m7*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 200*cm6_z*m6*cos(q1)*cos(q4)*cos(q5)*cos(q6) - 200*cm7_z*m7*cos(q1)*cos(q4)*cos(q5)*cos(q6) - 200*cm4_z*m4*cos(q2)*cos(q3)*cos(q4)*sin(q1) + 200*cm5_y*m5*cos(q2)*cos(q3)*cos(q4)*sin(q1) - 200*cm6_x*m6*cos(q1)*cos(q4)*cos(q5)*sin(q6) + 200*cm7_x*m7*cos(q1)*cos(q6)*cos(q7)*sin(q4) + 200*cm7_y*m7*cos(q1)*cos(q4)*cos(q7)*sin(q5) - 200*cm4_x*m4*cos(q2)*cos(q3)*sin(q1)*sin(q4) + 200*cm7_x*m7*cos(q1)*cos(q4)*sin(q5)*sin(q7) - 200*cm7_y*m7*cos(q1)*cos(q6)*sin(q4)*sin(q7) + 200*cm4_z*m4*cos(q4)*sin(q1)*sin(q2)*sin(q3) - 200*cm5_y*m5*cos(q4)*sin(q1)*sin(q2)*sin(q3) + 200*cm4_x*m4*sin(q1)*sin(q2)*sin(q3)*sin(q4) - 9*m7*cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4) - 200*cm6_x*m6*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q1) - 200*cm7_x*m7*cos(q1)*cos(q4)*cos(q5)*cos(q7)*sin(q6) + 200*cm5_x*m5*cos(q2)*cos(q3)*cos(q5)*sin(q1)*sin(q4) + 200*cm6_z*m6*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q6) + 200*cm7_z*m7*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q6) + 200*cm7_y*m7*cos(q1)*cos(q4)*cos(q5)*sin(q6)*sin(q7) + 200*cm5_z*m5*cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q5) + 200*cm6_x*m6*cos(q4)*cos(q6)*sin(q1)*sin(q2)*sin(q3) + 200*cm6_y*m6*cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q5) - 200*cm5_x*m5*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q4) - 200*cm6_z*m6*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q6) - 200*cm7_z*m7*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q6) - 200*cm5_z*m5*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 200*cm6_y*m6*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5) + 9*m7*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q1)*sin(q4) - 200*cm7_x*m7*cos(q2)*cos(q3)*cos(q4)*cos(q6)*cos(q7)*sin(q1) - 200*cm6_z*m6*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q1)*sin(q4) - 200*cm7_z*m7*cos(q2)*cos(q3)*cos(q5)*cos(q6)*sin(q1)*sin(q4) + 200*cm7_y*m7*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q1)*sin(q7) - 200*cm6_x*m6*cos(q2)*cos(q3)*cos(q5)*sin(q1)*sin(q4)*sin(q6) + 200*cm7_x*m7*cos(q4)*cos(q6)*cos(q7)*sin(q1)*sin(q2)*sin(q3) + 200*cm7_y*m7*cos(q2)*cos(q3)*cos(q7)*sin(q1)*sin(q4)*sin(q5) + 200*cm6_z*m6*cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4) + 200*cm7_x*m7*cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q5)*sin(q7) + 200*cm7_z*m7*cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4) - 200*cm7_y*m7*cos(q4)*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q7) + 200*cm6_x*m6*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - 200*cm7_y*m7*cos(q7)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5) - 200*cm7_x*m7*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*sin(q7) - 200*cm7_x*m7*cos(q2)*cos(q3)*cos(q5)*cos(q7)*sin(q1)*sin(q4)*sin(q6) + 200*cm7_y*m7*cos(q2)*cos(q3)*cos(q5)*sin(q1)*sin(q4)*sin(q6)*sin(q7) + 200*cm7_x*m7*cos(q5)*cos(q7)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - 200*cm7_y*m7*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6)*sin(q7)))/200;
    Torque(5) = -(g*(9*m6*cos(q1)*cos(q5)*sin(q4) + 9*m7*cos(q1)*cos(q5)*sin(q4) - 9*m7*cos(q1)*cos(q6)*sin(q4)*sin(q5) + 9*m6*cos(q2)*sin(q1)*sin(q3)*sin(q5) + 9*m6*cos(q3)*sin(q1)*sin(q2)*sin(q5) + 9*m7*cos(q2)*sin(q1)*sin(q3)*sin(q5) + 9*m7*cos(q3)*sin(q1)*sin(q2)*sin(q5) + 200*cm5_z*m5*cos(q1)*cos(q5)*sin(q4) + 200*cm6_y*m6*cos(q1)*cos(q5)*sin(q4) - 200*cm5_x*m5*cos(q1)*sin(q4)*sin(q5) - 9*m6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) - 9*m7*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) + 9*m7*cos(q2)*cos(q5)*cos(q6)*sin(q1)*sin(q3) + 9*m7*cos(q3)*cos(q5)*cos(q6)*sin(q1)*sin(q2) + 9*m6*cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3) + 9*m7*cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3) + 200*cm7_y*m7*cos(q1)*cos(q5)*cos(q7)*sin(q4) + 200*cm5_x*m5*cos(q2)*cos(q5)*sin(q1)*sin(q3) + 200*cm5_x*m5*cos(q3)*cos(q5)*sin(q1)*sin(q2) + 200*cm6_z*m6*cos(q1)*cos(q6)*sin(q4)*sin(q5) + 200*cm7_x*m7*cos(q1)*cos(q5)*sin(q4)*sin(q7) + 200*cm7_z*m7*cos(q1)*cos(q6)*sin(q4)*sin(q5) + 200*cm5_z*m5*cos(q2)*sin(q1)*sin(q3)*sin(q5) + 200*cm5_z*m5*cos(q3)*sin(q1)*sin(q2)*sin(q5) + 200*cm6_y*m6*cos(q2)*sin(q1)*sin(q3)*sin(q5) + 200*cm6_y*m6*cos(q3)*sin(q1)*sin(q2)*sin(q5) + 200*cm6_x*m6*cos(q1)*sin(q4)*sin(q5)*sin(q6) - 9*m7*cos(q4)*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q5) - 200*cm5_z*m5*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) - 200*cm6_y*m6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1) + 200*cm5_x*m5*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5) - 200*cm6_z*m6*cos(q2)*cos(q5)*cos(q6)*sin(q1)*sin(q3) - 200*cm6_z*m6*cos(q3)*cos(q5)*cos(q6)*sin(q1)*sin(q2) - 200*cm7_z*m7*cos(q2)*cos(q5)*cos(q6)*sin(q1)*sin(q3) - 200*cm7_z*m7*cos(q3)*cos(q5)*cos(q6)*sin(q1)*sin(q2) + 200*cm5_z*m5*cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3) + 200*cm6_y*m6*cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3) - 200*cm6_x*m6*cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q6) - 200*cm6_x*m6*cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q6) + 200*cm7_y*m7*cos(q2)*cos(q7)*sin(q1)*sin(q3)*sin(q5) + 200*cm7_y*m7*cos(q3)*cos(q7)*sin(q1)*sin(q2)*sin(q5) + 200*cm7_x*m7*cos(q1)*cos(q7)*sin(q4)*sin(q5)*sin(q6) - 200*cm5_x*m5*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5) + 200*cm7_x*m7*cos(q2)*sin(q1)*sin(q3)*sin(q5)*sin(q7) + 200*cm7_x*m7*cos(q3)*sin(q1)*sin(q2)*sin(q5)*sin(q7) - 200*cm7_y*m7*cos(q1)*sin(q4)*sin(q5)*sin(q6)*sin(q7) + 9*m7*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q1)*sin(q5) - 200*cm7_y*m7*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q7)*sin(q1) - 200*cm6_z*m6*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q1)*sin(q5) - 200*cm7_x*m7*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q7) - 200*cm7_z*m7*cos(q2)*cos(q3)*cos(q4)*cos(q6)*sin(q1)*sin(q5) - 200*cm6_x*m6*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5)*sin(q6) + 200*cm7_y*m7*cos(q4)*cos(q5)*cos(q7)*sin(q1)*sin(q2)*sin(q3) - 200*cm7_x*m7*cos(q2)*cos(q5)*cos(q7)*sin(q1)*sin(q3)*sin(q6) - 200*cm7_x*m7*cos(q3)*cos(q5)*cos(q7)*sin(q1)*sin(q2)*sin(q6) + 200*cm6_z*m6*cos(q4)*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q5) + 200*cm7_x*m7*cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q7) + 200*cm7_z*m7*cos(q4)*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q5) + 200*cm7_y*m7*cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q6)*sin(q7) + 200*cm7_y*m7*cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q6)*sin(q7) + 200*cm6_x*m6*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5)*sin(q6) - 200*cm7_x*m7*cos(q2)*cos(q3)*cos(q4)*cos(q7)*sin(q1)*sin(q5)*sin(q6) + 200*cm7_y*m7*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5)*sin(q6)*sin(q7) + 200*cm7_x*m7*cos(q4)*cos(q7)*sin(q1)*sin(q2)*sin(q3)*sin(q5)*sin(q6) - 200*cm7_y*m7*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5)*sin(q6)*sin(q7)))/200;
    Torque(6) = g*m7*((9*cos(q1)*cos(q4)*cos(q6))/200 - cm7_z*cos(q1)*cos(q4)*cos(q6) + (9*cos(q1)*cos(q5)*sin(q4)*sin(q6))/200 - cm7_x*cos(q1)*cos(q4)*cos(q7)*sin(q6) - cm7_z*cos(q1)*cos(q5)*sin(q4)*sin(q6) + cm7_y*cos(q1)*cos(q4)*sin(q6)*sin(q7) + (9*cos(q2)*cos(q3)*cos(q6)*sin(q1)*sin(q4))/200 - (9*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4))/200 + (9*cos(q2)*sin(q1)*sin(q3)*sin(q5)*sin(q6))/200 + (9*cos(q3)*sin(q1)*sin(q2)*sin(q5)*sin(q6))/200 - cm7_z*cos(q2)*cos(q3)*cos(q6)*sin(q1)*sin(q4) - cm7_y*cos(q1)*cos(q5)*cos(q6)*sin(q4)*sin(q7) + cm7_z*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4) - cm7_z*cos(q2)*sin(q1)*sin(q3)*sin(q5)*sin(q6) - cm7_z*cos(q3)*sin(q1)*sin(q2)*sin(q5)*sin(q6) - (9*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q6))/200 + (9*cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q6))/200 + cm7_x*cos(q1)*cos(q5)*cos(q6)*cos(q7)*sin(q4) + cm7_x*cos(q7)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6) - cm7_y*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6)*sin(q7) + cm7_z*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q6) - cm7_x*cos(q2)*cos(q3)*cos(q7)*sin(q1)*sin(q4)*sin(q6) + cm7_x*cos(q2)*cos(q6)*cos(q7)*sin(q1)*sin(q3)*sin(q5) + cm7_x*cos(q3)*cos(q6)*cos(q7)*sin(q1)*sin(q2)*sin(q5) - cm7_z*cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q6) + cm7_y*cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q6)*sin(q7) - cm7_y*cos(q2)*cos(q6)*sin(q1)*sin(q3)*sin(q5)*sin(q7) - cm7_y*cos(q3)*cos(q6)*sin(q1)*sin(q2)*sin(q5)*sin(q7) - cm7_x*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*cos(q7)*sin(q1) + cm7_y*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q1)*sin(q7) + cm7_x*cos(q4)*cos(q5)*cos(q6)*cos(q7)*sin(q1)*sin(q2)*sin(q3) - cm7_y*cos(q4)*cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q7)) - g*m6*(cm6_z*cos(q1)*cos(q4)*cos(q6) + cm6_x*cos(q1)*cos(q4)*sin(q6) - cm6_x*cos(q1)*cos(q5)*cos(q6)*sin(q4) + cm6_z*cos(q1)*cos(q5)*sin(q4)*sin(q6) + cm6_z*cos(q2)*cos(q3)*cos(q6)*sin(q1)*sin(q4) + cm6_x*cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q6) - cm6_x*cos(q2)*cos(q6)*sin(q1)*sin(q3)*sin(q5) - cm6_x*cos(q3)*cos(q6)*sin(q1)*sin(q2)*sin(q5) - cm6_z*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4) + cm6_z*cos(q2)*sin(q1)*sin(q3)*sin(q5)*sin(q6) + cm6_z*cos(q3)*sin(q1)*sin(q2)*sin(q5)*sin(q6) - cm6_x*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q6) + cm6_x*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q6)*sin(q1) - cm6_z*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q6) - cm6_x*cos(q4)*cos(q5)*cos(q6)*sin(q1)*sin(q2)*sin(q3) + cm6_z*cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q6));
    Torque(7) = -g*m7*(cm7_y*cos(q1)*cos(q4)*cos(q6)*cos(q7) + cm7_x*cos(q1)*cos(q4)*cos(q6)*sin(q7) + cm7_x*cos(q1)*cos(q7)*sin(q4)*sin(q5) - cm7_y*cos(q1)*sin(q4)*sin(q5)*sin(q7) - cm7_x*cos(q2)*cos(q5)*cos(q7)*sin(q1)*sin(q3) - cm7_x*cos(q3)*cos(q5)*cos(q7)*sin(q1)*sin(q2) + cm7_y*cos(q1)*cos(q5)*cos(q7)*sin(q4)*sin(q6) + cm7_y*cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q7) + cm7_y*cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q7) + cm7_x*cos(q1)*cos(q5)*sin(q4)*sin(q6)*sin(q7) - cm7_x*cos(q6)*sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q7) - cm7_y*cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5)*sin(q7) + cm7_x*cos(q2)*sin(q1)*sin(q3)*sin(q5)*sin(q6)*sin(q7) + cm7_x*cos(q3)*sin(q1)*sin(q2)*sin(q5)*sin(q6)*sin(q7) - cm7_x*cos(q2)*cos(q3)*cos(q4)*cos(q7)*sin(q1)*sin(q5) + cm7_y*cos(q2)*cos(q3)*cos(q6)*cos(q7)*sin(q1)*sin(q4) + cm7_x*cos(q2)*cos(q3)*cos(q6)*sin(q1)*sin(q4)*sin(q7) + cm7_y*cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5)*sin(q7) + cm7_x*cos(q4)*cos(q7)*sin(q1)*sin(q2)*sin(q3)*sin(q5) - cm7_y*cos(q6)*cos(q7)*sin(q1)*sin(q2)*sin(q3)*sin(q4) + cm7_y*cos(q2)*cos(q7)*sin(q1)*sin(q3)*sin(q5)*sin(q6) + cm7_y*cos(q3)*cos(q7)*sin(q1)*sin(q2)*sin(q5)*sin(q6) - cm7_y*cos(q2)*cos(q3)*cos(q4)*cos(q5)*cos(q7)*sin(q1)*sin(q6) - cm7_x*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1)*sin(q6)*sin(q7) + cm7_y*cos(q4)*cos(q5)*cos(q7)*sin(q1)*sin(q2)*sin(q3)*sin(q6) + cm7_x*cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)*sin(q6)*sin(q7));
    
end
Torque