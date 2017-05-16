function [torque_data_coupled,actual_pos_coupled] = coupledDataCollection()
    % collect the torque data of different arm poses
    % all joints, except 1,6,7, are changed every pose
    rosinit
    %receive and send torque/position data 
    sub_pos = rossubscriber('/dvrk/MTMR/state_joint_current');
    sub_tor = rossubscriber('/dvrk/MTMR/state_joint_current');
    pub_pos = rospublisher('/dvrk/MTMR/set_position_joint');

    torque_data_coupled = zeros(8,50,12);
    actual_pos_coupled = zeros(8,50,12);
    
    %initialize with home pose
    q_init = [0,0,0,0,0,0,0];
    Set_Position(pub_pos,q_init);
    pause(3)
    
    for i = 1:12
        r2 = 0.05*randn;
        r3 = 0.05*randn;
        r4 = 0.1*randn;
        r5 = 0.1*randn;
        q = [0,r2,r3,r4,r5,0,0];
        Set_Position(pub_pos,q);
        pause(3)
        for j = 1:50
            torque_data_coupled(:,j,i) = Get_Torque(sub_tor);
            actual_pos_coupled(:,j,i) = Get_Position(sub_pos);
        end
    end
    
    rosshutdown
end