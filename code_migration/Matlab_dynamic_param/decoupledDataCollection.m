function torque_data = decoupledDataCollection()
    % This function is for data collection for each joint torque of the MTM
    % (the joints are decoupled)
    sub_pos = subscriber('/dvrk/MTMR/state_joint_current');
    sub_tor = subscirber('/dvrk/MTMR/state_joint_current');
    pub_pos = publisher('/dvrk/MTMR/set_position_joint');
    torque_data = zeros(10,50,7);
    % reset the pose to home configuration
    q = [0,0,0,0,0,0,0];
    Set_Position(pub_pos,q)
    % set and record the torque data at different configurations
    % Each time the data collection of one joint is done, the robot is 
    % reset to the home pose and then move on to the next joint
    for i = 1:7
        for j = 1:10
            q(i) = q(i)+0.005;
            Set_Postion(pub_pos,q)
            for k = 1:50
                msg = receive(sub_tor);
                torque_data(j,k,i) = msg.Effort(i);
            end            
        end       
    end
end