function torque_data = decoupledDataCollection()
    sub_pos = subscriber('/dvrk/MTMR/state_joint_current');
    sub_tor = subscirber('/dvrk/MTMR/state_joint_current');
    pub_pos = publisher('/dvrk/MTMR/set_position_joint');
    torque_data = zeros(10,50,7);
    % reset the pose to home configuration
    q = [0,0,0,0,0,0,0];
    Set_Position(pub_pos,q)
    
    for i = 1:7
        msg = rosmessage(sub_pos);
        for j = 1:100
            
        end
        
    end

end