function handles = get_joint_handles(vrep,id)

robot_name = 'Franka';
handles = struct('id',id);

%% arm joints
armJoints = -ones(1,7);
for i=1:7
    [~,armJoints(i)] = vrep.simxGetObjectHandle(id,[robot_name,'_joint',num2str(i)],vrep.simx_opmode_oneshot_wait);
    
end
handles.armJoints = armJoints;


%% streaming
for i=1:7
    vrep.simxGetJointPosition(id,armJoints(i),vrep.simx_opmode_streaming);

    vrep.simxGetObjectFloatParameter(id,armJoints(i),2012,vrep.simx_opmode_streaming);

end

% Make sure that all streaming data has reached the client at least once
vrep.simxGetPingTime(id);
end