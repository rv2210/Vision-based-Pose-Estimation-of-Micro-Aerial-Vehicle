%% PROJECT 2 VELOCITY ESTIMATION
close all;
clear all;
clc;
addpath('../data')

%Change this for both dataset 1 and dataset 4. Do not use dataset 9.
datasetNum = 1;

[sampledData, sampledVicon, sampledTime] = init(datasetNum);

%% INITIALIZE CAMERA MATRIX AND OTHER NEEDED INFORMATION
cam_intrinsic= [311.0520, 0, 201.8724; 0,  311.3885, 113.6210; 0,0,1];

for n = 2:length(sampledData)
   
    Percentage_completed= (n/864)*100 

   
    
    %% Ransac Switch
    RANSAC_switch= 0;

        %change RANSAC Switch value to o to turn of Ransac velocity
        %function




    
    %% Initalize Loop load images
    current_image= sampledData(n).img;
    previous_image= sampledData(n-1).img;

    %% Detect good points
    good_points= vision.PointTracker('MaxBidirectionalError',1);

    corners= detectFASTFeatures(previous_image);
    Pos= double(corners.Location); %double is used for accuracy and precision
    %% Initalize the tracker to the last frame.
    initialize(good_points, Pos,previous_image);
    %% Find the location of the next points;
    [k, valid]= good_points(current_image);
    %% Calculate velocity
    % Use a for loop
    last_point=[];
    current_point=[];
    for i= 1:length(k)
        old_position= inv(cam_intrinsic)* [Pos(i,1);Pos(i,2);1];
        new_position= inv(cam_intrinsic)* [k(i,1); k(i,2); 1];
        last_point(i,1:2)= [old_position(1,1) old_position(2,1)];
        current_point(i,1:2)= [new_position(1,1) new_position(2,1)];
   
    end
    dt= (sampledData(n).t - sampledData(n-1).t) ;  %calculating change in time value for velocity calculation
    u= (current_point(:,1)-last_point(:,1))./dt;    %calculating velocity in x direction
    v= (current_point(:,2)-last_point(:,2))./dt;    %calculating velocity in y direction
    %% Calculate Height
    [position, orientation, R_c2w] = estimatePose(sampledData, n);
    X=position(1);
    Y=position(2);
    R_w_b= eul2rotm(orientation,'ZYX'); %calculating world wrt drone rotation matrix
    t_w_b= position;                    %calculating world wrt drone translation matrix
    
    t_b_c= [ 0.04*cosd(45) ; -0.04* sind(45) ; -0.03];  
    R_b_c= [0.7071 -0.7071 0; -0.7071 -0.7071 0 ; 0 0 -1];

    T_w_b= [R_w_b t_w_b; 0 0 0 1]; %Transformation matrix of drone wrt world
    T_b_c= [R_b_c t_b_c; 0 0 0 1]; %Transformation matrix of camera wrt body

    T_w_c= T_w_b*T_b_c;             
    T_c_w= pinv(T_w_c);             %Transformation matrix of world wrt camera
    R_c_w= T_c_w(1:3,1:3);
    
    Z= T_c_w(3,4);
    f= [];
    VEL= [];
    for i= 1:length(k)
        u_v= [-1/Z, 0, current_point(i,1)/Z, current_point(i,1)*current_point(i,2), -(1+current_point(i,1)^2), current_point(i,2); 
              0, -1/Z, current_point(i,2)/Z, 1+current_point(i,2)^2, -current_point(i,1)*current_point(i,2), -current_point(i,1)] ;
    
        f=vertcat(f,u_v); 
        VEL= vertcat(VEL,u(i),v(i));
        optV= VEL; %optical velocity matrix
    end
    optPos= (1/Z)*[current_point(:,1) current_point(:,2)]; %optical position matrix
    
    %% RANSAC    
    % Write your own RANSAC implementation in the file velocityRANSAC
    e=0.8;
    [Velo] = velocityRANSAC(optV,optPos,Z,R_c2w,e);
    %% Thereshold outputs into a range.
    % Not necessary
    
    %% Fix the linear velocity
    % Change the frame of the computed velocity to world frame
    V= pinv(f) * VEL;
    if RANSAC_switch==1
        Vel= [ R_c_w,zeros(3,3); zeros(3,3),R_c_w] * Velo;
    elseif RANSAC_switch==0
        Vel= [ R_c_w,zeros(3,3); zeros(3,3),R_c_w] * V;
    end
    %% ADD SOME LOW PASS FILTER CODE
    % Not neceessary but recommended 
   
    estimatedV(:,n) = Vel;
    
    %% STORE THE COMPUTED VELOCITY IN THE VARIABLE estimatedV AS BELOW
    estimatedV(:,n) = Vel; % Feel free to change the variable Vel to anything that you used.
    % Structure of the Vector Vel should be as follows:
    % Vel(1) = Linear Velocity in X
    % Vel(2) = Linear Velocity in Y
    % Vel(3) = Linear Velocity in Z
    % Vel(4) = Angular Velocity in X
    % Vel(5) = Angular Velocity in Y
    % Vel(6) = Angular Velocity in Z
end

%% Implementing Low Pass filter for better accuracy of code
vx= transpose(estimatedV(1,:));
vy= transpose(estimatedV(2,:));
vz= transpose(estimatedV(3,:));
wx= transpose(estimatedV(4,:));
wy= transpose(estimatedV(5,:));
wz= transpose(estimatedV(6,:));

if (n>=5)
    estimatedV(1,:)= sgolayfilt(vx, 1, 13);
    estimatedV(2,:)= sgolayfilt(vy, 1, 13);
    estimatedV(3,:)= sgolayfilt(vz, 1, 13);
    estimatedV(4,:)= sgolayfilt(wx, 1, 13);
    estimatedV(5,:)= sgolayfilt(wy, 1, 13);
    estimatedV(6,:)= sgolayfilt(wz, 1, 13);
end
plotData(estimatedV, sampledData, sampledVicon, sampledTime, datasetNum)
