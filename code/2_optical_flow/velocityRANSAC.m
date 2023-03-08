function [Velo] = velocityRANSAC(optV,optPos,Z,R_c2w,e)
%% CHANGE THE NAME OF THE FUNCTION TO velocityRANSAC
    %% Input Parameter Description
    % optV = The optical Flow
    % optPos = Position of the features in the camera frame 
    % Z = Height of the drone
    % R_c2w = Rotation defining camera to world frame
    % e = RANSAC hyper parameter    
    
    %% Output Parameter Description
    % Vel = Linear velocity and angualr velocity vector
    e; %epsilon value 
    probability_success= 0.99;  %the probability of hitting an inlier in the selected points
    Min_points= 4;              %minimum number of points required for the model

    k= (log(1-probability_success)/log(1-e^Min_points)); %Calculating the number of iterations to be done
    
    Max=0;

    for i= 1:k
        inliers=0;

        randomP= randperm(length(optPos),3); %selecting 3 random points from optical Position

        Point1= optPos(randomP(1,1),:);      %Defining each point in the randomly selected points
        Point2= optPos(randomP(1,2),:);
        Point3= optPos(randomP(1,3),:);

        H_First_row= [-1/Z, 0, Point1(1,1)/Z, Point1(1,1)*Point1(1,2), -(1+Point1(1,1)^2), Point1(1,2); 0, -1/Z, Point1(1,2)/Z, 1+Point1(1,2)^2, -Point1(1,1)*Point1(1,2), -Point1(1,1)] ;
        H_Second_row= [-1/Z, 0, Point2(1,1)/Z, Point2(1,1)*Point2(1,2), -(1+Point2(1,1)^2), Point2(1,2); 0, -1/Z, Point2(1,2)/Z, 1+Point2(1,2)^2, -Point2(1,1)*Point2(1,2), -Point2(1,1)] ;
        H_Third_row= [-1/Z, 0, Point3(1,1)/Z, Point3(1,1)*Point3(1,2), -(1+Point3(1,1)^2), Point3(1,2); 0, -1/Z, Point3(1,2)/Z, 1+Point3(1,2)^2, -Point3(1,1)*Point3(1,2), -Point3(1,1)] ;

        h_matrix=[ H_First_row; H_Second_row; H_Third_row]; %defining H matrix formed from the 3 random selected points

        optical_velocity= [optV(2*randomP(1,1) -1);optV(2*randomP(1,1)); optV(2*randomP(1,2) -1);optV(2*randomP(1,2)) ; optV(2*randomP(1,3) -1);optV(2*randomP(1,3)) ]; %defining optimal velocity matrix

        V= pinv(h_matrix)* optical_velocity;
%calculating output velocity to be given as an output value of the function
        for a=1: length(optPos)
            h= [-1/Z, 0, optPos(a,1)/Z, optPos(a,1)*optPos(a,2), -(1+optPos(a,1)^2), optPos(a,2);
                 0, -1/Z, optPos(a,2)/Z, 1+optPos(a,2)^2, -optPos(a,1)*optPos(a,2), -optPos(a,1) ] ;

            pos=[optV(2*a - 1); optV(2*a)];
            delta= norm(h *V - pos)^2;
            if(delta <= 1e-3)
                inliers= inliers+1;
            end
        end
        if(inliers>=Max)
            Max=inliers;
            Velo=V;
        end
    end

end