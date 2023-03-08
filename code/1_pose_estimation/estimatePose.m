function [position, orientation] = estimatePose(data, t)
%% CHANGE THE NAME OF THE FUNCTION TO estimatePose
% Please note that the coordinates for each corner of each AprilTag are
% defined in the world frame, as per the information provided in the
% handout. Ideally a call to the function getCorner with ids of all the
% detected AprilTags should be made. This function should return the X and
% Y coordinate of each corner, or each corner and the centre, of all the
% detected AprilTags in the image. You can implement that anyway you want
% as long as the correct output is received. A call to that function
% should made from this function.
    %% Input Parameter Defination
    % data = the entire data loaded in the current dataset
    % t = index of the current data in the dataset
    
    id= data(t).id;    %Getting id tags data 
    res = getCorner(id);  %Running Getcorner function

    cam_intrinsic= [311.0520    0                 201.8724;
                    0           311.3885          113.6210;
                    0           0                 1];           %camera intrinsic matrix defined in parameter text file


    for i= 1: length(data(t).id) %Defining xi dash and yi dash for p0 p1 p2 p3 p4

        p0_dash= [ res(1,i) res(2,i) 1 0 0 0 -data(t).p0(1,i)*res(1,i) -data(t).p0(1,i)*res(2,i) -data(t).p0(1,i);
                    0 0 0 res(1,i) res(2,i) 1 -data(t).p0(2,i)*res(1,i) -data(t).p0(2,i)*res(2,i) -data(t).p0(2,i)];
        
        
        p1_dash= [ res(3,i) res(4,i) 1 0 0 0 -data(t).p1(1,i)*res(3,i) -data(t).p1(1,i)*res(4,i) -data(t).p1(1,i);
                    0 0 0 res(3,i) res(4,i) 1 -data(t).p1(2,i)*res(3,i) -data(t).p1(2,i)*res(4,i) -data(t).p1(2,i)];

       
        p2_dash= [ res(5,i) res(6,i) 1 0 0 0 -data(t).p2(1,i)*res(5,i) -data(t).p2(1,i)*res(6,i) -data(t).p2(1,i);
                    0 0 0 res(5,i) res(6,i) 1 -data(t).p2(2,i)*res(5,i) -data(t).p2(2,i)*res(6,i) -data(t).p2(2,i)];

       
        p3_dash= [ res(7,i) res(8,i) 1 0 0 0 -data(t).p3(1,i)*res(7,i) -data(t).p3(1,i)*res(8,i) -data(t).p3(1,i);
                    0 0 0 res(7,i) res(8,i) 1 -data(t).p3(2,i)*res(7,i) -data(t).p3(2,i)*res(8,i) -data(t).p3(2,i)];

       
        p4_dash= [ res(9,i) res(10,i) 1 0 0 0 -data(t).p4(1,i)*res(9,i) -data(t).p4(1,i)*res(10,i) -data(t).p4(1,i);
                    0 0 0 res(9,i) res(10,i) 1 -data(t).p4(2,i)*res(9,i) -data(t).p4(2,i)*res(10,i) -data(t).p4(2,i)];

        %Concading p0 dash p1 dash p2 dash p3 dash p4 dash
        cordinates= [p0_dash;p1_dash;p2_dash;p3_dash;p4_dash];
        if i==1
            p_dash= cordinates;
        elseif i>1
            p_dash=vertcat(p_dash,cordinates);
        end
    end

    [u, s, v_transpose]= svd(p_dash);  %Using single value decomposition matrix 

    h= v_transpose(:,9);    %extracting 9th colun of V which gives h matrix
    
    h_matrix=transpose(reshape(h,[3,3]));  %writing the h matrix as 3 x 3 matrix here transpose is used because the reshape function fills data column wise

    singular_values= svd(h_matrix); 

    h_normalized= h_matrix/ singular_values(2); %Normalising the H matrix

    sign_h= transpose([data(t).p0(1:2,1);1]) * h_normalized * [res(1:2,1);1]; %Calculating Sign of H to avoid negative values

    if sign_h < 0
        h_normalized= -h_normalized ;
    end

    r_t_matrix= pinv(cam_intrinsic)* h_normalized; 

    r1_cap= r_t_matrix(:,1);
    r2_cap= r_t_matrix(:,2);
    t_cap= r_t_matrix(:,3);

    [u2,sv2,v2_transpose]= svd([r1_cap,r2_cap,cross(r1_cap,r2_cap)]);

    R_c_w= u2 * [ 1 0 0 ; 0 1 0 ; 0 0 det(u2*v2_transpose)]* v2_transpose ;     %world with respect to camera rotation matrix
    t_c_w= t_cap/norm(r1_cap);                                                  %world with respect to camera Translation matrix

    t_b_c= [ 0.04*cosd(45) ; -0.04* sind(45) ; -0.03];           %calculating camera with respect to body matrix using the pictures given

    R_b_c= [0.7071 -0.7071 0; -0.7071 -0.7071 0 ; 0 0 -1];      %calculated by rotating frame about Z axis by clockwise 45 degress and about X axis by clockwise 180 degrees 
    
    T_b_c= [R_b_c , t_b_c ;0 0 0 1];
    T_c_w= [R_c_w , t_c_w ;0 0 0 1];

    T_b_w= T_b_c* T_c_w;    %calculating world with respect to body matrix from previous matrix
    T_w_b= pinv(T_b_w);     %calculating body with respect to world matrix from previous matrix

    R_w_b= T_w_b(1:3, 1:3);
    t_w_b= T_w_b(1:3,4);

    %% Output Parameter Defination
    
    % position = translation vector representing the position of the
    % drone(body) in the world frame in the current time, in the order ZYX
     position= t_w_b;
    % orientation = euler angles representing the orientation of the
    % drone(body) in the world frame in the current time, in the order ZYX
     orientation= rotm2eul(R_w_b,"ZYX");
     return;
end
