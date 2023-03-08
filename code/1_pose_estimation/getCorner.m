function res = getCorner(id)
%% CHANGE THE NAME OF THE FUNCTION TO getCorner
    %% Input Parameter Description
    % id = List of all the AprilTag ids detected in the current image(data)
    
    for l= 1:length(id)             %initialising a loop to calculate each april tag present in id Data during that particular timestamp
        x=0;                        %Initialising with x and y values as zeros
        y=0;                         %considering x and y as p4 coordinates of april tags
       
        %calculating p4 coordinates of each april tag from the april tag
        %mat 
        if (id(l)>=0 && id(l)<=11)     
            x= rem(id(l),12)*(0.304);
            y=0;

        elseif (id(l)>=12 && id(l)<=23)
            x= rem(id(l),12)*(0.304);
            y= 0.304;

        elseif (id(l)>=24 && id(l)<=35)
            x= rem(id(l),12)*(0.304);
            y= 0.608;

        elseif (id(l)>=36 && id(l)<=47)
            x= rem(id(l),12)*(0.304);
            y= 0.938;

        elseif (id(l)>=48 && id(l)<=59)
            x= rem(id(l),12)*(0.304);
            y= 1.242;

        elseif (id(l)>=60 && id(l)<=71)
            x= rem(id(l),12)*(0.304);
            y= 1.546;

        elseif (id(l)>=72 && id(l)<=83)
            x= rem(id(l),12)*(0.304);
            y= 1.876;

        elseif (id(l)>=84 && id(l)<=95)
            x= rem(id(l),12)*(0.304);
            y= 2.18;

        elseif (id(l)>=96 && id(l)<=107)
            x= rem(id(l),12)*(0.304);
            y= 2.484;
        end
%calculating x and y of p0,p1,p2,p3,p4 values from the p4 x,y coordinates
        p4_x= x ;
        p4_y= y ;
        p1_x= 0.152 + x;
        p1_y= y ;
        p3_x= x;
        p3_y= y + 0.152;
        p2_x= x + 0.152;
        p2_y= y + 0.152;
        p0_x= x + 0.076;
        p0_y= y + 0.076;
        
        %concading the p0,p1,p2,p3,p4 valuesof tags horizontally
        cordinates=[p0_x; p0_y; p1_x; p1_y; p2_x; p2_y; p3_x; p3_y; p4_x; p4_y;];
        if l == 1
            res= cordinates;
        elseif l>1
            res= horzcat(res,cordinates);
        end

    end
    


    %% Output Parameter Description
    % res = List of the coordinates of the 4 corners (or 4 corners and the
    % centre) of each detected AprilTag in the image in a systematic method

    res;
    return;
end