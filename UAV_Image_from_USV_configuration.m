% This script generates A, B and C posion on image that the 
% camera captures, takes as origin the center of the image

function [ X Y X_prime Y_prime]= image_generator_nELLIPSE(Ang1,Ang2,Ang3,Method,Distance_o,n,r)

%%  Camera Parameters
Width=3840; %Image resolution
Height=2160; %Image resolution
Fangle=50; %Focal angle
Px=Distance_o*sind(Fangle)/sind(90-Fangle)/ (3840/2);
Py=Px;
fx=Distance_o/Px;  
fy=Distance_o/Py;
s=fx*tand(0);
cx =Width/2; %width/2
cy =Height/2; %height
K= [fx s cx;0 fy cy; 0 0 1];
T=   [ 0 1 0;  1  0  0;   0  0 -1];


%% Generate n points circle (BEACONs on a circle arange)
    for i=0:n-1
        rot = [cosd(i*360/n) -sind(i*360/n); sind(i*360/n) cosd(i*360/n)]; 
        P(:,i+1) = [  [rot*[ r 0]']' 0 ];    %Marker points on USV deck
    end

%% Generate 3D Rotation matrix

if Method==1    %ZYX Rotation
      
Rx = [     1            0                      0           ;
              0        cosd(Ang1)  -sind(Ang1)    ; 
              0        sind(Ang1)   cosd(Ang1)   ];

Ry = [ cosd(Ang2)       0        sind(Ang2)   ;
               0                  1               0          ;
          -sind(Ang2)       0        cosd(Ang2) ];

Rz = [  cosd(Ang3)  sind(Ang3)    0   ;
          -sind(Ang3)   cosd(Ang3)   0   ;
                  0                   0            1 ]  ;   
              
R=Rz*Ry*Rx;

elseif Method==2    %ZXZ Rotation

Rz1 = [  cosd(Ang1)   sind(Ang1)   0  ;
             -sind(Ang1)   cosd(Ang1)  0  ;
                    0                   0            1] ;
                
Rx = [  1         0                    0           ;
            0   cosd(Ang2)  -sind(Ang2) ; 
            0   sind(Ang2)   cosd(Ang2) ];

Rz2 = [  cosd(Ang3)  sind(Ang3)    0   ;
             -sind(Ang3)   cosd(Ang3)  0   ;
                    0                   0            1 ]  ;                         
                
R = Rz1*Rx*Rz2;                   

end %End Method

%% 3D Rotate Beacons

for i=1:n
    Prot(:,i)=[P(:,i)'*R]';
end

%% Convert rotated BEACONs to pixel coordinates (Distort Ellipse using Pinhole camera model)

    for i=0:n-1
        rot = [cosd(i*360/n) -sind(i*360/n); sind(i*360/n) cosd(i*360/n)]; 
        P = [  [rot*[ r 0]']' 0 ];    %Marker points on USV deck
        Oimage=K*T*[ Prot(:,i+1) + [ 0;0;Distance_o] ];
        Mpixel(i+1,:)= [ Oimage(1)/Oimage(3) Oimage(2)/Oimage(3)]; %PxCoordinate
    end         

%% Generate image and crop out of range values
Image = [Mpixel(:,2) Mpixel(:,1)];
Image = Image(Image(:,1) >= 0, :);
Image = Image(Image(:,1) <= Height, :);
Image = Image(Image(:,2) >= 0, :);
Image = Image(Image(:,2) <= Width, :);

%% Change coordinate to make image center (0,0)
X_prime=Image(:,2); X_prime= [ X_prime-Width/2];
Y_prime=Image(:,1); Y_prime= [ Height/2-Y_prime];


%%  Generate undistorted Ellipse as reference only
[Mproj] = Ellipse_2D_projection(Prot,n, r);     
X=Mproj(:,1) ;  Y=Mproj(:,2); 

end


function [Mproj ] = Ellipse_2D_projection(Prot,n,r)

% Project rotated circle using equation of the plane
P1=[ 0 1 0];    P2=[ 1 0 0];    P3=[0 0 0];
N=cross(P1,P2);                        %Not normalized yet
d=N*P1';                                    %d

Mrot =Prot';
    for i=0:n-1
        t =  ( d - N(1)*Mrot(i+1,1) - N(2)*Mrot(i+1,2) - N(3)*Mrot(i+1,3) ) / ( N(1)*N(1) + N(2)*N(2) + N(3)*N(3) );  
        rot = [cosd(90) -sind(90); sind(90) cosd(90)]; 
        Mproj(i+1,:) =  rot*[ N(1)*t+Mrot(i+1,1) N(2)*t+Mrot(i+1,2)]';
    end %End for i
        
end
    