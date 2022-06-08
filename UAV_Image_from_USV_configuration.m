% This script generates A, B and C posion on image that the 
% camera captures, takes as origin the center of the image

function [ X_prime Y_prime]= image_generator_nELLIPSE(Ang1,Ang2,Ang3,Distance_o,n,r)

%%  Camera Parameters
Width=3840; %Image resolution
Height=2160; %Image resolution
Fangle=50; %Focal angle
Px=Distance_o*sind(Fangle)/sind(90-Fangle)/ (3840/2);
Py=Px; %Asuming square pixels
fx=Distance_o/Px;  
fy=Distance_o/Py;
s=fx*tand(0);
cx =Width/2; %width/2
cy =Height/2; %height
K= [fx s cx;0 fy cy; 0 0 1];
T=   [ 0 1 0;  1  0  0;   0  0 -1];


%% Generate 3D Rotation matrix
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

%% Generate n points circle (BEACONs on a circle arange)
    for i=0:n-1
        rot = [cosd(i*360/n) -sind(i*360/n); sind(i*360/n) cosd(i*360/n)]; 
        [rot*[ r 0]']';
        P(:,i+1) = [  [rot*[ r 0]']' 0 ];   %Marker points on USV deck
    end

%% 3D Rotate Beacons

for i=1:n
    P(:,i);
    Prot(:,i)=[P(:,i)'*R]';
end

%% Convert rotated BEACONs to pixel coordinates (Distort Ellipse using Pinhole camera model)

    for i=0:n-1
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



end

