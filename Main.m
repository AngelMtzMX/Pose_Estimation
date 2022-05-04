clear all
clc
beep off
close all

%% Dont modify anything, just this section

%Note: Introduce desired Min and Max degree range for validation, it is considered to be Roll, Pitch 
% and Yaw, which is converted to ZXZ orientation, then circular pattern on USV deck is
% rotated and projected to an image by pinhole camera model
MinDeg=.001; 
MaxDeg=89;
step=10; 

%Note: Error on Beacon position on image can also be considered
p=0.01; %Measurement error, percent


%% Generate Roll Pitch Yaw Setpoints from 0 to 89 Degrees
        Ang1_i =[]; Ang2_j=[]; Ang3_k = []; count= 0;
        vector=MinDeg:step:MaxDeg;
        for i=MinDeg:step:MaxDeg
            count=count+1
                for j=MinDeg:step:MaxDeg
                        Ang1_i   = [Ang1_i i*ones(1,length(vector))];
                        Ang2_j   = [Ang2_j j*ones(1,length(vector))];
                        Ang3_k  = [Ang3_k vector];
                end
        end
        
 Ang1_i  = [  Ang1_i  flip(Ang1_i)  -Ang1_i -flip(Ang1_i)   Ang1_i   flip(Ang1_i)    -Ang1_i   -flip(Ang1_i) ];
 Ang2_j  = [ Ang2_j         Ang2_j    Ang2_j         Ang2_j   -Ang2_j        -Ang2_j    -Ang2_j          -Ang2_j  ];
 Ang3_k = [ Ang3_k       -Ang3_k   Ang3_k       -Ang3_k   Ang3_k       -Ang3_k     Ang3_k         -Ang3_k ];
%          Ang3_k = 0*ones(1,length(Ang1_i));

% Ang1_i  = [    0.0020    ];
% Ang2_j  = [    -0.0010     ];
% Ang3_k = [   -60.0010   ];


%% Convert ZYX Setpoints to ZXZ Setpoints, then use as script input
count2= 0;
for o=1:length(Ang1_i)
count2=count2+1

    Ang1= Ang1_i(o); Ang2=Ang2_j(o); Ang3=Ang3_k(o);

%% XYZ sequence
n=12; r=10; % r=262.9036;

R = eul2rotm([-Ang1*pi/180 -Ang2*pi/180 -Ang3*pi/180],'XYZ');
    for i=0:n-1
        %30 Degrees because it consider n=12 markers located at USV deck
        rot = [cosd(i*360/n) -sind(i*360/n); sind(i*360/n) cosd(i*360/n)]; 
        M = [  [rot*[ r 0]']' 0];   %Marker points on USV deck
        MrotXYZ(i+1,:) = [R*M'];     %Marker point to to rotate
    end
% MrotXYZ;
% eul = rotm2eul(R,'XYZ');
% Angles=[ eul(1)*180/pi eul(2)*180/pi eul(3)*180/pi] ;


%% ZYX sequence
R = eul2rotm([Ang3*pi/180 Ang2*pi/180 Ang1*pi/180],'ZYX');
    for i=0:n-1
        %30 Degrees because it consider n=12 markers located at USV deck
        rot = [cosd(i*360/n) -sind(i*360/n); sind(i*360/n) cosd(i*360/n)]; 
        M = [  [rot*[ r 0]']' 0];    %Marker points on USV deck
        MrotZYX(i+1,:) = [M*R];     %Marker point to to rotate
    end
% MrotZYX;
% eul = rotm2eul(R,'ZYX');
% Angles=[ eul(1)*180/pi eul(2)*180/pi eul(3)*180/pi] ;


%% Convert to ZXZ sequence
R = angle2quat(Ang3*pi/180, Ang2*pi/180, Ang1*pi/180,'ZYX');
[Yaw1, Roll, Yaw2] = quat2angle(R, 'ZXZ');
Deg=[-Yaw1*180/pi Roll*180/pi -Yaw2*180/pi];

Rz1 = [ cosd(Deg(1))  sind(Deg(1))   0   ;
             -sind(Deg(1))   cosd(Deg(1))  0   ;
                    0                   0                 1] ;
                
Rx = [  1         0                    0                ;
            0   cosd(Deg(2))  -sind(Deg(2)) ; 
            0   sind(Deg(2))   cosd(Deg(2)) ];

Rz2 = [ cosd(Deg(3))  sind(Deg(3))   0   ;
             -sind(Deg(3))   cosd(Deg(3))  0   ;
                    0                   0           1 ]      ;                         
                
R = Rz1*Rx*Rz2;
    for i=0:n-1
        %30 Degrees because it consider n=12 markers located at USV deck
        rot = [cosd(i*360/n) -sind(i*360/n); sind(i*360/n) cosd(i*360/n)]; 
        M = [  [rot*[ r 0]']' 0];    %Marker points on USV deck
        MrotZXZ(i+1,:) = [M*R];     %Marker point to to rotate
        
    end
AnglesZXZ(o,:)=Deg;
Error1(o)=norm (MrotXYZ - MrotZXZ); %This is always zero


%% Input ZXZ coordinates to scripts
D=30; %Distance
% Generate captured image
[ X Y X_prime Y_prime] = image_generator_nELLIPSE_04082022(Deg(1),Deg(2),Deg(3),2,D,n,r);

    %Introduce noise 
    for m=1:length(X_prime)
    a=norm( [X_prime(m) Y_prime(m)])*p; b=-a;
    X_prime_error(m,1) = X_prime(m)+a + (b-a)*rand;
    Y_prime_error(m,1) = Y_prime(m)+a + (b-a)*rand;                 
    end
    X_prime_error=X_prime_error(1:length(X_prime));
    Y_prime_error=Y_prime_error(1:length(X_prime));
    
% Compute position and distance
[Degrees(o,:),Distance(o), coeff(:,o),Theta(o),Mi(o)] = USV_Configuration_Ellipses_Test_05022022(X_prime_error,Y_prime_error);

% Compute error
Rz2 = [   cosd(Degrees(o,3))  sind(Degrees(o,3))   0   ;
             -sind(Degrees(o,3))   cosd(Degrees(o,3))  0   ;
                    0                   0                 1] ;
                
Rx = [  1         0                    0                ;
            0   cosd(Degrees(o,2))  -sind(Degrees(o,2)) ; 
            0   sind(Degrees(o,2))    cosd(Degrees(o,2)) ];

Rz1 = [   cosd(Degrees(o,1))  sind(Degrees(o,1))   0   ;
             -sind(Degrees(o,1))   cosd(Degrees(o,1))  0   ;
                    0                   0           1 ]      ;     
        
R2 = Rz2*Rx*Rz1;


%% Compute Error                

    for i=0:n-1
        %30 Degrees because it consider n=12 markers located at USV deck
        rot = [cosd(i*360/n) -sind(i*360/n); sind(i*360/n) cosd(i*360/n)]; 
        M = [  [rot*[ r 0]']' 0];    %Marker points on USV deck
        MrotZXZ_Script(i+1,:) = [M*R2];     %Marker point to to rotate      
    end
    
% Error
Error2(o)=norm (MrotZXZ - MrotZXZ_Script);

end %Main for o

%% Plot Results

         L=linspace(0,length(Ang1_i),9);

        %Plot Results in ZXZ
         f1=figure;
         f1.WindowState = 'maximized';
     
         subplot(4,3,1)
         plot(AnglesZXZ(:,3))
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         title('Alpha Setpoint input', 'FontSize', 24);
         ylabel('Deg')

         subplot(4,3,2)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(AnglesZXZ(:,2))
         grid on
         title('Beta Setpoint input', 'FontSize', 24);
         ylabel('Deg')
         
         subplot(4,3,3)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(AnglesZXZ(:,1))
         grid on
         title('Gamma Setpoint input', 'FontSize', 24);
         ylabel('Deg')
         
         subplot(4,3,4)
         plot(Degrees(:,1))
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         title('Alpha Measurement output', 'FontSize', 24);
         ylabel('Deg')  
         
         subplot(4,3,5)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Degrees(:,2))
         grid on
         title('Beta Measurement output', 'FontSize', 24);
         ylabel('Deg')  
         
         subplot(4,3,6)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Degrees(:,3))
         grid on
         title('Gamma Measurement output', 'FontSize', 24);
         ylabel('Deg')  
         
         subplot(4,1,3)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Error2)
         title('Error norm |Pn*Rsetpoint - Pn*Rmeasurement|', 'FontSize', 20);
         
         subplot(4,1,4)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Distance)
         title('Height measurement output', 'FontSize', 20);

         
         
         %Plot Results in ZXYZ
        
f2=figure;         
             f2.WindowState = 'maximized';
     
         subplot(5,3,1)
         plot(Ang1_i)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9));  
         grid on
         title('Roll Setpoint (not an input)', 'FontSize', 24);
         ylabel('Deg')  
         
         subplot(5,3,2)
         plot(Ang2_j)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9));          
         grid on
         title('Pitch Setpoint (not an input)', 'FontSize', 24);
         ylabel('Deg')  
         
         subplot(5,3,3)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Ang3_k)
         grid on
         title('Yaw Setpoint (not an input)', 'FontSize', 24);
         ylabel('Deg')  
         DegSP=[Ang1_i' Ang2_j' Ang3_k'];
         
         
            for i=1:length(Ang1_i)      
                Q = angle2quat(Degrees(i,3)*pi/180, Degrees(i,2)*pi/180, Degrees(i,1)*pi/180,'ZXZ');
                [Yaw, Pitch, Roll] = quat2angle(Q, 'ZYX');
                Deg(i,:)=[Roll*180/pi -Pitch*180/pi -Yaw*180/pi];
            end
        
         subplot(4,3,4)
         plot(Deg(:,1))
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         title('Roll Measurement (not an output)', 'FontSize', 22);
         ylabel('Deg')  
         
         subplot(4,3,5)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Deg(:,2))
         grid on
         title('Pitch Measurement (not an output)', 'FontSize', 22);
         ylabel('Deg')  
         
         subplot(4,3,6)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Deg(:,3))
         grid on
         title('Yaw Measurement (not an output)', 'FontSize', 22);
         ylabel('Deg')  
        
        
         subplot(4,1,3)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Error2)
         title('Error norm |Pn*Rsetpoint - Pn*Rmeasurement|', 'FontSize', 20);
         
         subplot(4,1,4)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Distance)
         title('Height measurement output', 'FontSize', 20);

         %Plot Theta and Phi
        
f3=figure;         
         f3.WindowState = 'maximized';
         
         subplot(7,1,1)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(coeff(1,:))
         title('Coeff B', 'FontSize', 20);
         
          subplot(7,1,2)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(coeff(2,:))
         title('Coeff C', 'FontSize', 20);
         
          subplot(7,1,3)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(coeff(3,:))
         title('Coeff D', 'FontSize', 20);
                          
         subplot(7,1,4)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(coeff(4,:))
         title('Coeff E', 'FontSize', 20);
                          
         subplot(7,1,5)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(coeff(5,:))
         title('Coeff F', 'FontSize', 20);         
 
         subplot(7,1,6)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Error2)
         title('Error norm |Pn*Rsetpoint - Pn*Rmeasurement|', 'FontSize', 20);
         
         subplot(7,1,7)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Mi)
         title('Undistorted Minor Semi-axis', 'FontSize', 20);
         
         
         
         
         %Plot Theta and Phi
        
f4=figure;                
         f4.WindowState = 'maximized';
  
         subplot(5,1,1)
         plot(Degrees(:,1))
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         title('Alpha Measurement output', 'FontSize', 24);
         ylabel('Deg')  
         
         subplot(5,1,2)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Degrees(:,2))
         grid on
         title('Beta Measurement output', 'FontSize', 24);
         ylabel('Deg')  
         
         subplot(5,1,3)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Degrees(:,3))
         grid on
         title('Gamma Measurement output', 'FontSize', 24);
         ylabel('Deg')  
         
         subplot(5,1,4)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Error2)
         title('Error norm |Pn*Rsetpoint - Pn*Rmeasurement|', 'FontSize', 20);
         
         subplot(5,1,5)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Distance)
         title('Height measurement output', 'FontSize', 20);
         
         
              
f5=figure;                
         f5.WindowState = 'maximized';
  

         subplot(5,1,1)
         plot(Deg(:,1))
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         title('Roll Measurement (not an output)', 'FontSize', 22);
         ylabel('Deg')  
         
         subplot(5,1,2)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Deg(:,2))
         grid on
         title('Pitch Measurement (not an output)', 'FontSize', 22);
         ylabel('Deg')  
         
         subplot(5,1,3)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Deg(:,3))
         grid on
         title('Yaw Measurement (not an output)', 'FontSize', 22);
         ylabel('Deg')  
     
         subplot(5,1,4)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Error2)
         title('Error norm |Pn*Rsetpoint - Pn*Rmeasurement|', 'FontSize', 20);
         
         subplot(5,1,5)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Distance)
         title('Height measurement output', 'FontSize', 20);