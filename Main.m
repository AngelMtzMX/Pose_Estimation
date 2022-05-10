clear all
clc
beep off

%% Overview of the script
% This script will do the following:
% 1-Compute Roll, Pitch and Yaw 
% 2-Covert to ZYX to ZXZ angles
% 3-Run pinhole camera model script which putput is the image (Xpixel,Ypixel)
% 4-Run USV_conf_from_image script
% 5-Convert ZXZ to ZYX
% 5-Plot 8 graphs with results (ZXZ, ZYX, Theta, Coefficients)

%Note: some extra info were ploted for better understanding and
%explatation, extra plots will be removed on final release

%% No need to modify anything else, just this section
%Note: Introduce desired Min and Max degree range for validation
MinDeg=0.01; 
MaxDeg=20;
step=1; 

%Note: Error percent on Beacon position on image can also be considered
p=0.0; 


n=12;  % Beacon Qty
r=10;   % Circle radius
D=30; %  Distance




%% Generate Roll Pitch Yaw Setpoints from 0 to 89 Degrees
        Ang1_i =[]; Ang2_j=[]; Ang3_k = []; count= 0;
        vector=MinDeg:step:MaxDeg;
        for i=MinDeg:step:MaxDeg
                for j=MinDeg:step:MaxDeg
                        Ang1_i   = [Ang1_i i*ones(1,length(vector))];
                        Ang2_j   = [Ang2_j j*ones(1,length(vector))];
                        Ang3_k  = [Ang3_k vector];
                end
        end
        
 Ang1_i  = [  Ang1_i  flip(Ang1_i)  -Ang1_i -flip(Ang1_i)   Ang1_i   flip(Ang1_i)    -Ang1_i   -flip(Ang1_i) ];
 Ang2_j  = [ Ang2_j         Ang2_j    Ang2_j         Ang2_j   -Ang2_j        -Ang2_j    -Ang2_j          -Ang2_j  ];
 Ang3_k = [ Ang3_k       -Ang3_k   Ang3_k       -Ang3_k   Ang3_k       -Ang3_k     Ang3_k         -Ang3_k ];
%          Ang3_k = 0*ones(1,length(Ang1_i));  %No ZYX Yaw rotation


%% 

combinations=length(Ang1_i);
exp_elapse_time_min=199.245667 /216000*combinations/60;
fprintf('Starting time %s\n', datestr(now,'HH:MM:SS.FFF'))
fprintf ('Total combinations = [%f ]  \n', round(combinations,5));
fprintf ('Expected Elapse Time [min] = [%f ]  \n', round(exp_elapse_time_min,5));
fprintf ('Press spacebar to continue  \n');
pause
tic



%% Convert ZYX Setpoints to ZXZ Setpoints, then use as script input
count= 0;
for o=1:length(Ang1_i)
count=count+1

    Ang1= Ang1_i(o); Ang2=Ang2_j(o); Ang3=Ang3_k(o);

%% XYZ sequence

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
% Generate captured image
[X Y  X_prime Y_prime] = UAV_Image_from_USV_configuration(Deg(1),Deg(2),Deg(3),D,n,r);

    %Introduce noise 
    for m=1:length(X_prime)
    a=norm( [X_prime(m) Y_prime(m)])*p; b=-a;
    X_prime_error(m,1) = X_prime(m)+a + (b-a)*rand;
    Y_prime_error(m,1) = Y_prime(m)+a + (b-a)*rand;                 
    end
    X_prime_error=X_prime_error(1:length(X_prime));
    Y_prime_error=Y_prime_error(1:length(X_prime));
    
% Compute position and distance
[Degrees(o,:),Distance(o),coeff_prime(:,o), coeff(:,o),Theta(o),Ma(o),Mi(o), e(o) , e_prime(o)] = USV_configuration_from_UAV_image(X_prime_error,Y_prime_error);

% Compute error
Rz2b = [   cosd(Degrees(o,3))  sind(Degrees(o,3))   0   ;
             -sind(Degrees(o,3))   cosd(Degrees(o,3))  0   ;
                    0                   0                 1] ;
                
Rxb = [  1         0                    0                ;
            0   cosd(Degrees(o,2))  -sind(Degrees(o,2)) ; 
            0   sind(Degrees(o,2))    cosd(Degrees(o,2)) ];        
        
Rz1b = [   cosd(Degrees(o,1))  sind(Degrees(o,1))   0   ;
             -sind(Degrees(o,1))   cosd(Degrees(o,1))  0   ;
                    0                   0           1 ]      ;     
        
R2 = Rz2b*Rxb*Rz1b;


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
         close all
         L=linspace(0,length(Ang1_i),9);
         x_limit=[0 length(Theta)];
        
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
         xlim([x_limit(1) x_limit(2)])

         subplot(4,3,2)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(AnglesZXZ(:,2))
         grid on
         title('Beta Setpoint input', 'FontSize', 24);
         ylabel('Deg')
         xlim([x_limit(1) x_limit(2)])
         
         subplot(4,3,3)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(AnglesZXZ(:,1))
         grid on
         title('Gamma Setpoint input', 'FontSize', 24);
         ylabel('Deg')
         xlim([x_limit(1) x_limit(2)])
         
         subplot(4,3,4)
         plot(Degrees(:,1))
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         title('Alpha Measurement output', 'FontSize', 24);
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
         
         subplot(4,3,5)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Degrees(:,2))
         grid on
         title('Beta Measurement output', 'FontSize', 24);
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
         
         subplot(4,3,6)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Degrees(:,3))
         grid on
         title('Gamma Measurement output', 'FontSize', 24);
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
         
         subplot(4,1,3)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Error2)
         title('Error norm |Pn*Rsetpoint - Pn*Rmeasurement|', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         
         subplot(4,1,4)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Distance)
         title('Height measurement output', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])

         
         
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
         xlim([x_limit(1) x_limit(2)])
         
         subplot(5,3,2)
         plot(Ang2_j)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9));          
         grid on
         title('Pitch Setpoint (not an input)', 'FontSize', 24);
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
         
         subplot(5,3,3)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Ang3_k)
         grid on
         title('Yaw Setpoint (not an input)', 'FontSize', 24);
         ylabel('Deg')  
         DegSP=[Ang1_i' Ang2_j' Ang3_k'];
         xlim([x_limit(1) x_limit(2)])
         
         
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
         xlim([x_limit(1) x_limit(2)])
         
         subplot(4,3,5)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Deg(:,2))
         grid on
         title('Pitch Measurement (not an output)', 'FontSize', 22);
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
         
         subplot(4,3,6)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Deg(:,3))
         grid on
         title('Yaw Measurement (not an output)', 'FontSize', 22);
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
        
        
         subplot(4,1,3)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Error2)
         title('Error norm |Pn*Rsetpoint - Pn*Rmeasurement|', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         
         subplot(4,1,4)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Distance)
         title('Height measurement output', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])

% Plot Coefficients 
        
f3=figure;         
         f3.WindowState = 'maximized';
         
         subplot(7,1,1)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(coeff(1,:))
         title('Coeff B', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         
          subplot(7,1,2)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(coeff(2,:))
         title('Coeff C', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         
          subplot(7,1,3)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(coeff(3,:))
         title('Coeff D', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
                         
         subplot(7,1,4)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(coeff(4,:))
         title('Coeff E', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
                          
         subplot(7,1,5)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(coeff(5,:))
         title('Coeff F', 'FontSize', 20);         
         xlim([x_limit(1) x_limit(2)])
 
         subplot(7,1,6)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Error2)
         title('Error norm |Pn*Rsetpoint - Pn*Rmeasurement|', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         
         subplot(7,1,7)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Mi)
         title('Undistorted Minor Semi-axis', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         
% Plot Coefficients prime 
        
f4=figure;         
         f4.WindowState = 'maximized';
         
         subplot(7,1,1)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(coeff_prime(1,:))
         title('Coeff B prime', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         
          subplot(7,1,2)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(coeff_prime(2,:))
         title('Coeff C prime', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         
          subplot(7,1,3)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(coeff_prime(3,:))
         title('Coeff D prime', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
                          
         subplot(7,1,4)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(coeff_prime(4,:))
         title('Coeff E prime', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
                          
         subplot(7,1,5)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(coeff_prime(5,:))
         title('Coeff F prime', 'FontSize', 20);         
         xlim([x_limit(1) x_limit(2)])
 
         subplot(7,1,6)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Error2)
         title('Error norm |Pn*Rsetpoint - Pn*Rmeasurement|', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         
         subplot(7,1,7)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(e)
         title('Eccentricity', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         
         
         
%Plot Theta and Phi
        
f5=figure;                
         f5.WindowState = 'maximized';
  
         subplot(5,1,1)
         plot(Degrees(:,1))
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         title('Alpha Measurement output', 'FontSize', 24);
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
         
         subplot(5,1,2)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Degrees(:,2))
         grid on
         title('Beta Measurement output', 'FontSize', 24);
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
         
         subplot(5,1,3)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Degrees(:,3))
         grid on
         title('Gamma Measurement output', 'FontSize', 24);
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
         
         subplot(5,1,4)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Error2)
         title('Error norm |Pn*Rsetpoint - Pn*Rmeasurement|', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         
         subplot(5,1,5)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(e)
         title('Eccentricity', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])    
         
 %       
              
f6=figure;                
         f6.WindowState = 'maximized';
  

         subplot(5,1,1)
         plot(Deg(:,1))
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         title('Roll Measurement (not an output)', 'FontSize', 22);
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
         
         subplot(5,1,2)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Deg(:,2))
         grid on
         title('Pitch Measurement (not an output)', 'FontSize', 22);
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
         
         subplot(5,1,3)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Deg(:,3))
         grid on
         title('Yaw Measurement (not an output)', 'FontSize', 22);
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
     
         subplot(5,1,4)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Error2)
         title('Error norm |Pn*Rsetpoint - Pn*Rmeasurement|', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         
         subplot(5,1,5)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(e)
         title('Eccentricity', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])    

 %     
              
f7=figure;                
         f7.WindowState = 'maximized';
           

         subplot(5,1,1)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Error2)
         title('Error norm |Pn*Rsetpoint - Pn*Rmeasurement|', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         
         subplot(5,1,2)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Ma)
         title('Undistorted Major Semi-axis', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         
         
         subplot(5,1,3)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Mi)
         title('Undistorted Minor Semi-axis', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         
         
         
         subplot(5,1,4)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(e)
         title('Eccentricity', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])         
 
         subplot(5,1,5)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(e_prime)
         title('Eccentricity prime', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])     
       
         
   f8=figure;                
   f8.WindowState = 'maximized';
  
         test=coeff_prime(1,:);
         delta=.01;  test(test>delta ) = 0; test(test<-delta ) = 0;
         subplot(4,1,1)         
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
%          plot(sign(test))
         plot(coeff_prime(1,:))
         title('Coeff B prime', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         ylim([-.01 .01])
         
          subplot(4,1,2)  
          test=coeff_prime(2,:);
         test(test>1 ) = -1;
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
%          plot(sign(test))
         plot(coeff_prime(2,:))
         title('Coeff C prime', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         ylim([.85 1.15])

         
         
         subplot(4,1,3)
         test=Error2;
         test(test<1 ) = 0;
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Error2)
%          plot(sign(test))
         title('Error norm |Pn*Rsetpoint - Pn*Rmeasurement|', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         
             
         subplot(4,1,4)
         test=Error2;
         test(test<1 ) = 0;
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Theta)
         title('Theta', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
             
      
  toc
       
         
