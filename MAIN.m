clear all
clc
beep off


%% Input Parameters 

% Other
n=12;        % Beacon Qty
r=10;         % Circle radius
D=20;       %  UAV height (Z-axis distance from circular pattern center to UAV's camera)
p=0.0;     % Measurement noise

% Wave Parameters 
a=[ 2.5];      %Wave amplitude (m)   --- > Range 1.25 to 2.5
beta=[45]*pi/180;   %Wave direction (rad) where beta = pi (i.e. 180 deg) is head seas
speed=[11 16 ]/1.94384; % 1m/s = 1.94384 knots
wave_length=[75 90]; %Wave length in meters
T_0=[   wave_length/speed(2) wave_length/speed(1) ];    %Wave periode (s) corresponding to the wave frequency w_0 = 2 pi/T_0
% T_0=[  9 12 15 ];    %Wave periode (s) corresponding to the wave frequency w_0 = 2 pi/T_0




% Ship Parameters 
L=88;      %Length (m)
B=12.83;      %Breadth(Beam) (m) 
T=2.627;      %Draught(Draft) (m)
GMT=1.1;    %Transver metacentric height (m)
U=0;      %Ship speed (m/s)
Cb=0.419;     %Block coefficeint 
T4=   0.88*B/sqrt(GMT) ;     %Natural roll periode (s)   ==> 0.88*B/sqrt(GMT)     .... https://www.dockwalk.com/technology/understanding-roll-periods
zeta4=0.1;  %Relative damping factor in roll

   
%% Generate Roll Pitch Yaw Setpoints from 0 to 89 Degrees
        Ang1_i =[]; Ang2_j=[]; Ang3_k = []; Heave_k=[]; count= 0; i=1;j=1;k=1;
        for i=1:length(a)
                for j=1:length(beta)
                         for k=1:length(T_0)
                                                          
                             count= count+1;
                  [Roll Pitch Yaw Heave]=Waveresponse(a(i), beta(j),T_0(k), zeta4,T4,GMT, Cb, U, L, B, T);
                  Ang1_i =[Ang1_i Roll]; 
                  Ang2_j=[Ang2_j Pitch];
                  Heave_k = [Heave_k Heave];     
                  Ang3_k=[Ang3_k Yaw];
                         end              
               end
        end
%         Ang3_k = 0*ones(1,length(Ang1_i));  %No ZYX Yaw rotation
    
         
%% 

combinations=length(Ang1_i);
exp_elapse_time_min=1205.678056 /1000000*combinations/60;
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
    Height= D+ Heave_k(o);
%% ZYX sequence

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

    for i=0:n-1
        %30 Degrees because it consider n=12 markers located at USV deck
        rot = [cosd(i*360/n) -sind(i*360/n); sind(i*360/n) cosd(i*360/n)]; 
        M = [  [rot*[ r 0]']' 0];    %Marker points on USV deck
        MrotZYX(i+1,:) = [M*R];     %Marker point to to rotate
    end

%% Input ZXZ coordinates to scripts
% Generate captured image
[X_prime Y_prime] = UAV_Image_from_USV_configuration(Ang1,Ang2,Ang3,Height,n,r);

    %Introduce noise 
    for m=1:length(X_prime)
    a=norm( [X_prime(m) Y_prime(m)])*p; b=-a;
    X_prime_error(m,1) = X_prime(m)+a + (b-a)*rand;
    Y_prime_error(m,1) = Y_prime(m)+a + (b-a)*rand;                 
    end
    X_prime_error=X_prime_error(1:length(X_prime));
    Y_prime_error=Y_prime_error(1:length(X_prime));
    
% Compute position and distance 
[Deg(o,:),Distance(o)] = USV_configuration_from_UAV_image(X_prime_error,Y_prime_error);


%% Compute Error                


Rx = [     1            0                      0           ;
              0        cosd(Deg(o,1))  -sind(Deg(o,1))    ; 
              0        sind(Deg(o,1))   cosd(Deg(o,1))   ];

Ry = [ cosd(Deg(o,2))       0        sind(Deg(o,2))   ;
               0                  1               0          ;
          -sind(Deg(o,2))       0        cosd(Deg(o,2)) ];   

Rz = [  cosd(Deg(o,3))  sind(Deg(o,3))    0   ;
          -sind(Deg(o,3))   cosd(Deg(o,3))   0   ;
                  0                   0            1 ]  ;   
      
R=Rz*Ry*Rx;


    for i=0:n-1
        %30 Degrees because it consider n=12 markers located at USV deck
        rot = [cosd(i*360/n) -sind(i*360/n); sind(i*360/n) cosd(i*360/n)]; 
        M = [  [rot*[ r 0]']' 0];    %Marker points on USV deck
        Mrot_ZYX_Script(i+1,:) = [M*R];     %Marker point to to rotate
    end
    
    
% Error
Error2(o)=norm (MrotZYX - Mrot_ZYX_Script);

end %Main for o

         if combinations<2
    return
end
%% FIG 1
        
         close all
         L=linspace(0,length(Ang1_i),9);
        x_limit=[0 combinations];
        x_limit2=[0 combinations];
        x_axis=linspace(0,length(Ang1_i)/10,length(Ang1_i));        
       
        f(1)=figure;         
        f(1).WindowState = 'maximized';
     
         subplot(3,3,1)
         plot(Ang1_i)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9));  
         grid on
         title('Roll Setpoint ', 'FontSize', 20);
         xlabel('Time [s]')
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
         
         subplot(3,3,2)
         plot(Ang2_j)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9));          
         grid on
         title('Pitch Setpoint ', 'FontSize', 20);
         xlabel('Time [s]')
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
         
         subplot(3,3,3)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Ang3_k)
         grid on
         title('Yaw Setpoint ', 'FontSize', 20);
         xlabel('Time [s]')
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
         
       
         subplot(3,3,4)
         plot(Deg(:,1))
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         title('Roll Measurement ', 'FontSize', 20);
         xlabel('Time [s]')
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
         
         subplot(3,3,5)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Deg(:,2))
         grid on
         title('Pitch Measurement ', 'FontSize', 20);
         xlabel('Time [s]')
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
         
         subplot(3,3,6)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Deg(:,3))
         grid on
         title('Yaw Measurement ', 'FontSize', 20);
         xlabel('Time [s]')
         ylabel('Deg')  
         xlim([x_limit(1) x_limit(2)])
        
        
         subplot(3,1,3)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Error2)
         title('Error norm |Pn*Rsetpoint - Pn*Rmeasurement|', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         xlabel('Time [s]')
         
%          subplot(4,1,4)
%          hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
%          hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
%          grid on
%          plot(Distance)
%          title('Height measurement output', 'FontSize', 20);
%          xlim([x_limit(1) x_limit(2)])
%          xlabel('Time [s]')

%% FIG 2
                     
         f(2)=figure;
         f(2).WindowState = 'maximized';
     
         subplot(3,3,1)
         plot(Ang1_i)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         title('Roll Setpoint input', 'FontSize', 24);
         xlabel('Time [s]')
         ylabel('Deg')
         xlim([x_limit2(1) x_limit2(2)])

         subplot(3,3,2)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Ang2_j)
         grid on
         title('Pitch Setpoint input', 'FontSize', 24);
         xlabel('Time [s]')
         ylabel('Deg')
         xlim([x_limit2(1) x_limit2(2)])
         
         subplot(3,3,3)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Ang3_k)
         grid on
         title('Yaw Setpoint input', 'FontSize', 24);
         xlabel('Time [s]')
         ylabel('Deg')
         xlim([x_limit2(1) x_limit2(2)])
         
         subplot(3,3,4)
         plot(Deg(:,1))
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         title('Roll Output', 'FontSize', 24);
         xlabel('Time [s]')
         ylabel('Deg')  
         xlim([x_limit2(1) x_limit2(2)])
         
         subplot(3,3,5)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Deg(:,2))
         grid on
         title('Pitch output', 'FontSize', 24);
         xlabel('Time [s]')
         ylabel('Deg')  
         xlim([x_limit2(1) x_limit2(2)])
         
         subplot(3,3,6)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Deg(:,3))
         grid on
         title('Yaw output', 'FontSize', 24);
         xlabel('Time [s]')
         ylabel('Deg')  
         xlim([x_limit2(1) x_limit2(2)])
         
  
         subplot(3,3,7)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Ang1_i-Deg(:,1)')
         grid on
         title('Roll Error', 'FontSize', 24);
         xlabel('Time [s]')
         ylabel('Deg')  
         xlim([x_limit2(1) x_limit2(2)])
         

          subplot(3,3,8)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Ang2_j-Deg(:,2)')
         grid on
         title('Pitch Error', 'FontSize', 24);
         xlabel('Time [s]')
         ylabel('Deg')  
         xlim([x_limit2(1) x_limit2(2)])
         
          subplot(3,3,9)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Ang3_k-Deg(:,3)')
         grid on
         title('Yaw Error', 'FontSize', 24);
         xlabel('Time [s]')
         ylabel('Deg')  
         xlim([x_limit2(1) x_limit2(2)]) 
         
         
%% FIG 3
                     
         f(3)=figure;
         f(3).WindowState = 'maximized';
         
         subplot(2,1,1)
         plot(Ang1_i,'LineWidth',2)
         hold on
         plot(Deg(:,1))
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         title('Roll Setpoint input vs Measurement', 'FontSize', 24);
         xlabel('Time [s]')
         ylabel('Deg')
         xlim([x_limit2(1) x_limit2(2)])
         
         subplot(2,1,2)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Ang1_i-Deg(:,1)')
         grid on
         title('Roll Error', 'FontSize', 24);
        xlabel('Time [s]')
        ylabel('Deg')  
         xlim([x_limit2(1) x_limit2(2)])         
%% FIG 4
                     
         f(4)=figure;
         f(4).WindowState = 'maximized';
         
         subplot(2,1,1)
         plot(Ang2_j,'LineWidth',2)
         hold on
         plot(Deg(:,2))
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         title('Pitch Setpoint input vs Measurement', 'FontSize', 24);
         xlabel('Time [s]')
         ylabel('Deg')
         xlim([x_limit2(1) x_limit2(2)])         
         

         subplot(2,1,2)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Ang2_j-Deg(:,2)')
         grid on
         title('Pitch Error', 'FontSize', 24);
         xlabel('Time [s]')
         ylabel('Deg')  
         xlim([x_limit2(1) x_limit2(2)])
         
         
         
%% FIG 5
                     
         f(5)=figure;
         f(5).WindowState = 'maximized';         
         
         
         
         subplot(2,1,1)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Ang3_k,'LineWidth',2)
         hold on
         plot(Deg(:,3))
         title('Yaw Setpoint input vs Measurement', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         xlabel('Time [s]')
         ylabel('Deg')
         
      subplot(2,1,2)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Ang3_k-Deg(:,3)')
         hold on
         title('Yaw Measurement Error', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         xlabel('Time [s]')
         ylabel('Deg')
         
         
%% FIG 6
                     
         f(6)=figure;
         f(6).WindowState = 'maximized';         
         
         
         
         subplot(2,1,1)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Heave_k+D,'LineWidth',2)
         hold on
         plot(Distance)
         title('Height Setpoint input  vs Measurement', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         xlabel('Time [s]')
         ylabel('meters')  
         
      subplot(2,1,2)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Heave_k+D-Distance)
         hold on
         title('Height Measurement Error', 'FontSize', 20);
         xlim([x_limit(1) x_limit(2)])
         xlabel('Time [s]')
         ylabel('meters')           
         

%% FIG 7
                     
        
        f(7)=figure;         
        f(7).WindowState = 'maximized';
     
   subplot(4,1,1)
         plot( x_axis,Ang1_i,'LineWidth',2)
         hold on
         plot( x_axis,Deg(:,1),'LineWidth',1)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
%          title('Roll Setpoint input vs Measurement', 'FontSize', 24);
%          xlabel('Time [s]', 'FontSize', 20)
         ylabel('Roll [Deg]', 'FontSize', 18)
         xlim([x_limit(1) x_limit(2)]/10)

 subplot(4,1,2)
         plot( x_axis,Ang2_j,'LineWidth',2)
         hold on
         plot( x_axis,Deg(:,2),'LineWidth',1)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
%          title('Pitch Setpoint input vs Measurement', 'FontSize', 24);
%          xlabel('Time [s]', 'FontSize', 20)
         ylabel('Pitch [Deg]', 'FontSize', 18)
         xlim([x_limit(1) x_limit(2)]/10)
         
        subplot(4,1,3)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot( x_axis,Ang3_k,'LineWidth',2)
         hold on
         plot( x_axis,Deg(:,3),'LineWidth',1)
         xlim([x_limit(1) x_limit(2)]/10)
         ylabel('Yaw [Deg]', 'FontSize', 18)

   subplot(4,1,4)
         plot( x_axis,Heave_k+D,'LineWidth',2)
         hold on
         plot( x_axis,Distance,'LineWidth',1)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         xlabel('Time [s]', 'FontSize', 20)
         ylabel('Height [m]', 'FontSize', 18)
         xlim([x_limit(1) x_limit(2)]/10)


%% FIG 8
                     
        
        f(8)=figure;         
        f(8).WindowState = 'maximized';         
         
         
         subplot(3,1,1)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Ang1_i-Deg(:,1)')
         grid on
         ylabel('Roll Error [Deg]', 'FontSize', 18)
         xlim([x_limit(1) x_limit(2)]/10)
         
         
         subplot(3,1,2)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         plot(Ang2_j-Deg(:,2)')
         grid on
         ylabel('Pitch Error [Deg]', 'FontSize', 18)
         xlim([x_limit(1) x_limit(2)]/10)
         
         
      subplot(3,1,3)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Ang3_k-Deg(:,3)')
         hold on
         xlim([x_limit(1) x_limit(2)]/10)
         ylabel('Yaw Error [Deg]', 'FontSize', 18)
         
         f(9)=figure;         
        f(9).WindowState = 'maximized';         
                 
      subplot(3,1,1)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Heave_k+D-Distance)
         hold on
         xlim([x_limit(1) x_limit(2)]/10)
         ylabel('Height Error [m]', 'FontSize', 18)

         subplot(3,1,2)
         hold on; xline(L(2)); hold on; xline(L(3)); hold on; xline(L(4)); hold on; xline(L(5));  
         hold on; xline(L(6)); hold on; xline(L(7)); hold on; xline(L(8)); hold on; xline(L(9)); 
         grid on
         plot(Error2)
         xlim([x_limit(1) x_limit(2)]/10)
         xlabel('Time [s]')         
         ylabel('Error 2-norm', 'FontSize', 18)
         
         
         
         
         
         %%
         ErrorRoll          =   [ max(abs(Ang1_i-Deg(:,1)'))   min(abs(Ang1_i-Deg(:,1)')) mean(abs(Ang1_i-Deg(:,1)'))]
         ErrorPitch        =   [ max(abs(Ang2_j-Deg(:,2)')) min(abs(Ang2_j-Deg(:,2)')) mean(abs(Ang2_j-Deg(:,2)'))]
         ErrorYaw         =   [ max(abs(Ang3_k-Deg(:,3)')) min(abs(Ang3_k-Deg(:,3)')) mean(abs(Ang3_k-Deg(:,3)'))]
         ErrorDistance =   [ max(abs((Heave_k+D-Distance))) min(abs((Heave_k+D-Distance))) mean(abs((Heave_k+D-Distance)))]

                  
% close(f(1))
close(f(2))
         
         
        
         toc

         
         
         
function [Roll Pitch Yaw Heave]=Waveresponse1(a, beta,T_0, zeta4,T4,GMT, Cb, U, L, B, T)

%% Equations - Don't Modify
% Refer paper "Estimation of shipmotions using closed-form expressions", Jensen et al. (2004)
% Refer https://github.com/cybergalactic/MSS

% Constants
g = 9.81;                 % acceleration of gravity (m/s^2)
rho = 1025;               % density of water (kg/m^3)

% Ship parameters
nabla = Cb * L * B * T;        % volume displacement (m^3) 
w_0 = 2 * pi / T_0;            % wave peak frequency (rad/s)
k = w_0^2/g;                   % wave number
w_e = w_0 - k * U * cos(beta); % frequency of encounter
k_e = abs(k * cos(beta));      % effective wave number
sigma = k_e * L/2;
kappa = exp(-k_e * T);

% Heave and pitch models (Jensen et al., 2004)
A = 2 * sin(k*B*(w_e/w_0)^2/2) * exp(-k*T*(w_e/w_0)^2); 
f = sqrt( (1-k*T)^2  + (A^2/(k*B*(w_e/w_0)^3))^2 );
F = kappa * f * sin(sigma)/sigma;
G = kappa * f * (6/L) * (1/sigma) * ( sin(sigma)/sigma - cos(sigma) );
wn  = sqrt(g/(2*T));
zeta = (A^2/(B*(w_e/w_0)^3)) * sqrt(1/(8*k^3*T));

% Roll model (simplifed version of Jensen et al., 2004)
w4 = 2*pi/T4;                    % natural frequency
C44 = rho * g * nabla * GMT;     % spring coeffient
M44 = C44/w4^2;                  % moment of inertia including added mass
B44 = 2 * zeta4 * w4 * M44;      % damping coefficient
M = sin(beta) * sqrt( B44 * rho*g^2/w_e );   % roll moment amplitude
t = 0:0.1:T_0;    % time vector for plotting
    
% Heave
s = sqrt( (2*wn*zeta)^2 + (1/w_e^2)*(wn^2-w_e^2)^2 );
eps = atan( 2*w_e*wn*zeta/(wn^2-w_e^2) );
Heave = (a*F*wn^2/(s*w_e)) * cos(w_e*t+eps);
    
% Pitch
% s = sqrt( (2*wn*zeta)^2 + (1/w_e^2)*(wn^2-w_e^2)^2 );
% eps5 = atan( 2*w_e*wn*zeta/(wn^2-w_e^2) );
Pitch = (180/pi) * (a*G*wn^2/(s*w_e)) * sin(w_e*t+eps);
    
% Roll
s = sqrt( (2*w4*zeta4)^2 + (1/w_e^2)*(w4^2-w_e^2)^2 );
eps = atan( 2*w_e*w4*zeta4/(w4^2-w_e^2) );
Roll = (180/pi) * ((M/C44)*w4^2/(s*w_e)) * cos(w_e*t+eps);


%Yaw

Yaw=Heave.*sin(0:0.1:T_0);

end

