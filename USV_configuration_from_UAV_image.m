function [XYZ,Heigth ]  = USV_Configuration_Ellipses_Test(X_prime,Y_prime)


%% Camera parameters to correct perspective distortion
Fangle=50; %Focal angle, the bigger the angle the better
fx=1920*sind(90-Fangle) /sind(Fangle);
fy=fx; %Asuming square pixels
s=fx*tand(0);
Width=3840; %Image resolution
Height=2160; %Image resolution
cx =Width/2; %Image width resolution
cy =Height/2; %Image height resolution
r=10;%USV deck circle radius

%% Camera plane normal
P1=[ 0 1 0];    P2=[ 1 0 0];    P3=[0 0 0];
N=cross(P1,P2);                        %Not normalized yet
d=N*P1';                                    %d

%% Ellipse fit

[a_prime, b_prime, k, h, Theta_prime, coeff_prime] = ellipse_fit(X_prime, Y_prime);
Ma_prime = max(a_prime, b_prime); 
Mi_prime = min(a_prime, b_prime);

%% Compute IR 3D positions

for i=1:length(X_prime)
%% Get IR Led opposite position homologous

V=[X_prime(i) Y_prime(i)]/norm([X_prime(i) Y_prime(i)]);
c(1)=(Mi_prime*Ma_prime*(Mi_prime^2*V(1)^2*cos((pi*Theta_prime)/180)^2 + Ma_prime^2*V(2)^2*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)^2*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)^2*sin((pi*Theta_prime)/180)^2 - V(1)^2*h^2*cos((pi*Theta_prime)/180)^4 - V(2)^2*k^2*cos((pi*Theta_prime)/180)^4 - V(1)^2*h^2*sin((pi*Theta_prime)/180)^4 - V(2)^2*k^2*sin((pi*Theta_prime)/180)^4 - 2*V(1)^2*h^2*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2 - 2*V(2)^2*k^2*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2 - 2*Mi_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*Ma_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*V(1)*V(2)*h*k*cos((pi*Theta_prime)/180)^4 + 2*V(1)*V(2)*h*k*sin((pi*Theta_prime)/180)^4 + 4*V(1)*V(2)*h*k*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2)^(1/2) + Ma_prime^2*V(2)*h*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(1)*k*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)*h*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)*k*sin((pi*Theta_prime)/180)^2 - Mi_prime^2*V(1)*h*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + Ma_prime^2*V(1)*h*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) - Mi_prime^2*V(2)*k*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + Ma_prime^2*V(2)*k*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180))/(Mi_prime^2*V(1)^2*cos((pi*Theta_prime)/180)^2 + Ma_prime^2*V(2)^2*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)^2*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)^2*sin((pi*Theta_prime)/180)^2 - 2*Mi_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*Ma_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180));
c(2)=(Ma_prime^2*V(2)*h*cos((pi*Theta_prime)/180)^2 - Mi_prime*Ma_prime*(Mi_prime^2*V(1)^2*cos((pi*Theta_prime)/180)^2 + Ma_prime^2*V(2)^2*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)^2*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)^2*sin((pi*Theta_prime)/180)^2 - V(1)^2*h^2*cos((pi*Theta_prime)/180)^4 - V(2)^2*k^2*cos((pi*Theta_prime)/180)^4 - V(1)^2*h^2*sin((pi*Theta_prime)/180)^4 - V(2)^2*k^2*sin((pi*Theta_prime)/180)^4 - 2*V(1)^2*h^2*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2 - 2*V(2)^2*k^2*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2 - 2*Mi_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*Ma_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*V(1)*V(2)*h*k*cos((pi*Theta_prime)/180)^4 + 2*V(1)*V(2)*h*k*sin((pi*Theta_prime)/180)^4 + 4*V(1)*V(2)*h*k*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2)^(1/2) + Mi_prime^2*V(1)*k*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)*h*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)*k*sin((pi*Theta_prime)/180)^2 - Mi_prime^2*V(1)*h*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + Ma_prime^2*V(1)*h*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) - Mi_prime^2*V(2)*k*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + Ma_prime^2*V(2)*k*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180))/(Mi_prime^2*V(1)^2*cos((pi*Theta_prime)/180)^2 + Ma_prime^2*V(2)^2*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)^2*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)^2*sin((pi*Theta_prime)/180)^2 - 2*Mi_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*Ma_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180));
 

%% Undistort 3D point

Xbar = -(2*V(2)*max(c)*min(c))/(fy*max(c) - fy*min(c));
Ybar = (2*max(c)*min(c)*(V(1)*fy + V(2)*s))/(fx*fy*(max(c) - min(c)));
Zbar = -(max(c) + min(c))/(max(c) - min(c));

%% Height 
Heigth(i)=r/norm([Xbar Ybar Zbar]);

%% 3D point projection to camera 2D plane
P3d(i,:)=r*[Xbar Ybar Zbar]/norm([Xbar Ybar Zbar]);
% t =  ( d - N(1)*P3d(i,1) - N(2)*P3d(i,2) - N(3)*P3d(i,3) ) / ( N(1)*N(1) + N(2)*N(2) + N(3)*N(3) )
% rot = [cosd(90) -sind(90); sind(90) cosd(90)] 
% P2d(i,:)=[rot*[ N(1)*t+P3d(i,1) N(2)*t+P3d(i,2)]']'
P2d(i,:)=[  -P3d(i,2) P3d(i,1)];

end % End for loop i

%% Height 
Heigth=mean(Heigth);

%% Compute 2D ellipse
[a, b, ignore1, ignore2, Theta, ignore3] = ellipse_fit(P2d(:,1),P2d(:,2));
Ma = 10; Mi = min(a, b);

%%  Minor Semi-axis

X1new= (0)*cosd(-Theta)-(1)*sind(-Theta);
Y1new= (0)*sind(-Theta)+(1)*cosd(-Theta);
V=Mi_prime*[X1new Y1new]+[k h];
V=[V(1) V(2)]/norm([V(1) V(2)]);
c(1)=(Mi_prime*Ma_prime*(Mi_prime^2*V(1)^2*cos((pi*Theta)/180)^2 + Ma_prime^2*V(2)^2*cos((pi*Theta)/180)^2 + Mi_prime^2*V(2)^2*sin((pi*Theta)/180)^2 + Ma_prime^2*V(1)^2*sin((pi*Theta)/180)^2 - V(1)^2*h^2*cos((pi*Theta)/180)^4 - V(2)^2*k^2*cos((pi*Theta)/180)^4 - V(1)^2*h^2*sin((pi*Theta)/180)^4 - V(2)^2*k^2*sin((pi*Theta)/180)^4 - 2*V(1)^2*h^2*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2 - 2*V(2)^2*k^2*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2 - 2*Mi_prime^2*V(1)*V(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*Ma_prime^2*V(1)*V(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*V(1)*V(2)*h*k*cos((pi*Theta)/180)^4 + 2*V(1)*V(2)*h*k*sin((pi*Theta)/180)^4 + 4*V(1)*V(2)*h*k*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2)^(1/2) + Ma_prime^2*V(2)*h*cos((pi*Theta)/180)^2 + Mi_prime^2*V(1)*k*cos((pi*Theta)/180)^2 + Mi_prime^2*V(2)*h*sin((pi*Theta)/180)^2 + Ma_prime^2*V(1)*k*sin((pi*Theta)/180)^2 - Mi_prime^2*V(1)*h*cos((pi*Theta)/180)*sin((pi*Theta)/180) + Ma_prime^2*V(1)*h*cos((pi*Theta)/180)*sin((pi*Theta)/180) - Mi_prime^2*V(2)*k*cos((pi*Theta)/180)*sin((pi*Theta)/180) + Ma_prime^2*V(2)*k*cos((pi*Theta)/180)*sin((pi*Theta)/180))/(Mi_prime^2*V(1)^2*cos((pi*Theta)/180)^2 + Ma_prime^2*V(2)^2*cos((pi*Theta)/180)^2 + Mi_prime^2*V(2)^2*sin((pi*Theta)/180)^2 + Ma_prime^2*V(1)^2*sin((pi*Theta)/180)^2 - 2*Mi_prime^2*V(1)*V(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*Ma_prime^2*V(1)*V(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180));
c(2)=(Ma_prime^2*V(2)*h*cos((pi*Theta)/180)^2 - Mi_prime*Ma_prime*(Mi_prime^2*V(1)^2*cos((pi*Theta)/180)^2 + Ma_prime^2*V(2)^2*cos((pi*Theta)/180)^2 + Mi_prime^2*V(2)^2*sin((pi*Theta)/180)^2 + Ma_prime^2*V(1)^2*sin((pi*Theta)/180)^2 - V(1)^2*h^2*cos((pi*Theta)/180)^4 - V(2)^2*k^2*cos((pi*Theta)/180)^4 - V(1)^2*h^2*sin((pi*Theta)/180)^4 - V(2)^2*k^2*sin((pi*Theta)/180)^4 - 2*V(1)^2*h^2*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2 - 2*V(2)^2*k^2*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2 - 2*Mi_prime^2*V(1)*V(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*Ma_prime^2*V(1)*V(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*V(1)*V(2)*h*k*cos((pi*Theta)/180)^4 + 2*V(1)*V(2)*h*k*sin((pi*Theta)/180)^4 + 4*V(1)*V(2)*h*k*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2)^(1/2) + Mi_prime^2*V(1)*k*cos((pi*Theta)/180)^2 + Mi_prime^2*V(2)*h*sin((pi*Theta)/180)^2 + Ma_prime^2*V(1)*k*sin((pi*Theta)/180)^2 - Mi_prime^2*V(1)*h*cos((pi*Theta)/180)*sin((pi*Theta)/180) + Ma_prime^2*V(1)*h*cos((pi*Theta)/180)*sin((pi*Theta)/180) - Mi_prime^2*V(2)*k*cos((pi*Theta)/180)*sin((pi*Theta)/180) + Ma_prime^2*V(2)*k*cos((pi*Theta)/180)*sin((pi*Theta)/180))/(Mi_prime^2*V(1)^2*cos((pi*Theta)/180)^2 + Ma_prime^2*V(2)^2*cos((pi*Theta)/180)^2 + Mi_prime^2*V(2)^2*sin((pi*Theta)/180)^2 + Ma_prime^2*V(1)^2*sin((pi*Theta)/180)^2 - 2*Mi_prime^2*V(1)*V(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*Ma_prime^2*V(1)*V(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180));
% Xbar = -(2*V(2)*max(c)*min(c))/(fy*max(c) - fy*min(c));
% Ybar = (2*max(c)*min(c)*(V(1)*fy + V(2)*s))/(fx*fy*(max(c) - min(c)));
Zbar = -(max(c) + min(c))/(max(c) - min(c));
% Pmi3d=sign(Theta)*r*[Xbar Ybar Zbar]/norm([Xbar Ybar Zbar]);
Pmi3d(3)=sign(Theta)*Zbar;
%% ZXZ euler angles

phi=sign(Theta)*(90 - abs(Theta));
Pma3d=[  r*cosd(phi)   r*sind(phi) 0];
Pbow=[ P3d(1,1) P3d(1,2) P3d(1,3) ];

Gamma=-sign( Pmi3d(3))*sign(Pbow(3))*acosd(1-(norm(Pma3d-Pbow)^2)/ (2*r^2));

Beta=sign( Pmi3d(3))*acosd(Mi/r);

Alpha=phi;

Degree = [ Alpha Beta Gamma];



%% Convert to quaternion , then to XYZ

Deg = 0.5*[ Alpha Beta Gamma]*pi/180;

q =    [ cos(Deg(3))*cos(Deg(2))*cos(Deg(1)) - sin(Deg(3))*cos(Deg(2))*sin(Deg(1)), ...
            cos(Deg(3))*sin(Deg(2))*cos(Deg(1)) + sin(Deg(3))*sin(Deg(2))*sin(Deg(1)), ...
            sin(Deg(3))*sin(Deg(2))*cos(Deg(1)) - cos(Deg(3))*sin(Deg(2))*sin(Deg(1)), ...
            cos(Deg(3))*cos(Deg(2))*sin(Deg(1)) + sin(Deg(3))*cos(Deg(2))*cos(Deg(1))];
        
        
q0=q(1); q1=q(2); q2=q(3); q3=q(4);        
        
% Roll (X-axis rotation)
sinr_cosp = 2.0 * (q0 * q1 + q2 * q3);
cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2);
Roll = atan2(sinr_cosp, cosr_cosp)*180/pi;

% Pitch (Y-axis rotation)
sinp = 2.0 * (q0 * q2 - q3 * q1);
Pitch = -asin(sinp)*180/pi;

% yaw (z-axis rotation)
siny_cosp = 2.0 * (q0 * q3 + q1 * q2);
cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3);
Yaw = atan2(siny_cosp, cosy_cosp)*180/pi;

XYZ=[Roll Pitch Yaw];

end




function [a_prime, b_prime, x0, y0, Theta , coeff ] = ellipse_fit(x, y)

x = x(:);
y = y(:);

M = [2*x.*y y.^2 2*x 2*y ones(size(x))];
coeff = M\(-x.^2);

%Extract parameters from vector e
A = 1;
B = coeff(1);
C = coeff(2);
D = coeff(3);
E = coeff(4);
F = coeff(5);

delta = B^2-A*C;

%Offset
x0 = (C*D - B*E)/delta;
y0 = (A*E - B*D)/delta;

 %Semi axis
nom = 2 * (A*E^2 + C*D^2 + F*B^2 - 2*B*D*E - A*C*F);
s = sqrt(1 + (4*B^2)/(A-C)^2);
a_prime = sqrt(nom/(delta*( (C-A)*s -(C+A))));
b_prime = sqrt(nom/(delta*( (A-C)*s -(C+A))));

%% Compute Theta
Theta = 0.5 * acotd((C-A)/(2*B));

if (a_prime < b_prime)
    Theta = 90 + Theta;
end

if B < 0 & Theta>90
Theta=-180+Theta;
end


end