function [Degrees,Heigth,coeff_prime,coeff, Theta_prime,Ma, Mi,e,e_prime] = USV_Configuration_Ellipses_Test(X_prime,Y_prime)


%% Camera parameters to correct perspective distortion
Fangle=50; %Focal angle, the bigger the angle the better
fx=1920*sind(90-Fangle) /sind(Fangle);
fy=fx;
s=fx*tand(0);
Width=3840; %Image resolution
Height=2160; %Image resolution
cx =Width/2; %width
cy =Height/2; %height
r=10;%USV deck circle radius

%% Camera plane normal
P1=[ 0 1 0];    P2=[ 1 0 0];    P3=[0 0 0];
N=cross(P1,P2);                        %Not normalized yet
d=N*P1';                                    %d

%% Ellipse fit

[a_prime, b_prime, k, h, Theta_prime, coeff_prime] = ellipse_fit(X_prime, Y_prime);
Ma_prime = max(a_prime, b_prime); Mi_prime = min(a_prime, b_prime);


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
t =  ( d - N(1)*P3d(i,1) - N(2)*P3d(i,2) - N(3)*P3d(i,3) ) / ( N(1)*N(1) + N(2)*N(2) + N(3)*N(3) );  
rot = [cosd(90) -sind(90); sind(90) cosd(90)]; 
P2d(i,:)=[rot*[ N(1)*t+P3d(i,1) N(2)*t+P3d(i,2)]']';

end % End for loop i



Heigth=mean(Heigth);



%% Undistort Major and Minor Semi-axis

phi=-sign(Theta_prime)*(90 - abs(Theta_prime));
X1new= (0)*cosd(-phi)-(1)*sind(-phi);
Y1new= (0)*sind(-phi)+(1)*cosd(-phi);
V=Ma_prime*[X1new Y1new]+[k h];
V=[V(1) V(2)]/norm([V(1) V(2)]);
c(1)=(Mi_prime*Ma_prime*(Mi_prime^2*V(1)^2*cos((pi*Theta_prime)/180)^2 + Ma_prime^2*V(2)^2*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)^2*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)^2*sin((pi*Theta_prime)/180)^2 - V(1)^2*h^2*cos((pi*Theta_prime)/180)^4 - V(2)^2*k^2*cos((pi*Theta_prime)/180)^4 - V(1)^2*h^2*sin((pi*Theta_prime)/180)^4 - V(2)^2*k^2*sin((pi*Theta_prime)/180)^4 - 2*V(1)^2*h^2*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2 - 2*V(2)^2*k^2*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2 - 2*Mi_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*Ma_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*V(1)*V(2)*h*k*cos((pi*Theta_prime)/180)^4 + 2*V(1)*V(2)*h*k*sin((pi*Theta_prime)/180)^4 + 4*V(1)*V(2)*h*k*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2)^(1/2) + Ma_prime^2*V(2)*h*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(1)*k*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)*h*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)*k*sin((pi*Theta_prime)/180)^2 - Mi_prime^2*V(1)*h*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + Ma_prime^2*V(1)*h*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) - Mi_prime^2*V(2)*k*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + Ma_prime^2*V(2)*k*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180))/(Mi_prime^2*V(1)^2*cos((pi*Theta_prime)/180)^2 + Ma_prime^2*V(2)^2*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)^2*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)^2*sin((pi*Theta_prime)/180)^2 - 2*Mi_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*Ma_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180));
c(2)=(Ma_prime^2*V(2)*h*cos((pi*Theta_prime)/180)^2 - Mi_prime*Ma_prime*(Mi_prime^2*V(1)^2*cos((pi*Theta_prime)/180)^2 + Ma_prime^2*V(2)^2*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)^2*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)^2*sin((pi*Theta_prime)/180)^2 - V(1)^2*h^2*cos((pi*Theta_prime)/180)^4 - V(2)^2*k^2*cos((pi*Theta_prime)/180)^4 - V(1)^2*h^2*sin((pi*Theta_prime)/180)^4 - V(2)^2*k^2*sin((pi*Theta_prime)/180)^4 - 2*V(1)^2*h^2*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2 - 2*V(2)^2*k^2*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2 - 2*Mi_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*Ma_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*V(1)*V(2)*h*k*cos((pi*Theta_prime)/180)^4 + 2*V(1)*V(2)*h*k*sin((pi*Theta_prime)/180)^4 + 4*V(1)*V(2)*h*k*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2)^(1/2) + Mi_prime^2*V(1)*k*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)*h*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)*k*sin((pi*Theta_prime)/180)^2 - Mi_prime^2*V(1)*h*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + Ma_prime^2*V(1)*h*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) - Mi_prime^2*V(2)*k*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + Ma_prime^2*V(2)*k*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180))/(Mi_prime^2*V(1)^2*cos((pi*Theta_prime)/180)^2 + Ma_prime^2*V(2)^2*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)^2*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)^2*sin((pi*Theta_prime)/180)^2 - 2*Mi_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*Ma_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180));
Xbar = -(2*V(2)*max(c)*min(c))/(fy*max(c) - fy*min(c));
Ybar = (2*max(c)*min(c)*(V(1)*fy + V(2)*s))/(fx*fy*(max(c) - min(c)));
Zbar = -(max(c) + min(c))/(max(c) - min(c));
Pma3d=r*[Xbar Ybar Zbar]/norm([Xbar Ybar Zbar]);
t =  ( d - N(1)*Pma3d(1) - N(2)*Pma3d(2) - N(3)*Pma3d(3) ) / ( N(1)*N(1) + N(2)*N(2) + N(3)*N(3) );  
rot = [cosd(90) -sind(90); sind(90) cosd(90)]; 
Pma2D=[rot*[ N(1)*t+Pma3d(1) N(2)*t+Pma3d(2)]']';
a=norm(Pma2D);

X1new= (0)*cosd(-Theta_prime)-(1)*sind(-Theta_prime);
Y1new= (0)*sind(-Theta_prime)+(1)*cosd(-Theta_prime);
V=Mi_prime*[X1new Y1new]+[k h];
V=[V(1) V(2)]/norm([V(1) V(2)]);
c(1)=(Mi_prime*Ma_prime*(Mi_prime^2*V(1)^2*cos((pi*Theta_prime)/180)^2 + Ma_prime^2*V(2)^2*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)^2*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)^2*sin((pi*Theta_prime)/180)^2 - V(1)^2*h^2*cos((pi*Theta_prime)/180)^4 - V(2)^2*k^2*cos((pi*Theta_prime)/180)^4 - V(1)^2*h^2*sin((pi*Theta_prime)/180)^4 - V(2)^2*k^2*sin((pi*Theta_prime)/180)^4 - 2*V(1)^2*h^2*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2 - 2*V(2)^2*k^2*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2 - 2*Mi_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*Ma_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*V(1)*V(2)*h*k*cos((pi*Theta_prime)/180)^4 + 2*V(1)*V(2)*h*k*sin((pi*Theta_prime)/180)^4 + 4*V(1)*V(2)*h*k*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2)^(1/2) + Ma_prime^2*V(2)*h*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(1)*k*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)*h*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)*k*sin((pi*Theta_prime)/180)^2 - Mi_prime^2*V(1)*h*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + Ma_prime^2*V(1)*h*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) - Mi_prime^2*V(2)*k*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + Ma_prime^2*V(2)*k*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180))/(Mi_prime^2*V(1)^2*cos((pi*Theta_prime)/180)^2 + Ma_prime^2*V(2)^2*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)^2*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)^2*sin((pi*Theta_prime)/180)^2 - 2*Mi_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*Ma_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180));
c(2)=(Ma_prime^2*V(2)*h*cos((pi*Theta_prime)/180)^2 - Mi_prime*Ma_prime*(Mi_prime^2*V(1)^2*cos((pi*Theta_prime)/180)^2 + Ma_prime^2*V(2)^2*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)^2*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)^2*sin((pi*Theta_prime)/180)^2 - V(1)^2*h^2*cos((pi*Theta_prime)/180)^4 - V(2)^2*k^2*cos((pi*Theta_prime)/180)^4 - V(1)^2*h^2*sin((pi*Theta_prime)/180)^4 - V(2)^2*k^2*sin((pi*Theta_prime)/180)^4 - 2*V(1)^2*h^2*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2 - 2*V(2)^2*k^2*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2 - 2*Mi_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*Ma_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*V(1)*V(2)*h*k*cos((pi*Theta_prime)/180)^4 + 2*V(1)*V(2)*h*k*sin((pi*Theta_prime)/180)^4 + 4*V(1)*V(2)*h*k*cos((pi*Theta_prime)/180)^2*sin((pi*Theta_prime)/180)^2)^(1/2) + Mi_prime^2*V(1)*k*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)*h*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)*k*sin((pi*Theta_prime)/180)^2 - Mi_prime^2*V(1)*h*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + Ma_prime^2*V(1)*h*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) - Mi_prime^2*V(2)*k*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + Ma_prime^2*V(2)*k*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180))/(Mi_prime^2*V(1)^2*cos((pi*Theta_prime)/180)^2 + Ma_prime^2*V(2)^2*cos((pi*Theta_prime)/180)^2 + Mi_prime^2*V(2)^2*sin((pi*Theta_prime)/180)^2 + Ma_prime^2*V(1)^2*sin((pi*Theta_prime)/180)^2 - 2*Mi_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180) + 2*Ma_prime^2*V(1)*V(2)*cos((pi*Theta_prime)/180)*sin((pi*Theta_prime)/180));
Xbar = -(2*V(2)*max(c)*min(c))/(fy*max(c) - fy*min(c));
Ybar = (2*max(c)*min(c)*(V(1)*fy + V(2)*s))/(fx*fy*(max(c) - min(c)));
Zbar = -(max(c) + min(c))/(max(c) - min(c));
Pmi3d=r*[Xbar Ybar Zbar]/norm([Xbar Ybar Zbar]);
t =  ( d - N(1)*Pmi3d(1) - N(2)*Pmi3d(2) - N(3)*Pmi3d(3) ) / ( N(1)*N(1) + N(2)*N(2) + N(3)*N(3) );  
rot = [cosd(90) -sind(90); sind(90) cosd(90)]; 
Pmi2D=[rot*[ N(1)*t+Pmi3d(1) N(2)*t+Pmi3d(2)]']';
b=norm(Pmi2D);

Ma = max(a, b); Mi = min(a, b);


%% Alternative option to compute  undistorted Semi-axis 
%Note, this method is more sensible to noise bur I added so you can take it
%as reference
P2d=double(P2d);
[ignore1, ignore2, ignore3, ignore4, Theta, coeff] = ellipse_fit(P2d(:,1),P2d(:,2));
% Ma = max(a, b); Mi = min(a, b);



%% Compute Alpha, Beta and Gamma


Alpha=sign(Theta_prime)*(90-abs(Theta_prime) ) -90*(sign(sign(Theta_prime)*(90-abs(Theta_prime) ))+ sign(c(1)+c(2)));

Beta=acosd(Mi/r);

Pma3D=[ r*sind(Theta_prime)  r*cosd(Theta_prime) 0];
Pbow=double([ P3d(1,1) P3d(1,2) P3d(1,3) ]);
Gamma = sign(P3d(1,3))*sign(c(1)+c(2))*atan2d(norm(cross(Pma3D,Pbow)), dot(Pma3D,Pbow))+90*(1+sign(c(1)+c(2)))-180*(sign(P3d(1,3))+1)*sign(1+sign(c(1)+c(2)));


%% Circle eccentricity

e_prime=sqrt(1-Mi_prime^2/Ma_prime^2); %Eccentricity of distorted Semi-axis
e=sqrt(1-Mi^2/r^2);%Eccentricity of undistorted Semi-axis

%% OUTPUTS

Degrees = [ Alpha Beta Gamma];

end




function [semimajor_axis, semiminor_axis, x0, y0, Theta , coeff ,test] = ellipse_fit(x, y)

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
semimajor_axis = a_prime;
semiminor_axis = b_prime;


%% Compute Theta
Theta = 0.5 * acotd((C-A)/(2*B));
if (a_prime < b_prime)
    Theta = 90 + Theta;
end

if B < 0 & Theta>90
Theta=-180+Theta;
end

limit=.01;
if B > -limit & B < limit & D > -limit & D < limit
Theta=0;
end


end