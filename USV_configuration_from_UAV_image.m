function [Degrees,Distance,coeff,Theta,Mi] = USV_Configuration_Ellipses_Test(X_prime,Y_prime)


[a_prime, b_prime, k, h, Theta, coeff] = ellipse_fit(X_prime, Y_prime);
ellipse_t = fit_ellipse( Y_prime,X_prime);
Ma = max(a_prime, b_prime); Mi = min(a_prime, b_prime);

%% Camera parameters to correct perspective distortion
Fangle=50; %Focal angle, the bigger the angle the better
fx=1920*sind(90-Fangle) /sind(Fangle);
fy=fx;
s=fx*tand(0);
Width=3840; %Image resolution
Height=2160; %Image resolution
cx =Width/2; %width
cy =Height/2; %height
r=10; %USV deck circle radius

%% Camera plane normal
P1=[ 0 1 0];    P2=[ 1 0 0];    P3=[0 0 0];
N=cross(P1,P2);                        %Not normalized yet
d=N*P1';                                    %d

%% Compute IR 3D positions

for i=1:length(X_prime)
%% Get IR Led opposite position homologous

Pn=[X_prime(i) Y_prime(i)]/norm([X_prime(i) Y_prime(i)]);
c(1)=(Mi*Ma*(Mi^2*Pn(1)^2*cos((pi*Theta)/180)^2 + Ma^2*Pn(2)^2*cos((pi*Theta)/180)^2 + Mi^2*Pn(2)^2*sin((pi*Theta)/180)^2 + Ma^2*Pn(1)^2*sin((pi*Theta)/180)^2 - Pn(1)^2*h^2*cos((pi*Theta)/180)^4 - Pn(2)^2*k^2*cos((pi*Theta)/180)^4 - Pn(1)^2*h^2*sin((pi*Theta)/180)^4 - Pn(2)^2*k^2*sin((pi*Theta)/180)^4 - 2*Pn(1)^2*h^2*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2 - 2*Pn(2)^2*k^2*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2 - 2*Mi^2*Pn(1)*Pn(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*Ma^2*Pn(1)*Pn(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*Pn(1)*Pn(2)*h*k*cos((pi*Theta)/180)^4 + 2*Pn(1)*Pn(2)*h*k*sin((pi*Theta)/180)^4 + 4*Pn(1)*Pn(2)*h*k*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2)^(1/2) + Ma^2*Pn(2)*h*cos((pi*Theta)/180)^2 + Mi^2*Pn(1)*k*cos((pi*Theta)/180)^2 + Mi^2*Pn(2)*h*sin((pi*Theta)/180)^2 + Ma^2*Pn(1)*k*sin((pi*Theta)/180)^2 - Mi^2*Pn(1)*h*cos((pi*Theta)/180)*sin((pi*Theta)/180) + Ma^2*Pn(1)*h*cos((pi*Theta)/180)*sin((pi*Theta)/180) - Mi^2*Pn(2)*k*cos((pi*Theta)/180)*sin((pi*Theta)/180) + Ma^2*Pn(2)*k*cos((pi*Theta)/180)*sin((pi*Theta)/180))/(Mi^2*Pn(1)^2*cos((pi*Theta)/180)^2 + Ma^2*Pn(2)^2*cos((pi*Theta)/180)^2 + Mi^2*Pn(2)^2*sin((pi*Theta)/180)^2 + Ma^2*Pn(1)^2*sin((pi*Theta)/180)^2 - 2*Mi^2*Pn(1)*Pn(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*Ma^2*Pn(1)*Pn(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180));
c(2)=(Ma^2*Pn(2)*h*cos((pi*Theta)/180)^2 - Mi*Ma*(Mi^2*Pn(1)^2*cos((pi*Theta)/180)^2 + Ma^2*Pn(2)^2*cos((pi*Theta)/180)^2 + Mi^2*Pn(2)^2*sin((pi*Theta)/180)^2 + Ma^2*Pn(1)^2*sin((pi*Theta)/180)^2 - Pn(1)^2*h^2*cos((pi*Theta)/180)^4 - Pn(2)^2*k^2*cos((pi*Theta)/180)^4 - Pn(1)^2*h^2*sin((pi*Theta)/180)^4 - Pn(2)^2*k^2*sin((pi*Theta)/180)^4 - 2*Pn(1)^2*h^2*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2 - 2*Pn(2)^2*k^2*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2 - 2*Mi^2*Pn(1)*Pn(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*Ma^2*Pn(1)*Pn(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*Pn(1)*Pn(2)*h*k*cos((pi*Theta)/180)^4 + 2*Pn(1)*Pn(2)*h*k*sin((pi*Theta)/180)^4 + 4*Pn(1)*Pn(2)*h*k*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2)^(1/2) + Mi^2*Pn(1)*k*cos((pi*Theta)/180)^2 + Mi^2*Pn(2)*h*sin((pi*Theta)/180)^2 + Ma^2*Pn(1)*k*sin((pi*Theta)/180)^2 - Mi^2*Pn(1)*h*cos((pi*Theta)/180)*sin((pi*Theta)/180) + Ma^2*Pn(1)*h*cos((pi*Theta)/180)*sin((pi*Theta)/180) - Mi^2*Pn(2)*k*cos((pi*Theta)/180)*sin((pi*Theta)/180) + Ma^2*Pn(2)*k*cos((pi*Theta)/180)*sin((pi*Theta)/180))/(Mi^2*Pn(1)^2*cos((pi*Theta)/180)^2 + Ma^2*Pn(2)^2*cos((pi*Theta)/180)^2 + Mi^2*Pn(2)^2*sin((pi*Theta)/180)^2 + Ma^2*Pn(1)^2*sin((pi*Theta)/180)^2 - 2*Mi^2*Pn(1)*Pn(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*Ma^2*Pn(1)*Pn(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180));
 

%% Undistort 3D point

X = -(2*Pn(2)*max(c)*min(c))/(fy*max(c) - fy*min(c));
Y = (2*max(c)*min(c)*(Pn(1)*fy + Pn(2)*s))/(fx*fy*(max(c) - min(c)));
Z = -(max(c) + min(c))/(max(c) - min(c));

%% Height 
Distance(i)=r/norm([X Y Z]);

%% 3D point projection to camera 2D plane
P3d(i,:)=r*[X Y Z]/norm([X Y Z]);
t =  ( d - N(1)*P3d(i,1) - N(2)*P3d(i,2) - N(3)*P3d(i,3) ) / ( N(1)*N(1) + N(2)*N(2) + N(3)*N(3) );  
rot = [cosd(90) -sind(90); sind(90) cosd(90)]; 
P2d(i,:)=[rot*[ N(1)*t+P3d(i,1) N(2)*t+P3d(i,2)]']';

end % End for loop i

P2d=double(P2d);
Distance=mean(Distance);

%% Undistort Mi

X1new= (0)*cosd(-Theta)-(1)*sind(-Theta);
Y1new= (0)*sind(-Theta)+(1)*cosd(-Theta);
Pn=Mi*[X1new Y1new]+[k h];
Pn=[Pn(1) Pn(2)]/norm([Pn(1) Pn(2)]);
c(1)=(Mi*Ma*(Mi^2*Pn(1)^2*cos((pi*Theta)/180)^2 + Ma^2*Pn(2)^2*cos((pi*Theta)/180)^2 + Mi^2*Pn(2)^2*sin((pi*Theta)/180)^2 + Ma^2*Pn(1)^2*sin((pi*Theta)/180)^2 - Pn(1)^2*h^2*cos((pi*Theta)/180)^4 - Pn(2)^2*k^2*cos((pi*Theta)/180)^4 - Pn(1)^2*h^2*sin((pi*Theta)/180)^4 - Pn(2)^2*k^2*sin((pi*Theta)/180)^4 - 2*Pn(1)^2*h^2*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2 - 2*Pn(2)^2*k^2*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2 - 2*Mi^2*Pn(1)*Pn(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*Ma^2*Pn(1)*Pn(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*Pn(1)*Pn(2)*h*k*cos((pi*Theta)/180)^4 + 2*Pn(1)*Pn(2)*h*k*sin((pi*Theta)/180)^4 + 4*Pn(1)*Pn(2)*h*k*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2)^(1/2) + Ma^2*Pn(2)*h*cos((pi*Theta)/180)^2 + Mi^2*Pn(1)*k*cos((pi*Theta)/180)^2 + Mi^2*Pn(2)*h*sin((pi*Theta)/180)^2 + Ma^2*Pn(1)*k*sin((pi*Theta)/180)^2 - Mi^2*Pn(1)*h*cos((pi*Theta)/180)*sin((pi*Theta)/180) + Ma^2*Pn(1)*h*cos((pi*Theta)/180)*sin((pi*Theta)/180) - Mi^2*Pn(2)*k*cos((pi*Theta)/180)*sin((pi*Theta)/180) + Ma^2*Pn(2)*k*cos((pi*Theta)/180)*sin((pi*Theta)/180))/(Mi^2*Pn(1)^2*cos((pi*Theta)/180)^2 + Ma^2*Pn(2)^2*cos((pi*Theta)/180)^2 + Mi^2*Pn(2)^2*sin((pi*Theta)/180)^2 + Ma^2*Pn(1)^2*sin((pi*Theta)/180)^2 - 2*Mi^2*Pn(1)*Pn(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*Ma^2*Pn(1)*Pn(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180));
c(2)=(Ma^2*Pn(2)*h*cos((pi*Theta)/180)^2 - Mi*Ma*(Mi^2*Pn(1)^2*cos((pi*Theta)/180)^2 + Ma^2*Pn(2)^2*cos((pi*Theta)/180)^2 + Mi^2*Pn(2)^2*sin((pi*Theta)/180)^2 + Ma^2*Pn(1)^2*sin((pi*Theta)/180)^2 - Pn(1)^2*h^2*cos((pi*Theta)/180)^4 - Pn(2)^2*k^2*cos((pi*Theta)/180)^4 - Pn(1)^2*h^2*sin((pi*Theta)/180)^4 - Pn(2)^2*k^2*sin((pi*Theta)/180)^4 - 2*Pn(1)^2*h^2*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2 - 2*Pn(2)^2*k^2*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2 - 2*Mi^2*Pn(1)*Pn(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*Ma^2*Pn(1)*Pn(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*Pn(1)*Pn(2)*h*k*cos((pi*Theta)/180)^4 + 2*Pn(1)*Pn(2)*h*k*sin((pi*Theta)/180)^4 + 4*Pn(1)*Pn(2)*h*k*cos((pi*Theta)/180)^2*sin((pi*Theta)/180)^2)^(1/2) + Mi^2*Pn(1)*k*cos((pi*Theta)/180)^2 + Mi^2*Pn(2)*h*sin((pi*Theta)/180)^2 + Ma^2*Pn(1)*k*sin((pi*Theta)/180)^2 - Mi^2*Pn(1)*h*cos((pi*Theta)/180)*sin((pi*Theta)/180) + Ma^2*Pn(1)*h*cos((pi*Theta)/180)*sin((pi*Theta)/180) - Mi^2*Pn(2)*k*cos((pi*Theta)/180)*sin((pi*Theta)/180) + Ma^2*Pn(2)*k*cos((pi*Theta)/180)*sin((pi*Theta)/180))/(Mi^2*Pn(1)^2*cos((pi*Theta)/180)^2 + Ma^2*Pn(2)^2*cos((pi*Theta)/180)^2 + Mi^2*Pn(2)^2*sin((pi*Theta)/180)^2 + Ma^2*Pn(1)^2*sin((pi*Theta)/180)^2 - 2*Mi^2*Pn(1)*Pn(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180) + 2*Ma^2*Pn(1)*Pn(2)*cos((pi*Theta)/180)*sin((pi*Theta)/180));
X = -(2*Pn(2)*max(c)*min(c))/(fy*max(c) - fy*min(c));
Y = (2*max(c)*min(c)*(Pn(1)*fy + Pn(2)*s))/(fx*fy*(max(c) - min(c)));
Z = -(max(c) + min(c))/(max(c) - min(c));
Pmi3d=r*[X Y Z]/norm([X Y Z]);
t =  ( d - N(1)*Pmi3d(1) - N(2)*Pmi3d(2) - N(3)*Pmi3d(3) ) / ( N(1)*N(1) + N(2)*N(2) + N(3)*N(3) );  
rot = [cosd(90) -sind(90); sind(90) cosd(90)]; 
Pmi2D=[rot*[ N(1)*t+Pmi3d(1) N(2)*t+Pmi3d(2)]']';
Mi=norm(Pmi2D);


%% Compute Alpha, Beta and Gamma

Pma3D=[ r*sind(Theta)  r*cosd(Theta) 0];
Pbow=double([ P3d(1,1) P3d(1,2) P3d(1,3) ]);

Alpha=sign(Theta)*(90-abs(Theta) ) -90*(sign(sign(Theta)*(90-abs(Theta) ))+ sign(c(1)+c(2)));
Beta=acosd(Mi/r);
Gamma = sign(P3d(1,3))*sign(c(1)+c(2))*atan2d(norm(cross(Pma3D,Pbow)), dot(Pma3D,Pbow))+90*(1+sign(c(1)+c(2)))-180*(sign(P3d(1,3))+1)*sign(1+sign(c(1)+c(2)));


%% Circle eccentricity

% e=sqrt(1-Mi^2/r^2);

%% OUTPUTS

% Degrees = [ round(Alpha,15) round(Beta,15) round(Gamma,15)];
Degrees = [ Alpha Beta Gamma];
Distance = Distance;

end




function [semimajor_axis, semiminor_axis, x0, y0, Theta , coeff ,test] = ellipse_fit(x, y)

x = x(:);
y = y(:);

M = [2*x.*y y.^2 2*x 2*y ones(size(x))];
coeff = M\(-x.^2);

%Extract parameters from vector e
a = 1;
b = coeff(1);
c = coeff(2);
d = coeff(3);
e = coeff(4);
f = coeff(5);

delta = b^2-a*c;

%Offset
x0 = (c*d - b*e)/delta;
y0 = (a*e - b*d)/delta;

 %Semi axis
nom = 2 * (a*e^2 + c*d^2 + f*b^2 - 2*b*d*e - a*c*f);
s = sqrt(1 + (4*b^2)/(a-c)^2);
a_prime = sqrt(nom/(delta*( (c-a)*s -(c+a))));
b_prime = sqrt(nom/(delta*( (a-c)*s -(c+a))));
semimajor_axis = a_prime;
semiminor_axis = b_prime;


%% Compute Theta
Theta = 0.5 * acotd((c-a)/(2*b));
if (a_prime < b_prime)
    Theta = 90 + Theta;
end

if b < 0 & Theta>90
Theta=-180+Theta;
end

limit=.01;
if b > -limit & b < limit & d > -limit & d < limit
Theta=0;
end


end