% Make Rapidly-Exploring Random Tree (RRT) planning algorithm 
% in conjunction with the Kalman s.Filter (KF) for localization in the 2D plane
% 100 instances of the path planning
% State = (xdot ydot x y).

% This code was used to generate Figure 1 of "the shortest path obtained",
%  Figure 2 of "the path of minimum uncertainty at the terminal state"
%  Figure 3 of "the path of maximum uncertainty at the terminal state"
% Deng Xiong, 2016.


clear all
clc
close all

% set up intial parameters
% ex:value along x axis of path point 
% ey;value along y axis of path point 
% x:flag
x=[];
ex=[];
ey=[];
x(1)=5; 
ey(1)=50; 
ex(1)=5;
n=0;

% map the obt and goal region
generate_obt


for io=1:100
    %   determine if clollision in segment.
    %   determine if collision between point & obt
    %     vpx1: value along x of nearest point 
    %     vpy1: value along x of nearest point 
    %     npx: value along x of new point
    %     npy: value along y of new point
    %     cs:collision in segment
% cp:collision in point
% dist: distance
% ndist: normal distance
% oo: value of the angle between two point
% gt: the number of target
% tpx: value along x of the target point
% tpy: value along y of the target point
% etpx: exact value along x of the target point
% etpy: exact value along y of the target point
% mn: min number
% ml: min length


    for i=1:1000000
%    initial x,y
      x(i+1)=0;
      ey(i+1)=0;
%     initial collision
       cs=1;
       cp=1;
   
       %  random points
       while(cs == 1||cp == 1)

        vpx2=randi([0 100],1);
        vpy2=randi([0 100],1);
        ndist=1000000;
       
        for j=1:10000000
            if x(j)==0
                break 
            end
            if ex(j)>90
                continue 
            end
            point_x_1=mod(x(j),100);
            point_y_1=mod(ey(j),100);
            dist=((vpx2-point_x_1)^2+(vpy2-point_y_1)^2);
            if dist<ndist
               ndist=dist; 
                z=j;
            end
        end
                 vpx1=ex(z);
                   vpy1=ey(z);
                     cs=collision_check_segment(vpx1,vpy1,vpx2,vpy2,obstacles); 
       
                     
 % find collision in line
  oo=atan((vpy1-vpy2)/(vpx1-vpx2)); 
        if vpx2>=vpx1 && vpy2>=vpy1
           npx=vpx1+2*cos(oo);
           npy=vpy1+2*sin(oo);
        end
        if vpx2<vpx1 && vpy2>vpy1
           npx=vpx1-2*cos(oo);
           npy=vpy1-2*sin(oo);
        end
        if vpx2<=vpx1 && vpy2<=vpy1
               npx=vpx1-2*cos(oo);
               npy=vpy1-2*sin(oo);
        end
        if vpx2>=vpx1 && vpy2<=vpy1
               npx=vpx1+2*cos(oo);
               npy=vpy1+2*sin(oo);
        end
            cp=collision_check_point(npx,npy,obstacles);
    end
    oo1(i)=oo;
    x(i+1)=z*100+npx; 
    ey(i+1)=npy; 
    ex(i+1)=npx; 
    k=[ex(z) ex(i+1)];
    l=[ey(z) ey(i+1)];
    
%     check if robot reaches the goal region
    if ex(i+1)>90
        break 
    end
    end
    d=x(i+1);
    gt=1;
    while(d~=5)
        gt=gt+1;
        d0=(d-mod(d,100))/100;
        d=x(d0); 
    end
    
    
    % generate the length of each path
    gt1(io)=gt;
    d=x(i+1);
    d2=ex(i+1);
    d3=ey(i+1);
    while(d ~=5 )
        tpx(io,gt)=d2;
        tpy(io,gt)=d3;
        gt=gt-1;
        d0=(d-mod(d,100))/100;
        d=x(d0);
        d2=ex(d0);
        d3=ey(d0);
    end
    % path point 
    tpx(io,1)=5;
    tpy(io,1)=50;
end
[ml,mn]=min(gt1);


% plot the shortest path
figure(1)
for c=1:100
etpx(c)=tpx(mn,c);
etpy(c)=tpy(mn,c);
if tpx(mn,c)>90
    break
end
end
figure(1)
generate_obt
hold on
plot(etpx,etpy);


% MATRIX VRIABLES:
%
% s.A= state transition matrix (defaults to identity).
% s.P = covariance of the state vector estimate. 
% s.B = input matrix, optional (defaults to zero).
% s.F = process noise covariance.
% s.z= measurement noise covariance (required).
% s.H = observation matrix (defaults to identity).
% xv:initial vector.
% ip:initial covariance.
% sx: flag to find if the robot could senor the x-direction obstacles
% sy: flag to find if the robot could senor the y-direction obstacles
% xs: speed of x
% ys: speed of y
% ipx: uncertainty along x
% ipy: uncertainty along y
% uta: uncertainty target
% mu: max uncertainty
% miu: min uncertainty

s.F=[1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

s.A=[1 0 0 0;
    0 1 0 0;
    1 0 1 0;
    0 1 0 1]; 

for io=1:100
    xv=[0;0;5;50]; 
    ip=eye(4); 
    I=eye(4);

    for i=2:100
        sx=collision_check_segment(tpx(io,i)-5,tpy(io,i),tpx(io,i)+5,tpy(io,i),obstacles);
        sy=collision_check_segment(tpx(io,i),tpy(io,i)-5,tpx(io,i),tpy(io,i)+5,obstacles);
        xs=tpx(io,i)-tpx(io,i-1);
        ys=tpy(io,i)-tpy(io,i-1);
     
        
        if sx==0&&sy==0
            s.z=[xs;ys];
            
            s.H=[1 0 0 0;
                0 1 0 0];
            s.R=eye(2);
        elseif sx==1&&sy==0
            s.z=[xs;ys;tpx(io,i)];
            
            s.H=[1 0 0 0;
                0 1 0 0;
                0 0 1 0];
            s.R=eye(3);
        elseif sx==0&&sy==1
            s.z=[xs;ys;tpy(io,i)];
            
            s.H=[1 0 0 0;
                0 1 0 0;
                0 0 1 0];
            s.R=eye(3);
        else 
            s.z=[xs;ys;tpx(io,i);tpy(io,i)];
            
            s.H=[1 0 0 0;
                0 1 0 0;
                0 0 1 0;
                0 0 0 1];
            s.R=eye(4);
        end
     % kalman filter

% State Prediction
% (Predict where we're gonna be)
       xb=s.A*xv;
%  Covariance Prediction
% (Predict how much error)  
       ib=s.A*ip*s.A'+s.F;
% Kalman Gain
% (Moderate the prediction)
       kg=ib*s.H'*(s.H*ib*s.H'+s.R)^-1;
%  State Update
% (New estimate of where we are) 
        xv=xb+kg*(s.z-s.H*xb); 
%  Covariance Update
% (New estimate of error)
        ip=(I-kg*s.H)*ib;
        
        ipx(io,i)=ip(3,3); 
        ipy(io,i)=ip(4,4); 
        
        if tpx(io,i)>90
            break
        end
    end
    uta(io)=(ip(3,3))+(ip(4,4));
    
end
[mu,mu_number]=max(uta);
[miu,miu_number]=min(uta);

figure(2)
generate_obt
for o=1:100
    ra=ipy(mu_number,o)^0.5;
    rb=ipx(mu_number,o)^0.5;
    x0=tpx(mu_number,o);
    y0=tpy(mu_number,o);
    ellipse(ra,rb,0,x0,y0,'r')
    if tpx(mu_number,o)>90
        break
    end
end
hold on
for c=1:100
   etpxmu(c)=tpx(mu_number,c);
   etpymu(c)=tpy(mu_number,c);
    if tpx(mu_number,c) > 90
        break
    end
end
plot(etpxmu,etpymu);

figure(3)
generate_obt
for o=1:100
    ra=ipy(miu_number,o)^0.5;
    rb=ipx(miu_number,o)^0.5;
    x0=tpx(miu_number,o);
    y0=tpy(miu_number,o);
    ellipse(ra,rb,0,x0,y0,'r')
    if tpx(miu_number,o)==0
        break
    end
end
hold on

for c=1:100
    etpxmiu(c)=tpx(miu_number,c);
    etpymiu(c)=tpy(miu_number,c);
    if tpx(miu_number,c)>90
        break
    end
end
plot(etpxmiu,etpymiu); 
hold off