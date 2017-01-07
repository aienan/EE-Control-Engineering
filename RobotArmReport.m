%
%  Robot Arm with PID controller simulation.
% 
 
L1 = 100; L2 = 100; L3 = 80;        % Length of arms
delta=0.1;      % Time duration
period = 70 ;     % Time Length
I1=1; c1=1;  I2 = 1; c2 = 1;  I3 = 1; c3 = 1;       % wheel motor parameters 
 
Zc = 0.001;       % We set a PID zero as Zc.
kd1 = 1 ;  kp1 = kd1 * (c1 / I1 + Zc);  ki1 = kd1 * c1 * Zc / I1;     % PID parameters for L1.   
kd2 = 1 ;  kp2 = kd2 * (c2 / I2 + Zc);  ki2 = kd2 * c2 * Zc / I2;     % PID parameters for L2.
kd3 = 1 ;  kp3 = kd3 * (c3 / I3 + Zc);  ki3 = kd3 * c3 * Zc / I3;     % PID parameters for L3. 
 
%Z1 = 10;  Z2 = 50; 
%kd1 = 40 ;  kp1 = kd1 * (Z1+Z2);  ki1 = kd1 * Z1*Z2;     % PID parameters for L1.   
%kd2 = 40 ;  kp2 = kd2 * (Z1+Z2);  ki2 = kd2 * Z1*Z2;     % PID parameters for L2.
%kd3 = 40 ;  kp3 = kd3 * (Z1+Z2);  ki3 = kd3 * Z1*Z2;     % PID parameters for L3.
 
%kd1 = 1; kp1 = 1.001; ki1 = 0.001;
%kd2 = 1; kp2 = 1.001; ki2 = 0.001;
%kd3 = 1; kp3 = 1.001; ki3 = 0.001;
 
for k=1:10,         % Give the initial position of Objects.
    ObX(k) =rand*200-100; ObY(k)=rand*200 - 100;
    ObZ(k)=5;
end       
 
for i=1:10,     % Calculate the desired value of theta of the objects.
    xd = ObX(i); yd = ObY(i); zd = ObZ(i);
    d = sqrt(xd^2+yd^2+(zd-L1)^2);
    D = (d^2-L2^2-L3^2)/(2*L2*L3);
    E = (L2^2+d^2-L3^2)/(2*L2*d);
    beta = atan2(sqrt(1-E^2),E);
    alpha = atan2(zd-L1,sqrt(xd^2+yd^2));
    Obth3(i) = atan2(-sqrt(1-D^2),D);
    Obth2(i) = alpha+beta;
    Obth1(i) = atan2(yd,xd);
end
 
for i=1:10,     % Calculate the desired value of theta of the stack.
    xd = 100; yd = 0; zd = 10*i - 5;
    d = sqrt(xd^2+yd^2+(zd-L1)^2);
    D = (d^2-L2^2-L3^2)/(2*L2*L3);
    E = (L2^2+d^2-L3^2)/(2*L2*d);
    beta = atan2(sqrt(1-E^2),E);
    alpha = atan2(zd-L1,sqrt(xd^2+yd^2));
    Stth3(i) = atan2(-sqrt(1-D^2),D);
    Stth2(i) = alpha+beta;
    Stth1(i) = atan2(yd,xd);
end
 
for i=1:10,     % Calculate the desired value of theta of the midway, which goes from object to stack.
    xd = (ObX(i)+100)/2; yd = ObY(i)/2; zd = 120;
    d = sqrt(xd^2+yd^2+(zd-L1)^2);
    D = (d^2-L2^2-L3^2)/(2*L2*L3);
    E = (L2^2+d^2-L3^2)/(2*L2*d);
    beta = atan2(sqrt(1-E^2),E);
    alpha = atan2(zd-L1,sqrt(xd^2+yd^2));
    MdOth3(i) = atan2(-sqrt(1-D^2),D);
    MdOth2(i) = alpha+beta;
    MdOth1(i) = atan2(yd,xd);
end
 
for i=1:10,     % Calculate the desired value of theta of the midway, which goes from stack to object.
    if(i ==10)  xd = 50; yd = 50; zd = 120;     % Final position.
    else  xd = (ObX(i+1)+100)/2; yd = ObY(i+1)/2; zd = 120;
    end
    d = sqrt(xd^2+yd^2+(zd-L1)^2);
    D = (d^2-L2^2-L3^2)/(2*L2*L3);
    E = (L2^2+d^2-L3^2)/(2*L2*d);
    beta = atan2(sqrt(1-E^2),E);
    alpha = atan2(zd-L1,sqrt(xd^2+yd^2));
    MdSth3(i) = atan2(-sqrt(1-D^2),D);
    MdSth2(i) = alpha+beta;
    MdSth1(i) = atan2(yd,xd);
end
 
    m1(1)=0;  m1(2)=0;      % initial setting for motor's angle
    m2(1)=0;  m2(2)=0;
    m3(1)=0;  m3(2)=0;
    m1prev = 0; m2prev = 0; m3prev = 0;
 
for i=1:10,     % i denotes the object's number.
    r1 = Obth1(i) * ones(1,period);     % reference input is step function
    r2 = Obth2(i) * ones(1,period);     % reference input is step function
    r3 = Obth3(i) * ones(1,period);     % reference input is step function
 
    e1(1)=0; e1(2)=0;       % Set the initial error
    e2(1)=0; e2(2)=0;       % Set the initial error
    e3(1)=0; e3(2)=0;       % Set the initial error
    
    t1(1)=0;  t1(2)=0;     % initial setting for torque
    t2(1)=0;  t2(2)=0;
    t3(1)=0;  t3(2)=0;
 
    m1(1)=m1prev;  m1(2)=m1prev;        % initial motor's angle position
    m2(1)=m2prev;  m2(2)=m2prev;
    m3(1)=m3prev;  m3(2)=m3prev;
 
    for n=3:period,         % Robot move toward object to pick it up.
        e1(n) = r1(n)-m1(n-1);
        t1(n)=t1(n-1)+kp1*(e1(n)-e1(n-1))+ki1*e1(n)*delta+kd1*(e1(n)-2*e1(n-1)+e1(n-2))/ delta; 
        m1(n)=( (2*I1+c1*delta)*m1(n-1) - I1*m1(n-2) + delta^2*t1(n) ) / ( I1 + c1*delta);       % Calculate motor1's angle position
        
        e2(n) = r2(n)-m2(n-1);
        t2(n)=t2(n-1)+kp2*(e2(n)-e2(n-1))+ki2*e2(n)*delta+kd2*(e2(n)-2*e2(n-1)+e2(n-2))/ delta; 
        m2(n)=( (2*I2+c2*delta)*m2(n-1) - I2*m2(n-2) + delta^2*t2(n) ) / ( I2 + c2*delta);       % Calculate motor2's angle position
        
        e3(n) = r3(n)-m3(n-1);
        t3(n)=t3(n-1)+kp3*(e3(n)-e3(n-1))+ki3*e3(n)*delta+kd3*(e3(n)-2*e3(n-1)+e3(n-2))/ delta; 
        m3(n)=( (2*I3+c3*delta)*m3(n-1) - I3*m3(n-2) + delta^2*t3(n) ) / ( I3 + c3*delta);       % Calculate motor3's angle position
 
        Lx3=cos(m1(n))*(L2*cos(m2(n))+L3*cos(m2(n)+m3(n)));     % Forward kinematics for drawing L3 arm.
        Ly3=sin(m1(n))*(L2*cos(m2(n))+L3*cos(m2(n)+m3(n)));
        Lz3=L1+L2*sin(m2(n))+L3*sin(m2(n)+m3(n)); 
        
        Lx2=cos(m1(n))*L2*cos(m2(n));       % Forward kinematics for drawing L2 arm.
        Ly2=sin(m1(n))*L2*cos(m2(n)); 
        Lz2=L1+L2*sin(m2(n)); 
    
        Lx1=0; Ly1=0; Lz1=L1;       % Forward kinematics for drawing L1 arm.
        Lx0=0; Ly0=0; Lz0=0;        % base position (0,0,0)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FIgure(4)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        figure(4);
        plot3([Lx0 Lx1], [Ly0 Ly1], [Lz0 Lz1], 'go-', 'LineWidth', 15);     % Draw L1 arm.
        hold on
        plot3([Lx1 Lx2], [Ly1 Ly2], [Lz1 Lz2], 'mo-', 'LineWidth', 10);     % Draw L2 arm.
        plot3([Lx2 Lx3], [Ly2 Ly3], [Lz2 Lz3], 'bo-','LineWidth', 5);       % Draw L3 arm.
        plot3(Lx3,Ly3,Lz3, 'rs-', 'MarkerSize', 15);    % Draw the robot's hand.
        for k=1:10, drawObject(ObX(k),ObY(k),ObZ(k)-5); end       % draw 10 cylindrical objects   
        xlabel('X'); ylabel('Y'); zlabel('Z');
        axis([-200 200 -200 200 -10 200])
        view(30,30);
        grid on
        pause(0.01); % pause makes small delay for the next drawing  
        hold off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FIgure(2)%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%        figure(2); hold on;     % Draw end effector's trajectories in the 3-dimensional space.
%        plot3(Lx3,Ly3,Lz3, 'rs-', 'MarkerSize', 15);    % Draw the robot's hand.
%        view(30,30);
%        hold off;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    
        m1prev = m1(n);  m2prev = m2(n);  m3prev = m3(n);
 
%%%%%%%%%%%%%%%% Figure(1)  Draw the graph of transient angular position in time course. %%%%%%%%%%%%%%%%%%%%%%%%  
%    figure(1);
%    plot((1:period)*delta - delta, r1);     % Plot the transient response graphs for angular position.
%    hold on
%    plot((1:period)*delta - delta, m1);
%    plot((1:period)*delta - delta, r2);
%    plot((1:period)*delta - delta, m2);
%    plot((1:period)*delta - delta, r3);
%    plot((1:period)*delta - delta, m3);
%    hold off
%    pause;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%            
        
    r1 = MdOth1(i) * ones(1,period);        % reference input is step function
    r2 = MdOth2(i) * ones(1,period);        % reference input is step function
    r3 = MdOth3(i) * ones(1,period);        % reference input is step function
    
    e1(1)=0; e1(2)=0;       % Set the initial error
    e2(1)=0; e2(2)=0;       % Set the initial error
    e3(1)=0; e3(2)=0;       % Set the initial error
    
    t1(1)=0;  t1(2)=0;     % initial setting for torque
    t2(1)=0;  t2(2)=0;
    t3(1)=0;  t3(2)=0;
 
    m1(1)=m1prev;  m1(2)=m1prev;        % initial motor's angle position
    m2(1)=m2prev;  m2(2)=m2prev;
    m3(1)=m3prev;  m3(2)=m3prev;    
    
    for n=3:period,         % Robot moves toward midway from obejct to stack.
        e1(n) = r1(n)-m1(n-1);
        t1(n)=t1(n-1)+kp1*(e1(n)-e1(n-1))+ki1*e1(n)*delta+kd1*(e1(n)-2*e1(n-1)+e1(n-2))/ delta; 
        m1(n)=( (2*I1+c1*delta)*m1(n-1) - I1*m1(n-2) + delta^2*t1(n) ) / ( I1 + c1*delta);       % Calculate motor1's angle position
        
        e2(n) = r2(n)-m2(n-1);
        t2(n)=t2(n-1)+kp2*(e2(n)-e2(n-1))+ki2*e2(n)*delta+kd2*(e2(n)-2*e2(n-1)+e2(n-2))/ delta; 
        m2(n)=( (2*I2+c2*delta)*m2(n-1) - I2*m2(n-2) + delta^2*t2(n) ) / ( I2 + c2*delta);       % Calculate motor2's angle position
        
        e3(n) = r3(n)-m3(n-1);
        t3(n)=t3(n-1)+kp3*(e3(n)-e3(n-1))+ki3*e3(n)*delta+kd3*(e3(n)-2*e3(n-1)+e3(n-2))/ delta; 
        m3(n)=( (2*I3+c3*delta)*m3(n-1) - I3*m3(n-2) + delta^2*t3(n) ) / ( I3 + c3*delta);       % Calculate motor3's angle position
 
        Lx3=cos(m1(n))*(L2*cos(m2(n))+L3*cos(m2(n)+m3(n)));     % Forward kinematics for drawing L3 arm.
        Ly3=sin(m1(n))*(L2*cos(m2(n))+L3*cos(m2(n)+m3(n)));
        Lz3=L1+L2*sin(m2(n))+L3*sin(m2(n)+m3(n)); 
        
        Lx2=cos(m1(n))*L2*cos(m2(n));       % Forward kinematics for drawing L2 arm.
        Ly2=sin(m1(n))*L2*cos(m2(n)); 
        Lz2=L1+L2*sin(m2(n)); 
    
        Lx1=0; Ly1=0; Lz1=L1;       % Forward kinematics for drawing L1 arm.
        Lx0=0; Ly0=0; Lz0=0;        % base position (0,0,0)
        
        ObX(i) = Lx3;  ObY(i) = Ly3;  ObZ(i) = Lz3;
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FIgure(4)%%%%%%%%%%%%%%%%%%%%%
        figure(4);
        plot3([Lx0 Lx1], [Ly0 Ly1], [Lz0 Lz1], 'go-', 'LineWidth', 15);     % Draw L1 arm.
        hold on
        plot3([Lx1 Lx2], [Ly1 Ly2], [Lz1 Lz2], 'mo-', 'LineWidth', 10);     % Draw L2 arm.
        plot3([Lx2 Lx3], [Ly2 Ly3], [Lz2 Lz3], 'bo-','LineWidth', 5);       % Draw L3 arm.
        plot3(Lx3,Ly3,Lz3, 'rs-','MarkerSize', 15);    % Draw the robot's hand.
        for k=1:10, drawObject(ObX(k),ObY(k),ObZ(k)-5); end       % draw 10 cylindrical objects   
        xlabel('X'); ylabel('Y'); zlabel('Z');
        axis([-200 200 -200 200 -10 200])
        view(30,30);
        grid on
        pause(0.01); % pause makes small delay for the next drawing  
        hold off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FIgure(2)%%%%%%%%%%%%%%%%%%%%%
%        figure(2); hold on;     % Draw end effector's trajectories in the 3-dimensional space.
%        plot3(Lx3,Ly3,Lz3, 'rs-', 'MarkerSize', 15);    % Draw the robot's hand.
%        view(30,30);
%        hold off;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    end
    
    m1prev = m1(n);  m2prev = m2(n);  m3prev = m3(n);
 
%%%%%%%%%%%%%%%% Figure(1)   Draw the graph of transient angular position in time course. %%%%%%%%%%%%%%%%%%%%%%%%  
%    figure(1);
%    plot((1:period)*delta - delta, r1);     % Plot the transient response graphs for angular position.
%    hold on
%    plot((1:period)*delta - delta, m1);
%    plot((1:period)*delta - delta, r2);
%    plot((1:period)*delta - delta, m2);
%    plot((1:period)*delta - delta, r3);
%    plot((1:period)*delta - delta, m3);
%    hold off
%    pause;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    
    r1 = Stth1(i) * ones(1,period);     % reference input is step function
    r2 = Stth2(i) * ones(1,period);     % reference input is step function
    r3 = Stth3(i) * ones(1,period);     % reference input is step function
    
    e1(1)=0; e1(2)=0;       % Set the initial error
    e2(1)=0; e2(2)=0;       % Set the initial error
    e3(1)=0; e3(2)=0;       % Set the initial error
    
    t1(1)=0;  t1(2)=0;     % initial setting for torque
    t2(1)=0;  t2(2)=0;
    t3(1)=0;  t3(2)=0;
 
    m1(1)=m1prev;  m1(2)=m1prev;        % initial motor's angle position
    m2(1)=m2prev;  m2(2)=m2prev;
    m3(1)=m3prev;  m3(2)=m3prev;    
    
    for n=3:period,         % Robot move toward stack to pile the objects.
        e1(n) = r1(n)-m1(n-1);
        t1(n)=t1(n-1)+kp1*(e1(n)-e1(n-1))+ki1*e1(n)*delta+kd1*(e1(n)-2*e1(n-1)+e1(n-2))/ delta; 
        m1(n)=( (2*I1+c1*delta)*m1(n-1) - I1*m1(n-2) + delta^2*t1(n) ) / ( I1 + c1*delta);       % Calculate motor1's angle position
        
        e2(n) = r2(n)-m2(n-1);
        t2(n)=t2(n-1)+kp2*(e2(n)-e2(n-1))+ki2*e2(n)*delta+kd2*(e2(n)-2*e2(n-1)+e2(n-2))/ delta; 
        m2(n)=( (2*I2+c2*delta)*m2(n-1) - I2*m2(n-2) + delta^2*t2(n) ) / ( I2 + c2*delta);       % Calculate motor2's angle position
        
        e3(n) = r3(n)-m3(n-1);
        t3(n)=t3(n-1)+kp3*(e3(n)-e3(n-1))+ki3*e3(n)*delta+kd3*(e3(n)-2*e3(n-1)+e3(n-2))/ delta; 
        m3(n)=( (2*I3+c3*delta)*m3(n-1) - I3*m3(n-2) + delta^2*t3(n) ) / ( I3 + c3*delta);       % Calculate motor3's angle position
 
        Lx3=cos(m1(n))*(L2*cos(m2(n))+L3*cos(m2(n)+m3(n)));     % Forward kinematics for drawing L3 arm.
        Ly3=sin(m1(n))*(L2*cos(m2(n))+L3*cos(m2(n)+m3(n)));
        Lz3=L1+L2*sin(m2(n))+L3*sin(m2(n)+m3(n)); 
        
        Lx2=cos(m1(n))*L2*cos(m2(n));       % Forward kinematics for drawing L2 arm.
        Ly2=sin(m1(n))*L2*cos(m2(n)); 
        Lz2=L1+L2*sin(m2(n)); 
    
        Lx1=0; Ly1=0; Lz1=L1;       % Forward kinematics for drawing L1 arm.
        Lx0=0; Ly0=0; Lz0=0;        % base position (0,0,0)
        
        ObX(i) = Lx3;  ObY(i) = Ly3;  ObZ(i) = Lz3;
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FIgure(4)%%%%%%%%%%%%%%%%%%%%%        
        figure(4);
        plot3([Lx0 Lx1], [Ly0 Ly1], [Lz0 Lz1], 'go-', 'LineWidth', 15);     % Draw L1 arm.
        hold on
        plot3([Lx1 Lx2], [Ly1 Ly2], [Lz1 Lz2], 'mo-', 'LineWidth', 10);     % Draw L2 arm.
        plot3([Lx2 Lx3], [Ly2 Ly3], [Lz2 Lz3], 'bo-','LineWidth', 5);       % Draw L3 arm.
        plot3(Lx3,Ly3,Lz3, 'rs-','MarkerSize', 15);    % Draw the robot's hand.
        for k=1:10, drawObject(ObX(k),ObY(k),ObZ(k)-5); end       % draw 10 cylindrical objects   
        xlabel('X'); ylabel('Y'); zlabel('Z');
        axis([-200 200 -200 200 -10 200])
        view(30,30);
        grid on
        pause(0.01); % pause makes small delay for the next drawing  
        hold off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FIgure(2)%%%%%%%%%%%%%%%%%%%%%
%        figure(2); hold on;     % Draw end effector's trajectories in the 3-dimensional space.
%        plot3(Lx3,Ly3,Lz3, 'rs-', 'MarkerSize', 15);    % Draw the robot's hand.
%        view(30,30);
%        hold off;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    end
    
    m1prev = m1(n);  m2prev = m2(n);  m3prev = m3(n);
 
%%%%%%%%%%%%%%%% Figure(1)   Draw the graph of transient angular position in time course. %%%%%%%%%%%%%%%%%%%%%%%%  
%    figure(1);
%    plot((1:period)*delta - delta, r1);     % Plot the transient response graphs for angular position.
%    hold on
%    plot((1:period)*delta - delta, m1);
%    plot((1:period)*delta - delta, r2);
%    plot((1:period)*delta - delta, m2);
%    plot((1:period)*delta - delta, r3);
%    plot((1:period)*delta - delta, m3);
%    hold off
%    pause;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    
    r1 = MdSth1(i) * ones(1,period);        % reference input is step function
    r2 = MdSth2(i) * ones(1,period);        % reference input is step function
    r3 = MdSth3(i) * ones(1,period);        % reference input is step function
    
    e1(1)=0; e1(2)=0;       % Set the initial error
    e2(1)=0; e2(2)=0;       % Set the initial error
    e3(1)=0; e3(2)=0;       % Set the initial error
    
    t1(1)=0;  t1(2)=0;     % initial setting for torque
    t2(1)=0;  t2(2)=0;
    t3(1)=0;  t3(2)=0;
 
    m1(1)=m1prev;  m1(2)=m1prev;        % initial motor's angle position
    m2(1)=m2prev;  m2(2)=m2prev;
    m3(1)=m3prev;  m3(2)=m3prev;    
    
    for n=3:period,         % Robot moves toward midway from stack to object.
        e1(n) = r1(n)-m1(n-1);
        t1(n)=t1(n-1)+kp1*(e1(n)-e1(n-1))+ki1*e1(n)*delta+kd1*(e1(n)-2*e1(n-1)+e1(n-2))/ delta; 
        m1(n)=( (2*I1+c1*delta)*m1(n-1) - I1*m1(n-2) + delta^2*t1(n) ) / ( I1 + c1*delta);       % Calculate motor1's angle position
        
        e2(n) = r2(n)-m2(n-1);
        t2(n)=t2(n-1)+kp2*(e2(n)-e2(n-1))+ki2*e2(n)*delta+kd2*(e2(n)-2*e2(n-1)+e2(n-2))/ delta; 
        m2(n)=( (2*I2+c2*delta)*m2(n-1) - I2*m2(n-2) + delta^2*t2(n) ) / ( I2 + c2*delta);       % Calculate motor2's angle position
        
        e3(n) = r3(n)-m3(n-1);
        t3(n)=t3(n-1)+kp3*(e3(n)-e3(n-1))+ki3*e3(n)*delta+kd3*(e3(n)-2*e3(n-1)+e3(n-2))/ delta; 
        m3(n)=( (2*I3+c3*delta)*m3(n-1) - I3*m3(n-2) + delta^2*t3(n) ) / ( I3 + c3*delta);       % Calculate motor3's angle position
 
        Lx3=cos(m1(n))*(L2*cos(m2(n))+L3*cos(m2(n)+m3(n)));     % Forward kinematics for drawing L3 arm.
        Ly3=sin(m1(n))*(L2*cos(m2(n))+L3*cos(m2(n)+m3(n)));
        Lz3=L1+L2*sin(m2(n))+L3*sin(m2(n)+m3(n)); 
        
        Lx2=cos(m1(n))*L2*cos(m2(n));       % Forward kinematics for drawing L2 arm.
        Ly2=sin(m1(n))*L2*cos(m2(n)); 
        Lz2=L1+L2*sin(m2(n)); 
    
        Lx1=0; Ly1=0; Lz1=L1;       % Forward kinematics for drawing L1 arm.
        Lx0=0; Ly0=0; Lz0=0;        % base position (0,0,0)
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FIgure(4)%%%%%%%%%%%%%%%%%%%%%        
        figure(4);        
        plot3([Lx0 Lx1], [Ly0 Ly1], [Lz0 Lz1], 'go-', 'LineWidth', 15);     % Draw L1 arm.
        hold on
        plot3([Lx1 Lx2], [Ly1 Ly2], [Lz1 Lz2], 'mo-', 'LineWidth', 10);     % Draw L2 arm.
        plot3([Lx2 Lx3], [Ly2 Ly3], [Lz2 Lz3], 'bo-','LineWidth', 5);       % Draw L3 arm.
        plot3(Lx3,Ly3,Lz3, 'rs-','MarkerSize', 15);    % Draw the robot's hand.
        for k=1:10, drawObject(ObX(k),ObY(k),ObZ(k)-5); end       % draw 10 cylindrical objects   
        
        xlabel('X'); ylabel('Y'); zlabel('Z');
        axis([-200 200 -200 200 -10 200])
        view(30,30);
        grid on
        pause(0.01); % pause makes small delay for the next drawing  
        hold off
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FIgure(2)%%%%%%%%%%%%%%%%%%%%%
%        figure(2); hold on;     % Draw end effector's trajectories in the 3-dimensional space.
%        plot3(Lx3,Ly3,Lz3, 'rs-', 'MarkerSize', 15);    % Draw the robot's hand.
%        view(30,30);
%        hold off;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%        
    end
    
    m1prev = m1(n);  m2prev = m2(n);  m3prev = m3(n);
 
%%%%%%%%%%%%%%%% Figure(1)   Draw the graph of transient angular position in time course. %%%%%%%%%%%%%%%%%%%%%%%%  
%    figure(1);
%    plot((1:period)*delta - delta, r1);     % Plot the transient response graphs for angular position.
%    hold on
%    plot((1:period)*delta - delta, m1);
%    plot((1:period)*delta - delta, r2);
%    plot((1:period)*delta - delta, m2);
%    plot((1:period)*delta - delta, r3);
%    plot((1:period)*delta - delta, m3);
%    hold off
%    pause;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
 
end

