clear;
clc;

global r d radius delta_t total_t num back_delta_t;
width=3;height=3;
r=0.025;
d=0.1;
radius=((d/2).^2+r.^2).^(0.5);
delta_t=0.5;
total_t=2000;
num=2;
back_delta_t=0.1*delta_t;
P=zeros(num,3);
%P(:,1:2)=rand(num,2)*2;
%P(:,1:2)=[0.1,0.2;0.1,0.7;0.3,0.2;0.3,0.7;0.2,0.45];
P(:,1:2)=[0.1,0.2;0.3,0.45];
P(:,3)=rand(num,1)*3;
P(end,3)=0;
P_next=P;

f=figure(1);

grid on;
colordef black;
%axis([0 5 0 5]);
axis equal;
hold on;

Q1={};
Q2={};
H={};
link=zeros(num,num);
for id=1:num
    q=scatter(P(id,1),P(id,2),'.b');
    Q1{id}=q;
    if id==5
        q=fill(0,0,'g');
    else 
        q=fill(0,0,'r');
    end
    Q2{id}=q;
    h=animatedline('color','b');
    H{id}=h;
    addpoints(H{id},P(id,1),P(id,2))
end

desire_pose=[5,4*pi/3;];
             %5,2*pi/3;
             %2.5,4*pi/3;
             %2.5,2*pi/3];
k1=0.002;
k2=0.001;

V_l=0.05;
W_l=0;

v_agents=zeros(num,int8(total_t/delta_t));
w_agents=zeros(num,int8(total_t/delta_t));
l_agents=zeros(num-1,int8(total_t/delta_t));
phi_agents=zeros(num-1,int8(total_t/delta_t));

for t=0:delta_t:total_t
    if t~=0
        for id=1:num
            P(id,:)=kinematics(P(id,:),w_agents(id,int8(t/delta_t)),v_agents(id,int8(t/delta_t)),delta_t);
        end
    end
    figure(1);
    for id=1:num
        
        THETA=linspace(0,2*pi,100);
        RHO=ones(1,100)*radius;
        [X,Y] = pol2cart(THETA,RHO);
        X=X+P(id,1);
        Y=Y+P(id,2);
        %set(Q3{id},'Xdata',P(id,1),'Ydata',P(id,2));
        %set(Q4{id},'Xdata',P(id,1)+radius*sin(P(id,3)),'Ydata',P(id,2)+radius*cos(P(id,3)));
        set(Q2{id},'Xdata',X,'Ydata',Y);
        set(Q2{id},'facealpha',0.5);
        set(Q1{id},'Xdata',P(id,1),'Ydata',P(id,2));
        %h=H{id};
        addpoints(H{id},P(id,1),P(id,2));
        %drawnow;
    end
    drawnow;
    %leader controller
    w_agents(end,int8(t/delta_t+1))=W_l;
    v_agents(end,int8(t/delta_t+1))=V_l;
    %follower controller
    for id=1:num-1
        castor=[P(id,1)+radius*cos(P(id,3)),P(id,2)+radius*sin(P(id,3))];
        
        l=norm([castor(1)-P(end,1),castor(2)-P(end,2)]);
        vec1=[castor(1)-P(end,1),castor(2)-P(end,2)];
        vec1=vec1/norm(vec1);
        vec2=[radius*cos(P(id,3)),radius*sin(P(id,3))];
        vec2=vec2/norm(vec2);
        phi=acos(sum((vec1.*vec2)));
        vec3=[radius*cos(P(id,3)+pi/2),radius*sin(P(id,3)+pi/2)];
        vec3=vec3/norm(vec3);
        if acos(sum((vec1.*vec3)))<=pi/2
            phi=phi;
        else
            phi=2*pi-phi;
        end
        l_agents(id,int8(t/delta_t+1))=l;
        phi_agents(id,int8(t/delta_t+1))=phi;
        
        gamma=P(end,3)+phi-P(id,3);
        rho=(k1*(desire_pose(id,1)-l)+v_agents(end,int8(t/delta_t+1))*cos(phi))/cos(gamma);
        w_agents(id,int8(t/delta_t+1))=cos(gamma)*(k2*l*(desire_pose(id,2)-phi)-v_agents(end,int8(t/delta_t+1))*sin(phi)+l*w_agents(end,int8(t/delta_t+1))+rho*sin(gamma))/radius;
        v_agents(id,int8(t/delta_t+1))=rho-radius*w_agents(id,int8(t/delta_t+1))*tan(gamma);
    end 
end
%%
g=figure(2);
colordef white;
grid on;
for i=1:num
    subplot(221)
    plot([1:int8(total_t/delta_t)],v_agents(i,:));
    hold on;
end
axis([0 150 0 0.06])
legend('1','2')
title('linear velocity')
for i=1:num
    subplot(222);
    plot([1:int8(total_t/delta_t)],w_agents(i,:));
    hold on;
end
legend('1','2')
title('angular velocity')
for i=1:num-1
    subplot(223)
    plot([1:int8(total_t/delta_t)],l_agents(i,:));
    hold on;
end
legend('1')
title('distance')
for i=1:num-1
    subplot(224)
    plot([1:int8(total_t/delta_t)],phi_agents(i,:));
    hold on;
end
legend('1')
title('offset angular')
