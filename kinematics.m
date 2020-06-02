function p_=kinematics(p,w,v,delta_t)
%p=[x,y,theta]:当前时刻小车位姿状态(x,y:世界坐标系下坐标;theta:小车朝向与x轴正方向夹角)
%w:旋转角速度,逆时针为正
%v:
%w_l,w_r:左右轮的角速度
%delta_t:更新时间步长
%r/d:小车轮半径/两轮间距
%p_:下一时刻新状态
theta=p(3);
%R=w_r*d/(w_l-w_r);
%d_theta=((w_l-w_r)*r/d)*delta_t;
d_theta=w*delta_t;
d_x=v*cos(theta)*delta_t;
d_y=v*sin(theta)*delta_t;
theta=theta+d_theta;
p(3)=theta;
p(1)=p(1)+d_x;
p(2)=p(2)+d_y;
p_=p;
end


