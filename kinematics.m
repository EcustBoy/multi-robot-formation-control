function p_=kinematics(p,w,v,delta_t)
%p=[x,y,theta]:��ǰʱ��С��λ��״̬(x,y:��������ϵ������;theta:С��������x��������н�)
%w:��ת���ٶ�,��ʱ��Ϊ��
%v:
%w_l,w_r:�����ֵĽ��ٶ�
%delta_t:����ʱ�䲽��
%r/d:С���ְ뾶/���ּ��
%p_:��һʱ����״̬
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


