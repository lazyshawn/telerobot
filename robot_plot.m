function [ output] = robot_plot( faces, vertices, colors, A0, AA,R0, q_deg, p0,p1,p2,p3)

% plot the base
Vb = vertices{1}';
Vb = [Vb(1,:); Vb(2,:); Vb(3,:)];
pp_pos=[R0(1)*1000;R0(2)*1000;R0(3)*1000];% 

rotMb=A0;
nvb=rotMb * Vb;    
nvb(1,:)=nvb(1,:)+pp_pos(1); 
nvb(2,:)=nvb(2,:)+pp_pos(2); 
nvb(3,:)=nvb(3,:)+pp_pos(3)-500; 
set(p0,'Vertices',nvb(1:3,:)');    
%   pp_pos
%   pause

% plot the link1
link1_base=pp_pos+A0*[500;0;0;];
%link1_ori=A0*rotz(q_deg(1));
link1_ori=AA(1:3,1:3);
vlink1 = vertices{2}';
vlink1 = [vlink1(1,:); vlink1(2,:); vlink1(3,:)];
nv1=link1_ori*cy(-90*pi/180)*vlink1;
nv1(1,:)=nv1(1,:)+link1_base(1); 
nv1(2,:)=nv1(2,:)+link1_base(2); 
nv1(3,:)=nv1(3,:)+link1_base(3);
set(p1,'Vertices',nv1(1:3,:)');  

% plot the link 2
link2_base=link1_base+link1_ori*[1000;0;0;];
%link2_ori=link1_ori*rotz(q_deg(2));
link2_ori=AA(1:3,4:6);
vlink2 = vertices{3}';
vlink2 = [vlink2(1,:); vlink2(2,:); vlink2(3,:)];
nv2=link2_ori*cy(-90/180*pi)*vlink2;
nv2(1,:)=nv2(1,:)+link2_base(1); 
nv2(2,:)=nv2(2,:)+link2_base(2); 
nv2(3,:)=nv2(3,:)+link2_base(3);
set(p2,'Vertices',nv2(1:3,:)');  

% plot the link 3
link3_base=link2_base+link2_ori*[1000;0;0;];
%link2_ori=link1_ori*rotz(q_deg(2));
link3_ori=AA(1:3,7:9);
vlink3 = vertices{4}';
vlink3 = [vlink3(1,:); vlink3(2,:); vlink3(3,:)];
nv3=link3_ori*cy(-90/180*pi)*vlink3;
nv3(1,:)=nv3(1,:)+link3_base(1); 
nv3(2,:)=nv3(2,:)+link3_base(2); 
nv3(3,:)=nv3(3,:)+link3_base(3);
set(p3,'Vertices',nv3(1:3,:)'); 
drawnow;

end

