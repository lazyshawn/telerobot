function [faces, vertices, colors, p0, p1, p2, p3] = animation_ini(A0, R0, q_deg)
stlfilePath='./stl';
% rangle of plot
Xmin = -1000.0;
Xmax =  4000.0;
Ymin = -1500.0;
Ymax =  1500.0;
Zmin =  -500.0;
Zmax =  500.0;

% load the .stl files
faces=cell(1,4);
vertices=cell(1,4);
colors=cell(1,4);

% set the figure size 
set(gcf,'Position',[60 70  1280 720]);

light                               % add a default light
daspect([1 1 1])                    % Setting the aspect ratio
view(-90,90)                             % Isometric view

xlabel('X'),ylabel('Y'),zlabel('Z')
axis([Xmin Xmax Ymin Ymax Zmin Zmax]); % setting of the range of graph
grid on;

% read the stl file
for i=1:4
disp(['reading ' sprintf('link%d.stl',i-1)])
[F,V,C]=rndread(fullfile(stlfilePath, sprintf('link%d.stl', i-1)));
disp(['done'])
faces{i}=F;
vertices{i}=V;
colors{i}=C;
end

% plot the base
Vb = vertices{1}';
Vb = [Vb(1,:); Vb(2,:); Vb(3,:)];
pp_pos=[R0(1)*1000;R0(2)*1000;R0(3)*1000];% 
rotMb=A0;
nvb=rotMb * Vb;    
nvb(1,:)=nvb(1,:)+pp_pos(1); 
nvb(2,:)=nvb(2,:)+pp_pos(2); 
nvb(3,:)=nvb(3,:)+pp_pos(3)-500; 
p0 = patch('faces',faces{1}, 'vertices' ,nvb(1:3,:)');
 set(p0, 'facec', 'flat');            % Set the face color flat
 set(p0, 'FaceVertexCData', colors{1});       % Set the color (from file)
 set(p0, 'EdgeColor','none');    

% plot link1
link1_base=pp_pos+A0*[500;0;0;];
link1_ori=A0*cz(q_deg(1)/180*pi);
vlink1 = vertices{2}';
vlink1 = [vlink1(1,:); vlink1(2,:); vlink1(3,:)];
nv1=link1_ori*cy(-90/180*pi)*vlink1;

nv1(1,:)=nv1(1,:)+link1_base(1); 
nv1(2,:)=nv1(2,:)+link1_base(2); 
nv1(3,:)=nv1(3,:)+link1_base(3);
p1 = patch('faces',faces{2}, 'vertices' ,nv1(1:3,:)');
 set(p1, 'facec', 'flat');            % Set the face color flat
 set(p1, 'FaceVertexCData', colors{2});       % Set the color (from file)
 set(p1, 'EdgeColor','none');  
 
%plot link2
link2_base=link1_base+link1_ori*[1000;0;0;];
link2_ori=link1_ori*cz(q_deg(2)/180*pi);
vlink2 = vertices{3}';
vlink2 = [vlink2(1,:); vlink2(2,:); vlink2(3,:)];
nv2=link2_ori*cy(-90*pi/180)*vlink2;
nv2(1,:)=nv2(1,:)+link2_base(1); 
nv2(2,:)=nv2(2,:)+link2_base(2); 
nv2(3,:)=nv2(3,:)+link2_base(3);
p2 = patch('faces',faces{3}, 'vertices' ,nv2(1:3,:)');
set(p2, 'facec', 'flat');            % Set the face color flat
set(p2, 'FaceVertexCData', colors{3});       % Set the color (from file)
set(p2, 'EdgeColor','none');    

%plot link 3
link3_base=link2_base+link2_ori*[1000;0;0;];
link3_ori=link2_ori*cz(q_deg(3)*pi/180);
vlink3 = vertices{4}';
vlink3 = [vlink3(1,:); vlink3(2,:); vlink3(3,:)];
nv3=link3_ori*cy(-90/180*pi)*vlink3;
nv3(1,:)=nv3(1,:)+link3_base(1); 
nv3(2,:)=nv3(2,:)+link3_base(2); 
nv3(3,:)=nv3(3,:)+link3_base(3);
p3 = patch('faces',faces{4}, 'vertices' ,nv3(1:3,:)');
set(p3, 'facec', 'flat');            % Set the face color flat
set(p3, 'FaceVertexCData', colors{4});       % Set the color (from file)
set(p3, 'EdgeColor','none');    
% 

end



