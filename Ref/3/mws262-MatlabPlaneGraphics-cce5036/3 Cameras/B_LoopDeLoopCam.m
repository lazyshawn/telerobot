%{ 
B_LoopDeLoopCam.m

Animate several camera motions around a surface.

Matt Sheen, mws262@cornell.edu
%}

close all; clear all;
fig = figure;

%% Compound Patch - same from folder 2, example b
pX = [-1 1 0; 
    0 0 0]';
pY = [-1/3 -1/3 2/3;
    -1/3 -1/3 2/3]';
pZ = [0 0 0;
    0 0.5 0]';

p1 = patch(pX,pY,pZ,'red');
p1.FaceAlpha = 0.5;

OrigVerts = p1.Vertices;

%% Set initial camera position.
axis(2*[-10 10 -10 10 -10 10])
campos([8,8,8]);
camtarget([0,0,0]);
camva(40);

%Turning the axis grid on to help get perspective of movement
fig.Children.ZGrid = 'on';
fig.Children.YGrid = 'on';
fig.Children.XGrid = 'on';



vel = 0.1*[0 1 0];
rot = angle2dcm(0,0,-0.02);
dt = 0.5;
center = [0 0 0];

totalRot = eye(3,3);

%% Cam mode 1: fixed everything. Camera stays exactly the same throughout.
input('Press enter to see default camera orientation.');
for i = 1:1000
    %Moving the plane is the same as previous example
    totalRot = totalRot*rot;
    center = dt*(totalRot*vel')' + center; %transform the velocity, keep track of the center position.
    p1.Vertices = (totalRot*OrigVerts')' + repmat(center,[size(OrigVerts,1),1]); %Rotate the patch object about its center and add new center point.
   
    pause(0.01); 
end

%% Cam mode 2: let's follow the plane. Camera keeps the plane in the center of view, but does not change orientation.
input('Press enter to see plane-centering camera.');
for i = 1:1000
    %Moving the plane is the same as previous example
    totalRot = totalRot*rot;
    center = dt*(totalRot*vel')' + center; %transform the velocity, keep track of the center position.
    p1.Vertices = (totalRot*OrigVerts')' + repmat(center,[size(OrigVerts,1),1]); %Rotate the patch object about its center and add new center point.
   
    %CAMERA CHANGES:
    camtarget(center); %make the target be the center of the plane.
    campos(center+[5,5,5]); %make the camera position be a fixed offset from the center of the plane.
    pause(0.01); 
end

%% Cam mode 3: let's fly the plane. Camera also changes orientation with the plane.
input('Press enter to see plane-following camera.');
for i = 1:1000
    %Moving the plane is the same as previous example
    totalRot = totalRot*rot;
    center = dt*(totalRot*vel')' + center; %transform the velocity, keep track of the center position.
    p1.Vertices = (totalRot*OrigVerts')' + repmat(center,[size(OrigVerts,1),1]); %Rotate the patch object about its center and add new center point.
   
    %CAMERA CHANGES:
    camtarget(center); %Camera target is the center of the plane.
    campos(center+5*(totalRot*[0.5 -1 0.5]')'); %Camera position is a little behind, up, and to the right of the plane (helps see what's going on). Note that we have to transform the camera offset from plane frame to world frame.
    camup(totalRot*[0 0 1]'); % The camera should think "up" is the up vector of the plane. So we take the plane's up and transform it into the world frame.
    
    pause(0.01); 
end
