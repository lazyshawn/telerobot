%{ 
E_LoopDeLoop.m

Combine translation and rotation (C and D) to make the plane do loops.

Matt Sheen, mws262@cornell.edu
%}

close all; clear all;
fig = figure;

%% Same code as B again:
%Compound Patch - columns are individual patches
pX = [-1 1 0; 
    0 0 0]';
pY = [-1/3 -1/3 2/3;
    -1/3 -1/3 2/3]';
pZ = [0 0 0;
    0 0.5 0]';

p1 = patch(pX,pY,pZ,'red');
p1.FaceAlpha = 0.5;

axis(2*[-2 2 -2 2 -1 3])
view(3)


%% Now do loops
%This will probably not match coordinate systems you're using, etc. This
%just shows how you can do rotation and translation together
OrigVerts = p1.Vertices; %Keep track of the original vertices

vel = 0.1*[0 1 0]; %Velocity vector -- in this case, we're saying that the plane always goes in the direction of its nose
rot = angle2dcm(0,0,-0.02);

dt = 0.5;
center = [0 0 0]; %Keep track of the current center position of the plane.=
totalRot = eye(3,3); %Keep track of the total rotation of the plane.

for i = 1:1000
    %Come up with a new center and rotation of the plane:
    totalRot = totalRot*rot; %Increase plane's rotation
    center = dt*(totalRot*vel')' + center; %Transform the velocity and increment the center position of the plane.
    
    %Now actually change the visuals:
    p1.Vertices = (totalRot*OrigVerts')' + repmat(center,[size(OrigVerts,1),1]); %Rotate the patch object about its center and add new center point.
   
    pause(0.01); 
end