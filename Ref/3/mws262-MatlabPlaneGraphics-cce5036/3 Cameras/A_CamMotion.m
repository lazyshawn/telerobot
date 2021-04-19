%{ 
A_CamMotion.m

Animate several camera motions around a surface.

Matt Sheen, mws262@cornell.edu
%}

fig = figure;

%% Surface we're looking at -- Matlab logo, same as used in folder 1 of
%examples

%Make the data for the surface plot
ground = 10000*membrane(1,40)-10000; %The numbers don't have to be large, this is just a relic of the past I'm too lazy to fix.

%Make the surface plot -- note that all the additional properties are NOT
%necessary, they just improve lighting and stuff.
groundSurf = surf(linspace(-10000,10000,size(ground,1)),linspace(-10000,10000,size(ground,2)),ground,'LineStyle','none','AmbientStrength',0.3,'DiffuseStrength',0.8,'SpecularStrength',0,'SpecularExponent',10,'SpecularColorReflectance',1);

%window position
fig.Position = [600 600 1000 700];
fig.Children.Projection = 'perspective';

axis([-10000 10000 -10000 10000 -10000 10000])

%% Initial camera properties.
%%%Note that all camera properties can also be altered with axis.Camera....
origPos = [10000,10000,0]';
origTarget = [0,0,-10000];

campos(origPos); %Set the camera position
camtarget(origTarget); %Set the camera target
camva(40); %Set the camera view angle. -- VERY IMPORTANT. By default, this number is way too high causing a fish-eye effect. Just reduce it until it looks about right.


%% Move that camera:

%pan around the scene - target stays the same, but we move the camera in a
%circle around the origin.
for i = 1:500
    campos(angle2dcm(0.01*i,0,0)*origPos);
    pause(0.01); 
end

%look up a bit - leave the camera position the same, but change the target
%upwards
for i = 1:500
    camtarget(origTarget+[0 0 20]*i);
    pause(0.01);  
end

%"fly" forward - Translate both the target and the position of the camera
%the same.
dir = camtarget - campos; %The direction we're going to move the camera.
for i = 1:500
    camtarget(camtarget + 0.01*dir);
    campos(campos + 0.01*dir)
    pause(0.01);  
end