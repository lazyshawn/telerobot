function FinalEx
%{ 
FinalEx.m

WASDQE for steering. V to switch to 1st person view. More below.


Combine stuff from all the previous examples to make:
1. Import a plane as a patch object.
2. Translate/rotate this through space as per user input. User changes
heading, constant velocity dynamics. Use Ws,AD, QE for pitch,roll,yaw
3. Put in horizon and ground with textures. If the plane COM tries to
penetrate, simply project it back up.
4. Have camera follow the plane in 2 ways, press v to switch between.


NOTE: I wouldn't start here if you want to understand each step. Start with
the other examples and see how this is built.


Matt Sheen, mws262@cornell.edu
%}

close all

fig = figure;

fig.UserData.firstPerson = false; %do we start in 1st person view, or not?
resetPositionOnBounds = true; %keeps the plane in the region with the matlab logo ground.

hold on
fig.Position = [600 600 1500 1200];
%Disable axis viewing, don't allow axes to clip the plane, turn on
%perspective mode.
fig.Children.Visible = 'off';
fig.Children.Clipping = 'off';
fig.Children.Projection = 'perspective';

%% Add the sky - Fancy environment folder ex
skymap = [linspace(1,0.4,100)',linspace(1,0.4,100)',linspace(1,0.99,100)'];

[skyX,skyY,skyZ] = sphere(50);
sky = surf(500000*skyX,500000*skyY,500000*skyZ,'LineStyle','none','FaceColor','interp');
colormap(skymap);

%% Add the plane - Fancy planes folder
fv = stlread('test.stl');
p1 = patch(fv,'FaceColor',       'red', ...
         'EdgeColor',       'none',        ...
         'FaceLighting',    'gouraud',     ...
         'AmbientStrength', 0.15);
vert = p1.Vertices;

material('metal')

%% Add the ground + textures - Fancy environments folder Ex
texture = imread('texture.jpg');


ground = 10000*membrane(1,40)-10000;
groundSurf = surf(linspace(-10000,10000,size(ground,1)),linspace(-10000,10000,size(ground,2)),ground,'LineStyle','none','AmbientStrength',0.3,'DiffuseStrength',0.8,'SpecularStrength',0,'SpecularExponent',10,'SpecularColorReflectance',1);
groundSurf.FaceColor = 'texturemap';
groundSurf.CData = texture;

%add some extra flat ground going off to (basically) infinity.
flatground = surf(linspace(-500000,500000,size(ground,1)),linspace(-500000,500000,size(ground,2)),-10001*ones(size(ground)));
flatground.FaceColor = 'texturemap';
flatground.CData = texture;
flatground.AlphaData = 0.8;

camlight('headlight');

camva(40); %view angle

%% Set keyboard callbacks and flags for movement.
set(fig,'WindowKeyPressFcn',@KeyPress,'WindowKeyReleaseFcn', @KeyRelease);
        fig.UserData.e = false;
         fig.UserData.q = false;
         fig.UserData.a = false;
         fig.UserData.d = false;
         fig.UserData.w = false;
         fig.UserData.s = false;
         


forwardVec = [1 0 0]'; %Vector of the plane's forward direction in plane frame
rot = eye(3,3); %Initial plane rotation
pos = [-8000,8000,-2000]; %Initial plane position
vel = 1000; %Velocity

hold off
axis([-10000 10000 -10000 10000 -10000 10000])


%% Animation loop:
tic
told = 0;
while(ishandle(fig))
  tnew = toc;
  
  %Check for user inputs:
  if fig.UserData.e
      rot = rot*angle2dcm(0.05,0,0);
  end
  if fig.UserData.q
      rot = rot*angle2dcm(-0.05,0,0);
  end
  if fig.UserData.s
      rot = rot*angle2dcm(0,-0.05,0);
  end
  if fig.UserData.w
      rot = rot*angle2dcm(0,0.05,0);
  end
  if fig.UserData.a
      rot = rot*angle2dcm(0,0,-0.05);
  end
  if fig.UserData.d
      rot = rot*angle2dcm(0,0,0.05);
  end
  
  %Update plane's center position.
  pos = vel*(rot*forwardVec*(tnew-told))' + pos;
  
  %If the plane wants to go under the ground, then bring it back up to the
  %ground surface.
  nearestGroundZ = interp2(groundSurf.XData,groundSurf.YData,groundSurf.ZData,pos(1),pos(2));
  if pos(3)<nearestGroundZ
      pos(3) = nearestGroundZ;
  end
  
  if resetPositionOnBounds
      % If we leave the ground area in the X direction, then snap the plane
      % back to the other side.
      if pos(1)>max(groundSurf.XData)
          pos(1) = min(groundSurf.XData);
      elseif pos(1)<min(groundSurf.XData)
          pos(1) = max(groundSurf.XData);
      end

      % If we leave the ground area in the y direction, then snap the plane
      % back to the other side.
      if pos(2)>max(groundSurf.YData)
          pos(2) = min(groundSurf.YData);
      elseif pos(2)<min(groundSurf.YData)
          pos(2) = max(groundSurf.YData);
      end
  end
  
  %Update the plane's vertices using the new position and rotation
  p1.Vertices = (rot*vert')' + repmat(pos,[size(vert,1),1]);
  
  
  %Camera updates:
  if fig.UserData.firstPerson %First person view -- follow the plane from slightly behind.
      camupvec = rot*[0 0 1]';
      camup(camupvec);
      campos(pos' - 1000*rot*[1 0 -0.25]');
      camtarget(pos' + 100*rot*[1 0 0]');    
  else %Follow the plane from a fixed angle
    campos(pos + [-3000,3000,1000]);%3000*abs(pos-campos)/norm(pos-campos));
    camtarget(pos);

  end
  
     cam = campos;
    %Also keep the camera from going into the ground (could be done a
    %smarter way to also not look through the ground).
     nearestGroundZ = interp2(groundSurf.XData,groundSurf.YData,groundSurf.ZData,cam(1),cam(2));
     if cam(3)<nearestGroundZ
      	campos([cam(1),cam(2),nearestGroundZ]);
     end
  
    told = tnew;
    pause(0.01);
    
end
end


function KeyPress(varargin)
     fig = varargin{1};
     key = varargin{2}.Key;
     if strcmp(key,'e') 
         fig.UserData.e = true;
     elseif strcmp(key,'q')
         fig.UserData.q = true;
     elseif strcmp(key,'a')
         fig.UserData.a = true;
     elseif strcmp(key,'d')
         fig.UserData.d = true;
     elseif strcmp(key,'w')
         fig.UserData.w = true;
     elseif strcmp(key,'s')
         fig.UserData.s = true;
     elseif strcmp(key,'v')
         fig.UserData.firstPerson = ~fig.UserData.firstPerson;
     end
end

function KeyRelease(varargin)
     fig = varargin{1};
     key = varargin{2}.Key;
     if strcmp(key,'e') 
         fig.UserData.e = false;
     elseif strcmp(key,'q')
         fig.UserData.q = false;
     elseif strcmp(key,'a')
         fig.UserData.a = false;
     elseif strcmp(key,'d')
         fig.UserData.d = false;
     elseif strcmp(key,'w')
         fig.UserData.w = false;
     elseif strcmp(key,'s')
         fig.UserData.s = false;
     end
end