% This program of hand extraction is based and modified on the previous work published on the internet
% Thanks to the framework by Chanseok Kang and Sanchul Kim
% The source of the original code is: https://github.com/goodboychan/handGesture
%
function Kinect_hand_tracking()
   colorVid = videoinput('kinect', 1,'RGB_640x480');  %create the video input for the color sensor
   depthVid = videoinput('kinect', 2,'Depth_640x480'); %create the video input for the depth sensor
    triggerconfig(colorVid, 'manual'); % configure trigger properties for the color sensor
    triggerconfig(depthVid, 'manual'); % configure trigger properties for the depth sensor
%     colorVid.FramesPerTrigger = 1; % 1 frame per trigger accquire from the color sensor
%     colorVid.TriggerRepeat = inf; % do not stop the trigger for color sensor until the stop button pressed
    depthVid.FramesPerTrigger = 1; % 1 frame per trigger accquire from the depth sensor
    depthVid.TriggerRepeat = inf; % do not stop the trigger for depth sensor until the stop button pressed
    Temp=getselectedsource(depthVid); %get the properties on the depth sensor
    set(Temp, 'TrackingMode', 'Skeleton'); % turn on the tracking mode of the depth sensor
    getselectedsource(depthVid) % double-check if the tracking mode is open
    
T = timer('TimerFcn', @hand_extraction, 'Period', 0.1,'executionMode', 'fixedRate'); % Building a timer for driving the trigger per 0.1 second
window=figure('Color',[0.8 0.8 0.8],'Name','Depth Camera', 'DockControl','off','Units','Pixels', 'toolbar','none', 'Position',[50 50 800 600]); % create a GUI with uicontrol in gray
             
start_camera=uicontrol('Parent',window,'Style','pushbutton','String','Start Camera','FontSize',11 ,'Units','normalized', 'Position',[0.8 0.32 0.2 0.08],'Callback',@startCallback);%create start button which calls the camera_start function
    
stop_camera=uicontrol('Parent',window,'Style','pushbutton','String','Stop Camera', 'FontSize',11 , 'Units','normalized','Position',[0.8 0.18 0.2 0.08],'Callback',@stopCallback);%create stop button which calls the the camera_stop function
n = 0; % give a parameter of mod function for the time duration
m= 1; % the number for the first image 
S=1/50; % the slope for the computation of the area of the rectangle according to the distance between the camera and hand
% threshold=2500;
function startCallback(~, ~)
       start(depthVid);  % start the skeleton tracking when start botton pressed
       start(T);  %start the timer when start botton pressed
    end

function stopCallback(~, ~)
       stop(depthVid);  % stop the skeleton tracking when start botton pressed
       stop(colorVid);
       stop(T); %stop the timer when start botton pressed
%        closepreview(colorVid);
end

function hand_extraction(~, ~)
trigger(depthVid); % trigger the depth camera according to the timer
[frame,ts, metaData]=getdata(depthVid); % get the skeletal data from the depth sensor
identification_joints = find(metaData.IsSkeletonTracked); % locate and get the tracked skeleton data
colorMap=getsnapshot(colorVid); % take a snapshot using the color sensor 
subplot 221; % creat a 2*2 layout in the figure and put the image from snapshot in the first position
imshow(colorMap);  % show the colorful frame from the color sensor 
title('RGB-D information'); % give a title for the RGB frame
subplot 222; % put the depth frame in the second position of the figure
imshow(frame, [0 4096]); %show the colorful frame from the color sensor using the grayscale of 0 to 4096
title('Depth information'); % give a title for the depth frame
% preview(colorVid);

if identification_joints ~= 0 % if the skeleton date has been tracked then start
Position_leftHand = metaData.JointDepthIndices(8,:,identification_joints);  %find and extract the position of the lefthand from the tracked skeleton data
RealPosition_leftHand=metaData.JointWorldCoordinates(8,:,identification_joints); % find and extract the real position of the lefthand from the tracked skeleton data
zAxes_coordinate = min(RealPosition_leftHand); % calculate the minimum distance between the hand and the Kinect sensor
Radius = round(100 - S*zAxes_coordinate); % set up a radius to gurantee the area of the handbox will decrease when the distance increases
HandBox_left = [Position_leftHand-0.5*Radius Radius Radius]; % create a handbox using above radius to enlarge the left hand 

rectangle('position', HandBox_left, 'EdgeColor', 'g','LineWidth',2); % create a green rectangle using the size of the handbox
Depth_left = imcrop(frame,HandBox_left); % crop the hand from the depth frame using the size of the handbox
Depth_left_Ori =(Depth_left); %% increase the contrast of the cropped hand depth images
subplot 223; % put it into the third position of the figure
imshow(Depth_left, [0 4096]); % show the cropped image using the grayscale of 0 to 4096
title('hand-tracking'); % give a title for the cropped image

           if ~isempty(Depth_left)% start when the cropped image got 
               F=im2double(Depth_left);       %% convert the image into double type
               i=graythresh(F);               %% create the global threshold by the Otsu's Method 
               M=im2bw(F,i); %% convert the double image into double and replace all pixels in the input image with luminance greater than i
               se=strel('disk',10);           %% create morphological structuring element
               ft=imtophat(F,se);             %% manipulate the top-hat filter by se
               j=graythresh(ft);              %% create local thresholding
               N=im2bw(F,j); %% convert the double image into double and replace all pixels in the input image with luminance greater than j
               Out=-M+N; 
           end

n = n+1; % integer n increases with the timer
if (mod(n,5)==1) % use the remainder for the names of the written images prieodically
    Temp3=imadjust(Out); % adjust the intensity value of the images before creat them in file exploer
imwrite(Temp3, strcat('Lefthand_seg','_',num2str(m),'.png'),'png'); % output the result of segmentated images with the name following the style as 'Letter_number.png'
imwrite(colorMap, strcat('Lefthand_RGB','_',num2str(m),'.png'),'png');% ouput the RGB images with the name following the style as 'Letter_number.png'
imwrite(Depth_left_Ori, strcat('Lefthand','_',num2str(m),'.png'),'png'); % ouput the cropped depth images with the name following the style as 'Letter_number.png'
m=m+1; % after creating an image, integer increases once
% end
end
end
end
end
           

