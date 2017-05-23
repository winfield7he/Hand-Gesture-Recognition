% This part is about hand feature extraction, in which hand direction can be determined without arm direction and relative axis system has been establisheed.
% The code is modified on the basis of the work conducted by Rishabh Sharma whith some innovative algorithms given by Wenfei He
clear all; clc; % clear everthing 
close all; A=16; % close everything
I=imread('Lefthand_17.png'); % load depth image 
Depth_left=I;
Depth_left_ori=I;
Mask_T=im2double(Depth_left); % change the image type into double 
[a,b,c]=size(Mask_T);
Temp=zeros(a+4,b+4); % enloarge the mattrix to support edge detection
for i=1:1:a;
    for j=1:1:b;
        if ~isempty(Mask_T)
            Temp(i+2,j+2)=Mask_T(i,j); 
        end
    end
end
Mask=Temp;
z1=graythresh(Mask);  % Global trhesholding the image 
z2 = 0.5*(min(Mask(:)) + max(Mask(:))); % % Calculate the second threshold value 
[a,b,c]=size(Mask);
for u = 1:1:a
    for v = 1:1:b
        if Mask(u, v) < z1 &&  Mask(u, v) > z1-z2 % partition hand from images
            Mask(u, v) = 1;
        else Mask(u, v) = 0;
        end
    end
end
Out=Mask; % output the reslut as a mask
BW=edge(Out,'sobel'); % using Sobel edge detector to extract hand edge

F=im2uint16(Temp); % convert the image into 16 byte image 
[a,b]=size(Temp);
for u=1:1:a;
    for v=1:1:b
        if Mask(u,v)==0;
            F(u,v)=0;
        else
            F(u,v)=20*F(u,v); % adjust hand image contrast 
        end
    end
end
I2=F;
I3=im2double(F); % convert the result into double

I4=imgaussfilt(I2,A); % apply gaussian filter with the standard deviation on the segmented image
[m,n]=(find(I4==max(max(I4)))); % find the pixel with higest intensity value
m1=round(mean(m));
n1=round(mean(n));
F(m1,n1)=100*F(m1,n1); % highlight the hand palm central point on the image 

Mor=strel('disk',3);  % create a disk with a smaill radius         
I5=imerode(I3,Mor);  % morphological erosion on hand image with the disk 

I6=imgaussfilt(I5,A); % applied Gaussian filter on erosed hand image
[c,d]=(find(I6==max(max(I6)))); % find the pixel with the highest intensity value on erosed hand image 
m2=round(mean(c));
n2=round(mean(d));
F(m2,n2)=100*F(m2,n2); % highlight the hand palm central point on the image

BW1=edge(F,'sobel'); % using Soble edge detector to support relative axis establishment and orientation normalization
Temp1=0.0001*ones(a,b); % convert zero pixels into non-zero pixels
% determine the gradient using two hand central points
for u=1:1:a
    for v=1:1:b
        if(BW(u,v)==1&& n1==n2&& m1~=m2)
            Gredian_H=90; 
            Temp1(m2,n2)=300;
        elseif(BW(u,v)==1&& n1~=n2&& m1==m2)
            Gredian_H=0;
            Temp1(m2,n2)=300;
        elseif(BW(u,v)==1&& n1~=n2&& m1~=m2)
            Gredian_H=round(atan((m2-m1)/(n2-n1))*180/pi);
            Temp1(m2,n2)=300;
        end
    end
end
% hand image orientation normalization 
for u=1:1:a
    for v=1:1:b
        if(BW(u,v)==1)
            if(v==n1)
                Temp1(u,v)=90;
                Temp1(m1,n1)=300;
            else
               Temp1(u,v)=round(atan((u-m1)/(v-n1))*180/pi);
               Temp1(m1,n1)=300;
            end
        end
    end
end
% Determine the first axis using angles given in orientation normalization and hand direction 
if Gredian_H==0
    for v=1:1:n1
        if(Temp1(m1,v)==0)
            y1=[m1];
            x1=[v];
        end
        for j2=n1:1:a
            if(Temp1(m1,j2)==0)
            y2=[m1];
            x2=[j2];
            end
        end
    end
elseif Gredian_H==90
    for u=1:1:m1
        if (Temp1(u,n1)==90)
            y1=[u];
            x1=[n1];
        end
    for i2=m1:1:b
        if (Temp1(i2,n1)==90)
            y2=[i2];
            x2=[n1];
        end
    end
    end
elseif Gredian_H~=0&& Gredian_H~=90
    for u=1:1:m1
        for j=1:1:b
            if(Temp1(u,j)==Gredian_H)
                y1=[u];
                x1=[j];
            end
        end
    end
    for i2=m1:1:a
        for j=1:1:b
            if(Temp1(i2,j)==Gredian_H)
                y2=[i2];
                x2=[j];
            end
        end
    end
end
% determine the first pair of coordinates on the hand edge by detected axis
Ly1=[y1 y2];
Lx1=[x1 x2];
% hand edge separation using orientation normilization 
BW1 = edge(Out,'canny'); 
theta=zeros(a,b);
part1=zeros(a,b);
part2=zeros(a,b);
part3=zeros(a,b);
part4=zeros(a,b);
for i=1:a   
    for j=1:b     
        if (BW1(i,j)==1 )  
            for u=1:a    
                for v=1:b  
                    if (BW1(u,v)==1)  
                        if(u< m1 && v>=n1)        
                    part1(u,v)=90+atan((n1-v)/(m1-u))*180/pi; 
                        elseif(u<m1 && v<n1)          
                    part2(u,v)=90+(atan((n1-v)/(m1-u))*180/pi); 
                        elseif(u>=m1 && v<n1)          
                    part3(u,v)=90+atan((n1-v)/(m1-u))*180/pi; 
                        elseif(u>=m1 && v>=n1)            
                    part4(u,v)=90+(atan((n1-v)/(m1-u))*180/pi); 
                end
                    end
                end
            end
        end
    end
end
part1=round(part1);
part2=round(part2);
part3=round(part3);
part4=round(part4);

Combine1=part1+part2; % the top section 
Combine2=part3+part4; % the bottom section 
Combine3=part1+part4; % the left section 
Combine4=part2+part3; % the right section 
Combine=part1+part2+part3+part4;
% distance within hand edge calculation 
Horizontal=0; 
X1=0; 
Y1=0;
X2=0;
Y2=0; 
for i=1:a   
    for j=1:b     
        if (Combine1(i,j)>0 )     
            for u=1:a         
                for v=1:b       
                    if (Combine2(u,v)>0 )      
                        if(Combine1(i,j)==Combine2(u,v))   
                            Distance1=sqrt(((i-u).^2+(j-v).^2));
                            % calculate distance between pixels and palm center    
                            if(Distance1>=Horizontal)                   
                                Horizontal=Distance1;                
                                X1=i;      
                                Y1=j;      
                                X2=u;     
                                Y2=v;    
                            else
                                Horizontal=Horizontal;    
                                X1=X1;
                                Y1=Y1;       
                                X2=X2; 
                                Y2=Y2; 
                            end
                        end
                    end
                end
            end
        end
    end
end
Vertical=0; 
X3=0; 
Y3=0;
X4=0;
Y4=0; 
for u=1:a   
    for v=1:b     
        if (Combine3(u,v)>0 )     
            for u1=1:a         
                for v1=1:b       
                    if (Combine4(u1,v1)>0 )      
                        if(Combine3(u,v)==Combine4(u1,v1))   
                            Distance2=sqrt(((u-u1).^2+(v-v1).^2));     
                            if(Distance2>=Vertical)                   
                                Vertical=Distance2;                
                                X3=u;      
                                Y3=v;      
                                X4=u1;     
                                Y4=v1;    
                            else
                                Vertical=Vertical;    
                                X3=X3;
                                Y3=Y3;       
                                X4=X4; 
                                Y4=Y4; 
                            end
                        end
                    end
                end
            end
        end
    end
end
% find the largest distance by comparing two distance given by above determination
if Distance1>=Distance2
Lx=[Y2 Y1];  
Ly=[X2 X1]; 
Flag=Combine1;
else
Lx=[Y4 Y3]; 
Ly=[X4 X3]; 
Flag=Combine3;
end
% Acquire the orthogonal axis 
Gredian_V=Flag(Ly1(:,2),Lx1(:,2));
if(Gredian_V>90) 
    Gredian_V=Gredian_V-90;   
elseif(Gredian_V<90)   
      Gredian_V=(90+Gredian_V);   
else
    Gredian_V=0;
end
Range_low=Gredian_V-5; 
if (Range_low<=0) 
    Range_low=1;
else
    Range_low=Range_low; 
end
Range_high=Gredian_V+5;
% determine the corrresponding coordinates given by the orthogonal axis
[y3,x3] = find (Combine1<Range_high & Combine1>Range_low); 
[y4,x4] = find (Combine2<Range_high & Combine2>Range_low); 
 y3=(mean(y3)); 
 y4=(mean(y4));  
 x3=(mean(x3)); 
 x4=(mean(x4));    
 Lx2=[x4 x3];  
 Ly2=[y4 y3]; 
% plot the relative axis on hand edge image with palm centers
figure;
imshow(BW)
hold on;
line(Lx1,Ly1,'LineWidth',2);
plot(Lx1(1,:),Ly1(1,:), 'c+', 'MarkerSize', 5, 'LineWidth', 3); 
plot(Lx1(:,2),Ly1(:,2), 'r+', 'MarkerSize', 5, 'LineWidth', 3);
line(Lx,Ly,'Color','y')
plot(Lx(1,:),Ly(1,:), 'c+', 'MarkerSize', 5, 'LineWidth', 3); 
plot(Lx(:,2),Ly(:,2), 'g+', 'MarkerSize', 5, 'LineWidth', 3);
line(Lx2,Ly2,'LineWidth',2);
plot(Lx2(1,:),Ly2(1,:), 'c+', 'MarkerSize', 5, 'LineWidth', 3); 
plot(Lx2(:,2),Ly2(:,2), 'm+', 'MarkerSize', 5, 'LineWidth', 3);
plot(n1, m1, 'rs', 'MarkerSize', 1, 'LineWidth', 3);  
plot(n2, m2, 'gs', 'MarkerSize', 1, 'LineWidth', 3); 

%% Palm area estimation, curvature and distance feature extraction
% this part is given by Rishabh_Sharma 
% calculates the radius of palm area
D=sqrt(((x3-x4).^2+(y3-y4).^2)); % calculate the distance of the orthogonal axis
R=mean(D)/2; 
px = n1-R; 
py = m1-R; 
h = rectangle('Position',[px py D D],'Curvature',[1,1]);
set(h,'edgecolor','r') 
daspect([1,1,1]) 

%The following part was designed to discard the wrist part, but this approach is less reliable since the acquired hand edge has not been smooth very well,
%this issue will be addresse in future. 
radius=R-5; 
for i=1:a 
    for j=1:b
        if(sqrt((m1-(i))^2+(n1-j)^2)<radius)
            A(i,j)=1; 
        else A(i,j)=0;
        end
    end
end
A2=BW-A;
% discard wrist part and leave the palm area for distance feature acquisition and normilization
for i=1:a    
    for j=1:b 
         if (A2(i,j)==1 )       
             for u=1:a 
                 for v=1:b  
                     if (A2(u,v)==1)  
                         if(u>=m1 && v<=n1)  
                theta(u,v)=270+atan((n1-v)/(m1-u))*180/pi; %red           
                dist(u,v)=sqrt(((i-u).^2+(j-v).^2));  
                elseif(u<=m1 && v<n1)          
                    dist(u,v)=sqrt(((i-u).^2+(j-v).^2));            
                    theta(u,v)=90+atan((n1-v)/(m1-u))*180/pi; %green   
                elseif(u<m1 && v>=n1)       
                    dist(u,v)=sqrt(((i-u).^2+(j-v).^2));   %yellow             
                    theta(u,v)=90+atan((n1-v)/(m1-u))*180/pi;   
                elseif(u>m1 && v>n1)        
                    dist(u,v)=sqrt(((i-u).^2+(j-v).^2));        
                    theta(u,v)=270+atan((n1-v)/(m1-u))*180/pi;  %blue   
                         end
                     end
                 end
             end
         end
    end
end
dist=round(dist); % distance array extraction 
theta=round(theta);
for i=1:a  
    for j=1:b 
        if(theta(i,j)>210 && theta(i,j)<330)  
        theta(i,j)=0;   
        else
            theta(i,j)=theta(i,j);  
        end
    end
end

dist2=zeros(a,b); 
for c=1:360  
    [a1,b1]=find(theta==c); 
    for i=1:size(a1)    
        dist2=(dist(a1(i),b1(i)));  
    end
    dist1(c)=max(max(dist2));
end

distemp=dist1;
t=15;
for c=1:360  
    if (t>359) 
        dist1(c)=distemp(t-359); 
    else
        dist1(c)=distemp(t);   
    end
    t=t+1;
end

count=1;
i=1; 
for c=1:360  
    if(c==count) 
        tempdis=[dist1(c) dist1(c+1) dist1(c+2) dist1(c+3) dist1(c+4)  ];  
        DistMax(i)=max(tempdis);  
     count=count+5;   
     i=i+1;
    end
end
% normalize distance feature 
Lmax=max(DistMax);
DistFeature=(DistMax-R)/Lmax;

for i=1:72  
    if(DistFeature(i)>0)   
        DistFeature(i)=DistFeature(i);   
    else
        DistFeature(i)=0;
    end
end
% plot distance features in a histogram
figure 
plot(DistFeature); 
xlabel('Angles in degree');
ylabel('Normalised Distance');
title('Distance Feature');
% extract curvature feature using circles of three different radius
dist=round(dist);
theta=round(theta); 
thetaC=zeros(a,b);
for i=1:a 
    for j=1:b 
        if(theta(i,j)>0)  
        thetaC(i,j)=1;   
        else
            thetaC(i,j)=0;     
        end
    end
end
r1=3; 
r2=6; 
r3=9;
area1=round(pi*(r1)^2); 
area2=round(pi*(r2)^2);
area3=round(pi*(r3)^2);
num=0;
c1=zeros(a,b); 
c2=zeros(a,b); 
c3=zeros(a,b);  
 for i=1:a  
     for j=1:b 
         if (BW(i,j)==1)   
             num=num+1;   
        for u=1:a   
            for v=1:b    
                if(sqrt((i-(u))^2+(j-v)^2)<r1)      
                    c1(u,v)=0;       
                else
                    c1(u,v)=1;    
                end
                if(sqrt((i-(u))^2+(j-v)^2)<r2)        
                    c2(u,v)=0;         
                else
                    c2(u,v)=1;     
                end
                if(sqrt((i-(u))^2+(j-v)^2)<r3) 
                    c3(u,v)=0;        
                else
                    c3(u,v)=1;     
                end
            end
        end
        dif1=Out-c1;       
        dif2=Out-c2;      
        dif3=Out-c3;       
        areadif1  = regionprops(dif1, 'area');     
        areadif2  = regionprops(dif2, 'area');    
        areadif3  = regionprops(dif3, 'area');
        cur_1(num)= areadif1(1).Area/(area1); 
        cur_2(num)= areadif2(1).Area/(area2);      
        cur_3(num)= areadif3(1).Area/(area3);  
     end  
     end
 end
figure 
bin=0.1:0.05:0.9; 
cur=[cur_1,cur_2,cur_3]; % combine all data from three circles
hisT=hist(cur,bin); % plot the curvature feature in a histogram
hist(cur,bin);
xlabel('Ratio of area');
ylabel('Numbers of Elements');
CC= findobj(gca,'Type','patch');
set(CC,'FaceColor','[0 0.5 0.5]','EdgeColor','w')
feature=[DistMax,hisT];% output the distance feature and curvatuer feature
