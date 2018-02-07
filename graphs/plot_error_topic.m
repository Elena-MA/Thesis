%Plotting data from error topic

%Loading files
XYError=csvread('error_D.txt');
yaw_error=csvread('error_Yaw.txt');
markers=csvread('markers.txt');
ErrorX=csvread('error_X.txt');
ErrorY=csvread('error_Y.txt');
valorD=max(XYError(2:end,2));
valorX=max(ErrorX(2:end,2));
valorCY=max(ErrorY(2:end,2));
%Starting from time 0
XYError(:,1)=XYError(:,1)-XYError(1,1);
yaw_error(:,1)=yaw_error(:,1)-yaw_error(1,1);

%Quitar marcadores en laser (hibrido)
for i=2:size(markers,1)-2
    if (markers (i,2)== 0 && markers(i-1,2)>=1 && markers(i+1,2)>=1)
        markers(i,2)=1;
    end
end
num=markers(:,2);
num(num >= 1)=valorD;
markers(:,1)= markers(:,1)-markers(1,1);
ErrorX(:,1)=ErrorX(:,1)-ErrorX(1,1);
ErrorY(:,1)=ErrorY(:,1)-ErrorY(1,1);

%from nsecs to secs
XYError(:,1)=XYError(:,1)/1000000000;
yaw_error(:,1)=yaw_error(:,1)/1000000000;
markers(:,1)=markers(:,1)/1000000000;
ErrorX(:,1)=ErrorX(:,1)/1000000000;
ErrorY(:,1)=ErrorY(:,1)/1000000000;

%from radians to degrees
yaw_error(:,2)=yaw_error(:,2)*(360/(2*pi));
max_yaw=max(yaw_error(2:end,2));

%Plotting
subplot(2,2,1);
plot(XYError(2:end,1),XYError(2:end,2),'color','b','LineWidth',1);
hold on;
plot(XYError(2:end,1),num(2:end),'color','r');
%plot(markers(2:50,1),markers(2:50,2));
title('Absolute Error');
xlabel('time(seconds)');
ylabel('Error(meters)');
%legend('Error','Number of markers');

numx=markers(:,2);
numx(numx >= 1)=valorX;
subplot(2,2,2);
plot(ErrorX(2:end,1),ErrorX(2:end,2),'color','b','LineWidth',1);
hold on;
plot(ErrorX(2:end,1),numx(2:end),'color','r','LineWidth',1);
%plot(markers(2:50,1),markers(2:50,2));
title('Error X');
xlabel('time(seconds)');
ylabel('Error(meters)');
%leg50('Error','Number of markers');
numy=markers(:,2);
numy(numy >= 1)=valorCY;
subplot(2,2,3);
plot(ErrorY(2:end,1),ErrorY(2:end,2),'color','b','LineWidth',1);
hold on;
plot(XYError(2:end,1),numy(2:end),'color','r','LineWidth',1);
%plot(markers(2:50,1),markers(2:50,2));
title('Error Y');
xlabel('time(seconds)');
ylabel('Error(meters)');
%legend('Error','Number of markers');

numyaw=markers(:,2);
numyaw(numyaw >= 1)=max_yaw;

subplot(2,2,4);
plot(yaw_error(2:end,1),yaw_error(2:end,2),'color','b','LineWidth',1);
hold on;
plot(yaw_error(2:end,1),numyaw(2:end),'color','r','LineWidth',1);
%plot(markers(2:50,1),markers(2:50,2));
title('Yaw Error');
xlabel('time(seconds)');
ylabel('Error(degrees)');
%legend('Error','Number of markers');


figure;
plot(markers(2:end,1),markers(2:end,2));
title('Number of visual markers');
xlabel('time(seconds)');
ylabel('Number');


