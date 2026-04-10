
%%
currentFolderName=cd;
txtFileAdress='\ewap_dataset\seq_hotel\obsmat.txt';
filename = strcat(currentFolderName,txtFileAdress); 
fileID = fopen(filename,'r');
formatSpec = '%16f%16f%16f%16f%16f%16f%16f%f%[^\n\r]';
dataArray = textscan(fileID, formatSpec, 'Delimiter', '', 'WhiteSpace', '', 'TextType', 'string', 'EmptyValue', NaN,  'ReturnOnError', false);
obsTable = table(dataArray{1:end-1}, 'VariableNames', {'e00','e1','e2','e3','e4','e01','e5','e6'});
obsMat =table2array(obsTable); % transformation from table to matrix
obsMat=obsMat(:,[1,2,3,5,6,8]); % deleting the columns corresponding to x_z and v_z
%%
figure
video=VideoReader('seq_hotel.avi');
frameNum=411;
% frameNumMat=[]
myread=read(video,frameNum);
imshow(myread)
%% Pixels are taken from the Cursor points
% BenchPix=[295,22,1;266,22,1;259,150,1;291,151,1];
% Tree1Pix=[285,246,1;276,246,1;275,256,1;285,256,1];
% % BenchPix=[246,285,1;246,276,1;256,275,1;256,282,1];
% LampPix=[278,394,1;272,394,1;272,402,1;278,402,1];
% Tree2Pix=[272,547,1;262,547,1;262,559,1;272,559,1];
% AllObstaclePix={BenchPix,Tree1Pix,LampPix,Tree2Pix};
% H=[   1.1048200e-02   6.6958900e-04  -3.3295300e+00;
%   -1.5966000e-03   1.1632400e-02  -5.3951400e+00;
%    1.1190700e-04   1.3617400e-05   5.4276600e-01];
% figure
% hold on
% set(gca, 'XLim', [0 16],'YLim', [0 16]);
% for kr=1:4
%     ObsPix=AllObstaclePix{kr};
%     ObsCoord=H*ObsPix';
%     ObsCoord=ObsCoord(:,:)./ObsCoord(3,:);
%     simObsCoordY=ObsCoord(1,:).*(-1)+10;
%     simObsCoordX=ObsCoord(2,:)+10;
%     simObsCoord=[simObsCoordX;simObsCoordY];
%     OBS{kr}=polyshape(simObsCoord(1,:),simObsCoord(2,:));
%     plot(OBS{kr})
% end

% plot(OBS{[1:4]})

%%
frameNumMat=frameNum:10:frameNum+200; % for depicting the trajetory
frameNumMat=411;
hold on
for fr=1:numel(frameNumMat)

% allPedID=[3,11,12,4,5,6,7]; % Some pedestrians from data set
frameMat=obsMat(obsMat(:,1)==frameNumMat(fr),:);
frameMat_XY=[frameMat(:,3:4)];
frameMat_XY=frameMat_XY';
frameMat_XY(3,1:end)=ones;
H=[   1.1048200e-02   6.6958900e-04  -3.3295300e+00;
  -1.5966000e-03   1.1632400e-02  -5.3951400e+00;
   1.1190700e-04   1.3617400e-05   5.4276600e-01];
% ped=[2.2598137e+00;-4.5465974e+00;1];
scaledMat=inv(H)*frameMat_XY;
pixelMat=scaledMat(:,:)./scaledMat(3,:);

% figure(12)
centers=[pixelMat(2,:);pixelMat(1,:)];
for i=1:numel(pixelMat(2,:))
%  viscircles([centers(1,i),centers(2,i)],5,'Color',(frameMat(i,2)-34)*[10/255 0 0],'LineWidth',(frameMat(i,2)-34)/2)
 viscircles([centers(1,i),centers(2,i)],5,'Color','c')
end
end