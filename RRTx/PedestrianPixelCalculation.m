clear all
currentFolderName=cd;
txtFileAdress='\ewap_dataset\seq_eth\obsmat.txt';
filename = strcat(currentFolderName,txtFileAdress); 
fileID = fopen(filename,'r');
formatSpec = '%16f%16f%16f%16f%16f%16f%16f%f%[^\n\r]';
dataArray = textscan(fileID, formatSpec, 'Delimiter', '', 'WhiteSpace', '', 'TextType', 'string', 'EmptyValue', NaN,  'ReturnOnError', false);
obsTable = table(dataArray{1:end-1}, 'VariableNames', {'e00','e1','e2','e3','e4','e01','e5','e6'});
obsMat =table2array(obsTable); % transformation from table to matrix
obsMat=obsMat(:,[1,2,3,4,5]); % deleting the columns corresponding to x_z and v_z

numStaticObs=0;

pedID=[1:3]; % Id of pedestrian
initTimePlanning=70; % Initial waiting time for planning 
for k=1:numel(pedID)
    rowPedest=obsMat(:,2)==pedID(k);
    pedData=obsMat(rowPedest,:);
    sampNum=numel(pedData(:,3)); % number of data for the pedestrian
    
    % IMPORTANT NOTE:the x direction in the pedestrian data actually correspons to y directon our
    % graph, the same change is applied for y directon in the pedestrian
    % data
    time=pedData(:,1);
    xRealPos= pedData(:,3); % y pos of pedestrian with sampling time 0.4 seconds
    zRealPos= pedData(:,4);
    yRealPos= pedData(:,5); % x pos of pedestrian with sampling time 0.4 seconds
    
    obstacle(numStaticObs+k).Path(1:4,:)=[xRealPos';yRealPos';ones(1,length(xRealPos));time'];
    
end
H=[   2.8128700e-02   2.0091900e-03  -4.6693600e+00;
   8.0625700e-04   2.5195500e-02  -5.0608800e+00;
   3.4555400e-04   9.2512200e-05   4.6255300e-01];
Hinv=inv(H);
for k=1:numel(pedID)
    scaledRes=Hinv*obstacle(numStaticObs+k).Path(1:3,:);
    obstacle(numStaticObs+k).Pixel=scaledRes(:,:)./scaledRes(3,:);
    obstacle(numStaticObs+k).Pixel(end+1,:)=obstacle(numStaticObs+k).Path(4,:);
end