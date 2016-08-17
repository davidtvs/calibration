
clc, close all, clear all;

path_number = 1;

%pathToFolder = 'ultimate_centers/sa_c/path1';
pathToFolder = 'NO_centers/sa_c/path1';
    
files = dir(fullfile(pathToFolder, '*.txt') );
alg = cell(1,length(files));

Total_Senergy1 = [];
Total_Senergy2 = []; 
Total_Medium_energy1 = [];
Total_Medium_energy2 = [];


% Ground truth file

%gt = dlmread('ultimate_centers/gt/gt_p1.txt');
gt = dlmread('NO_centers/gt/gt_p1.txt');
    
%read all files
for w =1:numel(files); %21
 
    % open algorithm files
    alg = dlmread(fullfile(pathToFolder,files(w).name));
          
    Senergy1 = 0;
    Senergy2 = 0;
    iteration = 0; 
    
    line_gt = 1;
    line_alg = 1;
 
    
while (line_alg <= size(alg,1)) %fim do ficheiro
 %while(iteration<1)       
        a = gt(line_gt,:);
        axx = alg(line_alg,:);
        
        center_gt = [];
        center_alg = [];
        
        % Extrat the clusters centers form the GT file
        if(a(1)==Inf)
            c_end = a(2)+1;
            values = gt((line_gt+1):(line_gt+c_end-1),:);
           
            for k = 1:size(values,1)
                center_gt{k}.x = values(k,1);
                center_gt{k}.y = values(k,2);
                center_gt{k}.size = values(k,3);
            end
        end
        
        line_gt = line_gt+c_end;
        
        
        % Extrat the clusters centers form the algorithm file
        if(axx(1) == Inf && a(1) == Inf)
        
            c_end = axx(2)+1;
            values_x = alg((line_alg+1):(line_alg+c_end-1),:);
            
            for k = 1:size(values_x,1)
                center_alg{k}.x = values_x(k,1);
                center_alg{k}.y = values_x(k,2);
                center_alg{k}.size = values_x(k,3);
            end
            
        end
        
        line_alg = line_alg+c_end;
        
     % Applying the METRIC 1 - each cluster algorithm, calculate the Euclidean
     % Distance to every Gt cluster  
     
        sum = 0;
        for ( i = 1:size(center_alg,2) )
        
            distance = [];
          
            for (j = 1:size(center_gt,2)) %Compare with groudtruth               
                new_distance =  norm([(center_alg{i}.x - center_gt{j}.x)  (center_alg{i}.y - center_gt{j}.y) ],2 );
                distance = [distance new_distance];                
            end    
        
            %find the correspondet clusters
            min_distance = min(distance);
            idx = find(distance==min_distance);
            m = max((center_alg{i}.size/center_gt{idx(1)}.size), (center_gt{idx(1)}.size /center_alg{i}.size) );
            aux = min_distance*m;
            sum = sum + aux;          
        
        end
        
       Energy1 = sum / size(center_alg,2);
        
      %    clusters_gt = size(center_gt,2);
      %    clusters_alg = size(center_alg,2);
         
      %  cena = max( (clusters_gt/clusters_alg) , (clusters_alg/clusters_gt)  );
        
%         Energy1 = sum * cena; 
        Senergy1 = Senergy1 + Energy1;
        
      %  Scena = [Scena cena];
        
     % Applying the METRIC 2 - each cluster GT, calculate the Euclidean
     % Distance to every algorithm cluster        
        
        sum2 = 0;
        for ( i = 1:size(center_gt,2))  %For every GT cluster
        
            distance = [];
            
            for (j = 1:size(center_alg,2)) %Compare with algorithm
            
                new_distance =  norm([(center_gt{i}.x - center_alg{j}.x)  (center_alg{j}.y - center_gt{i}.y) ],2 );
                distance = [distance new_distance];
                
            end   
               
            %find the correspondet cluster
            min_distance = min(distance);
            idx = find(distance==min_distance);
            m = max((center_gt{i}.size/center_alg{idx(1)}.size), (center_alg{idx(1)}.size /center_gt{i}.size) );
            aux = min_distance*m;
            sum2 = sum2 + aux;            
    
         end    
        
        Energy2 = sum2 / size(center_gt,2);                   
        Senergy2 = Senergy2 + Energy2;

        %Next iteration
        iteration = iteration +1;
           
end %end while ended read file
    
    w
    
    Total_Senergy1 = [Total_Senergy1 Senergy1];
    Total_Senergy2 = [Total_Senergy2 Senergy2];
    
    Medium_energy1 = Senergy1/iteration;
    Medium_energy2 = Senergy2/iteration;
   
    Total_Medium_energy1 = [Total_Medium_energy1 Medium_energy1];
    Total_Medium_energy2 = [Total_Medium_energy2 Medium_energy2];
       
    
end   %Ended with this algorithm for this path

    %Write results on file Metric 1

%      fn1 = ['Metric_results/sa_c/sa_c_m1_P' num2str(path_number) '.txt'];
       fn1 = ['NO_Metric_results/sa_c/sa_c_m1_P' num2str(path_number) '.txt'];
  
    fid_w1 = fopen(fn1,'w')   
    fprintf(fid_w1,'Number of iterations = %d\n', iteration);
    fprintf(fid_w1,'%\n'); 
    
    fprintf(fid_w1, '%f \n', Total_Senergy1);
    fprintf(fid_w1, 'Medium_Energy = %f \n', Total_Medium_energy1);
    fclose(fid_w1);
    
    %Write results on file Metric 2

  %      fn2 = ['Metric_results/sa_c/sa_c_m2_P' num2str(path_number) '.txt'];
      fn2 = ['NO_Metric_results/sa_c/sa_c_m2_P' num2str(path_number) '.txt'];

        
    fid_w2 = fopen(fn2,'w')   
    fprintf(fid_w2,'Number of iterations = %d\n', iteration);
    fprintf(fid_w2,'%\n'); 
    
    fprintf(fid_w2, '%f \n', Total_Senergy2);
    fprintf(fid_w2, 'Medium_Energy = %f \n', Total_Medium_energy2);
    fclose(fid_w2);    



