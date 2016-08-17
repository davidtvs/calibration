%% (# Clusters GT / # CLUSTERS ALG ) ratio

clc, close all, clear all;

path_number = 1;

pathToFolder = 'ultimate_centers/ms/path1';

files = dir(fullfile(pathToFolder, '*.txt') );
alg = cell(1,length(files));

% Ground truth file
gt = dlmread('ultimate_centers/gt/gt_p1.txt');

Total_ratio = [];
Total_Medium_ratio = [];

for w=1:numel(files); %21
    
    % open algorithm files
    alg = dlmread(fullfile(pathToFolder,files(w).name));
    
    iteration = 0;
    
    line_gt = 1;
    line_alg = 1;
    
    Sratio = 0;
    
    while (line_alg <= size(alg,1)) %end of file        
       
        a = gt(line_gt,:);
        axx = alg(line_alg,:);
        
        center_gt = [];
        center_alg = [];        
        
        if(a(1)== inf)
        
            c_end = a(2)+1;
            values = gt((line_gt+1):(line_gt+c_end-1),:);
             
            for k = 1:size(values,1);
                center_gt{k}.size = values(k,3);
            end
        end
        
        line_gt = line_gt+c_end;
        
        
        % Extrat the clusters centers form the algorithm file
        if(axx(1) == Inf && a(1) == Inf)
        
            c_end = axx(2)+1;
            values_x = alg((line_alg+1):(line_alg+c_end-1),:);
            
            for k = 1:size(values_x,1)
                center_alg{k}.size = values_x(k,3);
            end
        end
        
        line_alg = line_alg+c_end;
        
        clusters_gt_it = size(center_gt,2);
        clusters_alg_it = size(center_alg,2);
        
        ratio = clusters_gt_it/clusters_alg_it;
        
        Sratio = Sratio + ratio;
       
        %Next iteration
        iteration = iteration +1;
        
    end    %end while - Ended read file
    
        w 
    
        Total_ratio = [Total_ratio Sratio];
        
        Medium_ratio = Sratio/iteration;
        
        Total_Medium_ratio = [Total_Medium_ratio Medium_ratio];
        
        
end    %end for - Ended with this algorithm for this path

       %Write de results
       
       fn = ['Ratio_results/ms/ms_r_P' num2str(path_number) '.txt'];

       fid = fopen(fn,'w');
       fprintf(fid,'Number of iterations = %d\n', iteration);
       fprintf(fid,'%\n'); 
       
       fprintf(fid, '%f \n', Total_ratio);
       fprintf(fid, 'Medium_ratio = %f \n', Total_Medium_ratio);
       fclose(fid)
       
       
       
       
       
       
       


