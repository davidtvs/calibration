clc, close all, clear all;

path_number = 1;

%Centers
pathToFolder = 'NO_centers/ms/path1';
files = dir(fullfile(pathToFolder, '*.txt') );
alg = cell(1,length(files));

%Boundaries
pathToFolder_bond = 'NO_boundaries/ms/path1';
files_bond = dir(fullfile(pathToFolder_bond, '*.txt') );
alg_bon = cell(1,length(files_bond));

% Ground truth file - centros
gt = dlmread('NO_centers/gt/gt_p1.txt');
% Ground truth file - BOUNDARIES
gt_bond = dlmread('NO_boundaries/gt/gt_p1.txt');

Total_Senergy2A = []; 
Total_Medium_energy2A = [];

%read all files
for w =1:numel(files); %21
   
    % open algorithm files
    alg = dlmread(fullfile(pathToFolder,files(w).name));
    alg_bond = dlmread(fullfile(pathToFolder_bond,files_bond(w).name));
    
    Senergy2A = 0;
    iteration = 0; 
    
    line_gt = 1;
    line_gt_bond = 1;
    line_alg = 1;
    line_alg_bond = 1;

while (line_alg <= size(alg,1)) %fim do ficheiro
         
    a = gt(line_gt,:);
    axx = alg(line_alg,:);    
        
    center_gt = [];
    center_alg = [];
   
    BOND_gt = [];
    BOND_alg = [];
        
    % Extrat the clusters CENTERS AND BOUNDARIES form the GT file
    if(a(1)==Inf)
        
            c_end = a(2)+1;
            values = gt((line_gt+1):(line_gt+c_end-1),:);
            bonds = gt_bond((line_gt+1):(line_gt+c_end-1),:);
            
            for k = 1:size(values,1)
                center_gt{k}.x = values(k,1);
                center_gt{k}.y = values(k,2);
                center_gt{k}.size = values(k,3);
            end
            
            for kb = 1:size(bonds,1)
                BOND_gt{kb}.xi = bonds(kb,1);
                BOND_gt{kb}.yi = bonds(kb,2);
                BOND_gt{kb}.xf = bonds(kb,3);
                BOND_gt{kb}.yf = bonds(kb,4);
            end    
    end
        
    line_gt = line_gt+c_end;
        
        
    % Extrat the clusters centers and Boundaries form the algorithm file
    if(axx(1) == Inf && a(1) == Inf)
        
            c_end = axx(2)+1;
            values_x = alg((line_alg+1):(line_alg+c_end-1),:);
            
            bonds_x = alg_bond((line_alg+1):(line_alg+c_end-1),:);
            
            for k = 1:size(values_x,1)
                center_alg{k}.x = values_x(k,1);
                center_alg{k}.y = values_x(k,2);
                center_alg{k}.size = values_x(k,3);
            end
            
            for kb = 1:size(bonds_x,1)
                BOND_alg{kb}.xi = bonds_x(kb,1);
                BOND_alg{kb}.yi = bonds_x(kb,2);
                BOND_alg{kb}.xf = bonds_x(kb,3);
                BOND_alg{kb}.yf = bonds_x(kb,4);
            end
      end
       
      line_alg = line_alg+c_end;
        
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
            
            %calculate the boundarie distences
            initial_dis = norm([ (BOND_gt{i}.xi - BOND_alg{idx(1)}.xi) (BOND_gt{i}.yi - BOND_alg{idx(1)}.yi)] , 2);
            final_dis =   norm([ (BOND_gt{i}.xf - BOND_alg{idx(1)}.xf) (BOND_gt{i}.yf - BOND_alg{idx(1)}.yf)] , 2);
            
            aux = (initial_dis + final_dis);              
            sum2 = sum2 + aux;            
    
        end    
        
        n_alg =  size(center_alg,2);
        n_gt = size(center_gt,2);
        
        m2 = min( (n_alg/n_gt) , (n_gt/n_alg) );
        
        Energy2A = sum2 / m2;                   
        Senergy2A = Senergy2A + Energy2A;

        %Next iteration
        iteration = iteration +1;
           
end %end while ended read file
    
    w
    
    Total_Senergy2A = [Total_Senergy2A Senergy2A];
    Medium_energy2A = Senergy2A/iteration;
   
    Total_Medium_energy2A = [Total_Medium_energy2A Medium_energy2A];
       
    
end   %Ended with this algorithm for this path


%      fn2 = ['Metric_results/sa_c/sa_c_m2_P' num2str(path_number) '.txt'];
fn3 = ['Metric_TEST_AA/ms/ms_NO_m3A_P' num2str(path_number) '.txt'];

fid_w3 = fopen(fn3,'w')   
fprintf(fid_w3,'Number of iterations = %d\n', iteration);
fprintf(fid_w3,'%\n'); 
    
fprintf(fid_w3, '%f \n', Total_Senergy2A);
fprintf(fid_w3, 'Medium_Energy = %f \n', Total_Medium_energy2A);
fclose(fid_w3);    

    
    
    