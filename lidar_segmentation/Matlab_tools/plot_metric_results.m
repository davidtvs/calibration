%% Plot data
% Simple Segmentation

close all, clc;

xs = linspace(0.5,3,21);
ys = [495.0502008032 439.6646586345	366.3493975904 249.6365461847 227.5281124498 212.4287148594	151.4678714859 143.2369477912 177.1325301205 186.5552208835 184.3383534137 188.4618473896 182.374497992 182.0251004016 183.6204819277 182.3765060241 182.3022088353	180.4196787149 180.3453815261 180.9477911647 180.0090361446];

ys1 = [984.7978	856.4185 685.6556 410.3737 358.9302 322.6553 175.2005 155.0718 236.1755 258.433 254.9387 264.8464 250.216 249.3493 252.0385 250.0983 249.9677 245.4192 245.2535 246.8241 244.4924];
ys2 = [190.4591	168.983	155.7171 135.8782 130.5528 130.1241 131.2842 133.6945 135.7462 135.5934 129.9531 130 129.9611 130.0224 133.7135 130.3578 130.3652 130.2956 130.3958 130.5584 130.5474];
ys3 = [134.982	136.822	135.3141 135.7462 135.6459 135.6192	135.7005 135.2109 135.1703 135.6565	135.6792 135.7174 135.662 135.6713 135.6048 135.6576 135.6042 135.6343 135.5934 135.4318 135.4892];

plot(xs,ys,'ro');
title('Simple Segmentation - complete route');
xlabel('Threshold');
ylabel('Medium Energy');

figure;
plot(xs,ys1,'ro');
title('Simple Segmentation - straight line');
xlabel('Threshold');
ylabel('Medium Energy');

figure;
plot(xs,ys2,'ro');
title('Simple Segmentation - roundabout approach');
xlabel('Threshold');
ylabel('Medium Energy');

figure;
plot(xs,ys3,'ro');
title('Simple Segmentation - roundabout');
xlabel('Threshold');
ylabel('Medium Energy');

% [r,c] = find(ys==min(min(ys)));
% xs(c)
% min_ys = min(ys) 
% 
% [r,c] = find(ys1==min(min(ys1)));
% xs(c)
% min_ys1 = min(ys1)
% 
% [r,c] = find(ys2==min(min(ys2)));
% xs(c)
% min_ys2 = min(ys2)

% [r,c] = find(ys3==min(min(ys3)));
% xs(c)
% min_ys3 = min(ys3)


%% Dietmayer Segmentation

close all, clc;

xd = linspace(0.5,3,21);
yd = [440.9086345382	387.1415662651	310.1305220884	194.1637550201	162.3480923695	148.9876506024	87.9975903614	78.4062248996	112.3365461847	123.0282128514	121.5735943775	126.671686747	120.5275100402	119.9789156627	121.3664658635	120.538253012	119.9497991968	117.6321285141	117.5975903614	117.7454819277	116.5830321285];

yd1 = [972.9426	837.6167	669.0346	407.6371	337.2997	322.536	175.0833	154.7607	236.1755	258.433	254.9387	254.9387	250.2147	249.0199	252.0386	250.0983	249.9677	245.4192	245.2535	246.8241	244.4924];
yd2 = [115.7114	89.4293	67.8988	48.4787	39.9978	38.8599	39.7587	42.0352	44.1337	43.9489	38.3208	38.3478	38.2713	38.2971	41.2969	38.0661	35.6304	33.4895	32.7552	32.4603	31.8815];
yd3 = [47.9716	61.3757	52.1562	41.2284	37.9297	21.7123	22.1347	18.8219	18.5544	21.8903	21.8705	25.8274	25.671	25.537	24.9265	24.9265	25.4215	25.1243	25.4323	24.3933	24.1276];

plot(xd,yd,'bo');
title('Dietmayer Segmentation - complete route');
xlabel('C0 value');
ylabel('Medium Energy');

figure;
plot(xd,yd1,'bo');
title('Dietmayer Segmentation - straight line');
xlabel('C0 value');
ylabel('Medium Energy');

figure;
plot(xd,yd2,'bo');
title('Dietmayer Segmentation - roundabout approach');
xlabel('C0 value');
ylabel('Medium Energy');

figure;
plot(xd,yd3,'bo');
title('Dietmayer Segmentation - roundabout');
xlabel('C0 value');
ylabel('Medium Energy');

% [r,c] = find(yd==min(min(yd)));
% xd(c)
% min_yd = min(yd) 

% [r,c] = find(yd1==min(min(yd1)));
% xd(c)
% min_yd1 = min(yd1)

% [r,c] = find(yd2==min(min(yd2)));
% xd(c)
% min_yd2 = min(yd2)
% 
% [r,c] = find(yd3==min(min(yd3)));
% xd(c)
% min_yd3 = min(yd3)

%% ABD Segmentation

close all, clc;

x_abd = linspace(2,20,21);
y_abd = [175.8514056225	178.8925702811	173.8935742972	175.3253012048	180.5773092369	160.733935743	194.4397590361	201.5080321285	206.6757028112	221.3664658635	228.2208835341	182.7218875502	204.516064257	169.3032128514	210.7751004016	205.6997991968	176.2520080321	186.9518072289	196.4036144578	193.1596385542	200.3363453815];

y_abd1 = [246.5701	254.56	243.7453	247.5479	259.5278	211.7002	293.0296	306.3271	319.5371	354.4549	372.7319	263.0407	315.19	230.8129	331.0563	320.6902	248.8853	274.6823	294.9242	286.002	306.1255];
y_abd2 = [94.3925	92.4271	89.9371	90.6941	90.6982	90.7714	90.4736	103.5284	99.7296	101.1071	96.2704	96.2896	96.3461	96.464	96.2414	88.8079	91.8218	91.9754	98.9602	102.3779	95.9059];
y_abd3 = [135.488	135.2207	135.0954	134.5345	135.189	135.13	135.1832	134.6187	135.0931	135.1419	135.0172	134.9734	135.3539	134.8048	134.654	135.217	135.0256	134.9914	135.1923	135.1445	134.5398];

plot(x_abd ,y_abd , 'go' );
title('ABD Segmentation - complete route');
xlabel('lamda');
ylabel('Medium Energy');

figure;
plot(x_abd ,y_abd1 , 'go' );
title('ABD Segmentation - straight line');
xlabel('lamda');
ylabel('Medium Energy');

figure;
plot(x_abd ,y_abd2 , 'go' );
title('ABD Segmentation - roundabout approach');
xlabel('lamda');
ylabel('Medium Energy');

figure;
plot(x_abd ,y_abd3 , 'go' );
title('ABD Segmentation - roundabout');
xlabel('lamda');
ylabel('Medium Energy');

% [r,c] = find(y_abd==min(min(y_abd)));
% x_abd(c)
% min_y_abd = min(y_abd) 

% [r,c] = find(y_abd1==min(min(y_abd1)));
% x_abd(c)
% min_y_abd1 = min(y_abd1) 
% 
% [r,c] = find(y_abd2==min(min(y_abd2)));
% x_abd(c)
% min_y_abd2 = min(y_abd2) 
% 
[r,c] = find(y_abd3==min(min(y_abd3)));
x_abd(c)
min_y_abd3 = min(y_abd3) 


%% NN Segmentation
close all, clc;

x_nn = linspace(0.5,3,21);
y_nn = [488.90562249	436.765060241	361.7630522088	292.8524096386	241.1214859438	220.0271084337	188.984939759	197.812248996	254.1204819277	266.7781124498	267.3955823293	262.3714859438	307.9126506024	252.5020080321	252.9618473896	269.8574297189	255.1807228916	245.8644578313	143.9026104418	242.8524096386	315.6024096386];

y_nn1 = [977.9474	852.6295	668.9471	511.1698	384.7421	334.2564	247.667	267.1259	387.5783	408.4429	405.4425	401.2184	514.1097	389.5823	383.4609	426.2251	422.8603	415.9343	414.3396	412.9372	588.5212];
y_nn2 = [207.864	207.864	180.6096	154.8784	140.9477	142.0257	144.2143	149.7381	154.5448	155.3536	160.1778	156.3921	161.0905	164.4618	161.8677	155.8393	145.6524	150.3339	147.7723	152.6163	157.497];
y_nn3 = [122.1912	127.7675	132.8359	132.8359	132.8359	138.1954	148.3987	156.838	161.14	169.8477	169.7332	166.3812	162.0686	152.5746	160.0946	151.9984	133.5292	117.6508	116.4366	112.9891	111.3381];

plot(x_nn ,y_nn , 'ko' );
title('NN Segmentation - complete route');
xlabel('Threshold');
ylabel('Medium Energy');

figure;
plot(x_nn ,y_nn1 , 'ko' );
title('NN Segmentation - straight line');
xlabel('Threshold');
ylabel('Medium Energy');

figure;
plot(x_nn ,y_nn2 , 'ko' );
title('NN Segmentation - roundabout approach');
xlabel('Threshold');
ylabel('Medium Energy');

figure;
plot(x_nn ,y_nn3 , 'ko');
title('NN Segmentation - roundabout');
xlabel('Threshold');
ylabel('Medium Energy');


% [r,c] = find(y_nn==min(min(y_nn)));
% x_nn(c)
% min_y_nn = min(y_nn) 
% 
% [r,c] = find(y_nn1==min(min(y_nn1)));
% x_nn(c)
% min_y_nn1 = min(y_nn1) 
% 
% [r,c] = find(y_nn2==min(min(y_nn2)));
% x_nn(c)
% min_y_nn2 = min(y_nn2) 
% 
[r,c] = find(y_nn3==min(min(y_nn3)));
x_nn(c)
min_y_nn3 = min(y_nn3) 


%% Premebida Segmentation
close all, clc;

x_p = linspace(0.475, 0.975,21);
y_p = [136.6161646586	156.9668674699	165.8704819277	171.1265060241	147.7580321285	144.2570281125	145.1124497992	156.2289156627	159.3323293173	160.8172690763	138.0903614458	140.9126506024	145.1516064257	156.4889558233	165.7469879518	160.2931726908	196.9246987952	176.0230923695	193.7991967871	231.453815261	252.6144578313];

y_p1 = [249.591	290.7295	303.7088	313.8661	257.288	248.8618	248.2052	273.4341	274.8665	282.0155	206.2954	209.9883	211.4883	233.7817	254.286	241.7041	319.7637	267.9615	308.7588	383.8972	438.9931];
y_p2 = [69.9323	79.359	97.5047	95.4323	95.6469	93.2007	93.1849	93.5187	91.8177	95.2283	99.015	103.3918	110.6314	115.2526	113.2793	110.7084	106.6947	109.5487	116.075	126.9373	126.3784];
y_p3 = [52.4475	56.8925	59.1494	62.117	62.2653	63.0077	65.5322	66.895	73.0427	68.6801	86.9081	88.4401	94.2889	97.5419	99.8449	100.1133	110.9056	111.3049	111.2054	122.3834	118.7958];

plot(x_p ,y_p , 'ro' );
title('Multivariable Segmentation - complete route');
xlabel('Cosine distance threshold');
ylabel('Medium Energy');

figure;
plot(x_p ,y_p1, 'ro' );
title('Multivariable Segmentation - straight line');
xlabel('Cosine distance threshold');
ylabel('Medium Energy');

figure;
plot(x_p ,y_p1, 'ro' );
title('Multivariable Segmentation - straight line');
xlabel('Cosine distance threshold');
ylabel('Medium Energy');

figure;
plot(x_p ,y_p2, 'ro' );
title('Multivariable Segmentation - roundabout approach');
xlabel('Cosine distance threshold');
ylabel('Medium Energy');

figure;
plot(x_p ,y_p3, 'ro' );
title('Multivariable Segmentation - roundabout');
xlabel('Cosine distance threshold');
ylabel('Medium Energy');

% [r,c] = find(y_p==min(min(y_p)));
% x_p(c)
% min_y_p = min(y_p) 

% [r,c] = find(y_p1==min(min(y_p1)));
% x_p(c)
% min_y_p1 = min(y_p1) 
% 
% [r,c] = find(y_p2==min(min(y_p2)));
% x_p(c)
% min_y_p2 = min(y_p2) 
% 
% [r,c] = find(y_p3==min(min(y_p3)));
% x_p(c)
% min_y_p3 = min(y_p3) 





