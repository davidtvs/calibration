%% Simple Segmentation -- METRIC
% Ratios + Metrics

clear all, close all, clc;

xs = linspace(0.5,3.2,21);

ysr = [0.3600569123	0.4520463887	0.5490273241	0.6415356759	0.70240809	0.7412226955	0.7602669827	0.777757391	0.7905924048	0.8014123322	0.8134991142	0.8291913356	0.8449269954	0.861036451	0.8741983264	0.8885503576	0.8994146275	0.9065365306	0.9161304245	0.9232031326	0.9371049666];
ysr1 = [0.1492532107	0.249242276	0.393149	0.5342866368	0.6264984019	0.6863824794	0.7164910024	0.7478083874	0.771520908	0.7868846271	0.8054891937	0.8295680412	0.85430323	0.8778057966	0.8967740218	0.9145568983	0.9284986344	0.9358796295	0.9474861501	0.9586842203	0.9818278717];
ysr2 = [0.3074804259	0.4210264444	0.5287428796	0.6170023241	0.6664164537	0.693012	0.7042186111	0.7077651667	0.7129448426	0.7177192963	0.7207194259	0.7307838704	0.7351837037	0.7616843426	0.7763321667	0.8005668333	0.8174161852	0.8254811667	0.8403022315	0.847160037	0.8542915926];
ysr3 = [0.6280921416	0.7038043439	0.7414217399	0.7772104017	0.8042514364	0.821730685	0.8300146821	0.835353052	0.8375937486	0.8448770434	0.8520202225	0.8594584653	0.867990263	0.8720315029	0.8777988555	0.8849709335	0.8902936358	0.8968119046	0.902371841	0.9045874249	0.9095711069];

% Metric - ultimate results

ys = [1252.9348330807	722.5629787636	506.4717701799	311.5760062653	253.9999749089	193.4363501223	142.7677551349	131.117463301	126.8930285029	124.6672988281	121.9576326217	104.2127969746	85.1968843183	67.5791702999	63.9347368166	63.5922990046	64.091441278	63.1297523968	76.037142466	82.0790907163	95.6554821153];
ys1 = [2532.1852353559	1436.5143007022	1002.369862816	594.5616586634	475.76063677	352.5937740702	247.8509058305	223.8691810654	215.6158602446	211.6252729637	203.9270581332	162.929184707	118.9078984092	79.1246690145	67.2939313002	61.7134717869	60.7791505327	60.2425934407	69.334368368	78.9861142252	97.7844499976];
ys2 = [235.9422235648	166.6348212222	115.414164537	107.3936408056	99.4873672315	95.5992089074	93.2114134167	92.8654875926	91.4355356019	90.6365078148	90.3417406389	90.2219810278	90.051674213	89.9464791481	89.740518	94.9545845278	92.738241963	92.2544945185	97.3080723796	97.2168687222	97.3234586296];
ys3 = [43.4110922948	43.8876754509	36.6113920029	37.525778026	37.5265884364	33.9981858468	32.8045865462	32.3450758439	32.0574207688	31.4929696705	33.9840592052	38.4935483873	43.4426469538	46.8162791879	51.8700787543	56.0455614451	59.1033534277	57.4850255173	77.3983715751	81.0459094046	92.5936173786];

st_v = 3;

xs = xs(st_v:end);
ysr = ysr(st_v:end);
ysr1 = ysr1(st_v:end);
ysr2 = ysr2(st_v:end);
ysr3 = ysr3(st_v:end);

ys = ys(st_v:end);
ys1 = ys1(st_v:end);
ys2 = ys2(st_v:end);
ys3 = ys3(st_v:end);

%Find the max value - all paths
v_max = max( [ys ys1 ys2 ys3 ] );
vr_max = max( [ysr ysr1 ysr2 ysr3 ] );
step = 50;
step_r = 0.10;
st_r = 0.35;

%Find optimal values
c = find(ys==min(min(ys)));
min_ys = min(ys);
v_max = max(ys);

%Plot
[AX,H1,H2] = plotyy(xs , ys , xs , ysr , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
xlabel('Threshold [m]');
title('Simple Segmentation - Overall - Metric A');
hold('on', AX(1));
hold('on', AX(2));
plot(xs(c),min_ys,'r*','Parent' , AX(1));

%Find optimal values
c = find(ys1==min(min(ys1)));
min_ys1 = min(ys1);
v_max = max(ys1);
step = 100;

figure;
[AX,H1,H2] = plotyy(xs , ys1 , xs , ysr1 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
xlabel('Threshold [m]');
title('Simple Segmentation - Scene 1 - Metric A');
hold on
plot(xs(c),min_ys1,'r*');


% Find optimal values
c = find(ys2==min(min(ys2)));
min_ys2 = min(ys2);

v_max = max(ys2);
step = 10;
step_r = 0.10;

figure;
[AX,H1,H2] = plotyy(xs , ys2 , xs , ysr2 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
xlabel('Threshold [m]');
title('Simple Segmentation - Scene 2 - Metric A');
hold on
plot(xs(c),min_ys2,'r*');


% Find optimal values
c = find(ys3==min(min(ys3)));
min_ys3 = min(ys3);

v_max = max(ys3);
step = 10;
step_r = 0.10;

figure;
[AX,H1,H2] = plotyy(xs , ys3 , xs , ysr3 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
xlabel('Threshold [m]');
title('Simple Segmentation - Scene 3 - Metric A');
hold on
plot(xs(c),min_ys3,'r*');


%%  Simple Segmentation Metric 1  Small clusters removed 

clear all, close all, clc;

xs = linspace(0.5,3.2,21);

%ratios -- No small CLusters
ysr = [0.2397510196	0.3280651961	0.4258516597	0.5317902653	0.6090198639	0.6613588708	0.6867737093	0.7108174937	0.7279312376	0.746730489	0.7655617612	0.7886916805	0.8129918258	0.8360560058	0.8568192076	0.8777924175	0.8951966125	0.9054630219	0.915999391	0.9260803045	0.9434885213];
ysr1 = [0.1097316683	0.1930721356	0.3220164576	0.4579046513	0.5539697458	0.6213452954	0.6557170557	0.6924802809	0.7205414383	0.7401567433	0.7615583245	0.7913653414	0.8204475448	0.8488193462	0.8717260339	0.8937832155	0.9119286077	0.9225016513	0.9361349564	0.9498870581	0.9802192639];
ysr2 = [0.2411258704	0.3431615741	0.4492457407	0.5436432407	0.6007829722	0.6327096389	0.6461919907	0.6512056204	0.6587381852	0.6654851019	0.6693249352	0.6820573704	0.6875760648	0.7230078981	0.740891213	0.7673511481	0.7875727593	0.7967475	0.8147679907	0.8231925093	0.8318818333];
ysr3 = [0.3945183844	0.4844863671	0.5424914798	0.6162834364	0.6773010289	0.7180632746	0.7365113497	0.7513127283	0.758349841	0.7799370173	0.8003796705	0.8187849855	0.8432394855	0.8561078439	0.8752114162	0.8931781329	0.9088181792	0.9190593295	0.9235629827	0.9297788382	0.934481948];

% Metric - No centers

ys = [1842.2482408858	938.9078283679	402.3821579723	279.0430279608	196.6082744879	137.6397571765	90.4291579689	80.0543500588	73.0698951303	67.20347091	59.3529827151	48.7975518858	43.982999654	39.4202570473	38.4648484475	35.1046599216	35.147621752	35.7528973368	35.7775415629	35.1022951234	41.989168654];
ys1 = [3708.4529092518	1850.0659913826	772.4138767724	527.603198414	357.4879097216	235.1611809927	137.7960352228	118.4500880024	106.9679802663	95.8941463293	79.2797589467	57.5789905666	50.712613109	42.3953982978	38.8164471429	33.8597235811	34.5211109613	34.2558018232	35.1203253438	34.8931216416	48.3709408789];
ys2 = [438.5706585741	330.5404874722	145.2589756481	81.39706625	69.7525466204	65.2482441296	64.11072525	63.6811960833	62.6353293148	61.564675537	61.2510139074	59.3229266481	58.8432561019	57.066376037	62.4842015093	66.0526632593	65.6414700648	71.722929	68.4907679074	62.4889688426	62.9515925741];
ys3 = [52.8108156098	41.2065321012	40.955001401	44.0442229538	44.1722318815	43.8303218382	42.1050840607	39.334294737	35.8647620289	34.7171730838	34.9751042399	35.0302841127	31.3117914075	30.3609674884	30.5477952919	26.9305973439	26.3771400723	26.3122529075	26.3509573237	26.803531789	27.8284353815];

st_v = 3;

xs = xs(st_v:end);
ysr = ysr(st_v:end);
ysr1 = ysr1(st_v:end);
ysr2 = ysr2(st_v:end);
ysr3 = ysr3(st_v:end);

ys = ys(st_v:end);
ys1 = ys1(st_v:end);
ys2 = ys2(st_v:end);
ys3 = ys3(st_v:end);

%Find the max value - all paths
v_max = max( [ys ys1 ys2 ys3 ] );
vr_max = max( [ysr ysr1 ysr2 ysr3 ] );
step = 50;
step_r = 0.10;
st_r = 0.35;

%Find optimal values
c = find(ys==min(min(ys)));
min_ys = min(ys);
v_max = max(ys);

%Plot
[AX,H1,H2] = plotyy(xs , ys , xs , ysr , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
xlabel('Threshold [m]');
title('Simple Segmentation - Overall - Metric A - Small clusters removed');
hold('on', AX(1));
hold('on', AX(2));
plot(xs(c),min_ys,'r*','Parent' , AX(1));

%Find optimal values
c = find(ys1==min(min(ys1)));
min_ys1 = min(ys1);
v_max = max(ys1);
step = 100;

figure;
[AX,H1,H2] = plotyy(xs , ys1 , xs , ysr1 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
xlabel('Threshold [m]');
title('Simple Segmentation - Scene 1 - Metric A - Small clusters removed');
hold on
plot(xs(c),min_ys1,'r*');


% Find optimal values
c = find(ys2==min(min(ys2)));
min_ys2 = min(ys2);

v_max = max(ys2);

step = 10;
step_r = 0.10;

figure;
[AX,H1,H2] = plotyy(xs , ys2 , xs , ysr2 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
xlabel('Threshold [m]');
title('Simple Segmentation - Scene 2 - Metric A - Small clusters removed');
hold on
plot(xs(c),min_ys2,'r*');


% Find optimal values
c = find(ys3==min(min(ys3)));
min_ys3 = min(ys3);

v_max = max(ys3);
step = 5;
step_r = 0.10;

figure;
[AX,H1,H2] = plotyy(xs , ys3 , xs , ysr3 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
xlabel('Threshold [m]');
title('Simple Segmentation - Scene 3 - Metric A - Small clusters removed');
hold on
plot(xs(c),min_ys3,'r*');


%% Dietmayer Segmentation -- METRIC
% Ratios + Metrics

clear all, close all, clc;

xd = linspace(0.5,3,21);

%Ratios

ydr = [0.5561711915	0.6593033518	0.7210270473	0.7537444637	0.7822403702	0.8060756851	0.8249652953	0.8445960035	0.8571253068	0.871839654	0.8886672987	0.9078001107	0.9284550346	0.95071706	0.9721474614	0.9889777324	1.004362083	1.0157879792	1.0361289619	1.0518823622	1.0645621765];
ydr1 = [0.3764877022	0.5284536707	0.6258138136	0.6779579225	0.7240162179	0.7645056828	0.7934927893	0.8224463995	0.8403951065	0.8636813995	0.8886158765	0.9147041743	0.9388506102	0.9700941235	1.0021342591	1.02510346	1.0448681598	1.0597965738	1.0816448814	1.0948792591	1.1062505351];
ydr2 = [0.5397551111	0.6537073889	0.705723213	0.7221526204	0.7440065278	0.755896787	0.7638287407	0.775153537	0.7878505185	0.794022037	0.8016177222	0.8100051481	0.8339865	0.8598724074	0.8768398333	0.8957380926	0.913237463	0.9206890278	0.9360418056	0.9505513241	0.9671018241];
ydr3 = [0.7757729769	0.8172376936	0.8394544451	0.8540674711	0.8636734046	0.8713581474	0.8816152746	0.8927103757	0.8987185145	0.905867578	0.9159002225	0.9300847283	0.9455337312	0.9559439249	0.9661031734	0.974960263	0.9844558671	0.9929415549	1.0130403439	1.0321888179	1.0452223671];

% Metric - ultimate results

yd = [558.2522632238	342.7328682145	276.8577905375	250.4784041488	206.8843483403	175.4270484591	180.8087777751	176.7731452964	169.8555638593	165.0075764948	165.9630041246	164.8547689619	164.6589565075	170.9134217601	201.7463472814	232.7704195617	285.0237957393	316.9139464498	375.0402409031	399.0828144844	440.2116562434];
yd1 = [1071.3457621404	624.217661276	484.4407933196	427.368546724	333.2889470218	265.6363767143	269.7235460339	253.9551968305	237.0663495157	223.7742356586	219.3712310048	205.7012330387	181.2493243584	180.3458484673	238.8274083051	295.8575238475	401.9341151695	464.2913372349	576.774668707	600.1999495811	678.0947467264];
yd2 = [121.439379713	116.4282834167	111.3550108704	109.5114715185	110.595542713	112.1201682407	131.1866816481	153.2116266204	154.2103999259	156.4983884537	157.68234275	160.5483921019	178.2210063704	184.1353873611	189.4158136574	187.2099925185	190.2659333241	188.8419922407	192.2212792963	203.7396340556	213.6419459722];
yd3 = [82.148726711	77.3793295549	80.7379063035	83.3362071561	86.0580239249	87.5099689624	90.1653878642	91.9998988613	94.5134344624	97.5172934711	104.7974368613	117.4429164682	140.6227619769	155.5273971503	161.3336866705	171.6882000549	175.0523715087	180.974376104	191.3065101821	219.9952037659	226.9856803468];

st_v = 1;

xd = xd(st_v:end);
ydr = ydr(st_v:end);
ydr1 = ydr1(st_v:end);
ydr2 = ydr2(st_v:end);
ydr3 = ydr3(st_v:end);

yd = yd(st_v:end);
yd1 = yd1(st_v:end);
yd2 = yd2(st_v:end);
yd3 = yd3(st_v:end);

%Find the max value - all paths
v_max = max( [yd yd1 yd2 yd3 ] );
vr_max = max( [ydr ydr1 ydr2 ydr3 ] );
step = 75;
step_r = 0.10;
st_r = 0.50;

%Find optimal values
c = find(yd==min(min(yd)));
min_yd = min(yd);
v_max = max(yd);

%Plot
[AX,H1,H2] = plotyy(xd , yd , xd , ydr , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters DS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Dietmayer Segmentation - Overall - Metric A');
xlabel('C0 [m]');
hold on
plot(xd(c),min_yd,'r*');
axes(AX(2));
lh=line([xd(1) xd(end)],[1 1] , 'Color','r');


% Find optimal values
c = find(yd1==min(min(yd1)));
min_yd1 = min(yd1);
v_max = max(yd1);
step = 100;

figure;
[AX,H1,H2] = plotyy(xd , yd1 , xd , ydr1 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters DS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Dietmayer Segmentation - Scene 1 - Metric A');
xlabel('C0 [m]');
hold on
plot(xd(c),min_yd1,'r*');
axes(AX(2));
lh=line([xd(1) xd(end)],[1 1] , 'Color','r');

% Find optimal values
c = find(yd2==min(min(yd2)));
min_yd2 = min(yd2);

v_max = max(yd2);
step = 20;
step_r = 0.10;

figure;
[AX,H1,H2] = plotyy(xd , yd2 , xd , ydr2 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters DS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Dietmayer Segmentation - Scene 2 - Metric A');
xlabel('C0 [m]');
hold on
plot(xd(c),min_yd2,'r*');



% Find optimal values
c = find(yd3==min(min(yd3)));
min_yd3 = min(yd3);

v_max = max(yd3);
step = 20;
step_r = 0.10;

figure;
[AX,H1,H2] = plotyy(xd , yd3 , xd , ydr3 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters DS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Dietmayer Segmentation - Scene 3 - Metric A');
xlabel('C0 [m]');
hold on
plot(xd(c),min_yd3,'r*');
axes(AX(2));
lh=line([xd(1) xd(end)],[1 1] , 'Color','r');


%%  Dietmayer Segmentation Metric 1  Small clusters removed 

clear all, close all, clc;

xd = linspace(0.5,3,21);

%Ratios - NO small clusters
ydr = [0.4538301269	0.5736247347	0.6477034118	0.691183143	0.7300153991	0.7633615479	0.789866925	0.8170682145	0.8362844464	0.859158639	0.8825938939	0.9076950438	0.9336391153	0.9656983691	0.9984231869	1.0258173333	1.046764767	1.0643178985	1.0841373322	1.0973925536	1.1107888028];
ydr1 = [0.306456385	0.4503854746	0.5513598523	0.6105289855	0.6687438257	0.7200259782	0.7576199225	0.79263146	0.819400368	0.8556813705	0.8903337191	0.9246656683	0.9579666828	0.9986773172	1.0402815303	1.0749333535	1.1014927893	1.1239663148	1.1495236005	1.1673202107	1.1851376102];
ydr2 = [0.4475173704	0.5621416296	0.6177684352	0.6367129074	0.6532843148	0.6624931204	0.6683963333	0.6773324352	0.6870102778	0.6943139907	0.7005918796	0.7083883241	0.72988225	0.7583451111	0.7775301944	0.7944829537	0.8084111111	0.8142698333	0.8288315463	0.843898037	0.8557514444];
ydr3 = [0.6317120145	0.7243125665	0.7720469595	0.8044575723	0.8271024422	0.8465736301	0.8662739653	0.8898538902	0.9030322341	0.9147636503	0.9301651936	0.9496495462	0.9682011272	0.9910563064	1.0174085838	1.0393988266	1.0558385289	1.0711687514	1.0857803844	1.093049448	1.1016500087];


% Metric - No centers
yd = [418.0524013114	266.2102438005	198.1636485825	161.04264691	101.1822821845	85.1645602226	82.0029795063	75.1336563806	78.4462378535	75.7083843899	67.2239555386	65.9109291845	69.4465559573	78.7675759273	87.8834566805	95.4104191592	103.7714428973	111.1823896424	119.1187196955	121.9462771776	125.4219191476];
yd1 = [791.1702358232	498.8260109298	353.7800900024	276.9545527143	154.7018780823	120.1591385375	112.1189882833	106.8514198136	113.6021153269	105.9473518717	85.6000808111	77.4903288717	79.2848592228	84.2921547676	87.5547039419	96.2431888547	109.9356653245	118.3839267288	131.2757870872	137.2158739758	143.1175293002];
yd2 = [156.0822587222	78.8078285926	68.2840021204	66.3283053056	65.1814745	63.1829549815	65.2402368148	65.0371724537	64.6534011852	66.8477802222	67.3242776944	68.5645071852	69.0727616111	70.9367313056	78.2208330833	81.920384037	82.9443499722	87.1768587685	86.8569541481	83.0639209444	84.3101349259];
yd3 = [54.4544526012	47.045934604	52.953855263	52.2493862052	48.5363114451	50.2548854306	51.2875592341	40.4255175549	40.7879978642	42.3796320202	45.2581331965	51.2609913382	57.8198234162	74.6175185809	91.2919486734	98.627153	102.914512763	110.0793911387	114.6776845694	115.8565401705	117.1322824509];

st_v = 1;

xd = xd(st_v:end);
ydr = ydr(st_v:end);
ydr1 = ydr1(st_v:end);
ydr2 = ydr2(st_v:end);
ydr3 = ydr3(st_v:end);

yd = yd(st_v:end);
yd1 = yd1(st_v:end);
yd2 = yd2(st_v:end);
yd3 = yd3(st_v:end);

%Find the max value - all paths
v_max = max( [yd yd1 yd2 yd3 ] );
vr_max = max( [ydr ydr1 ydr2 ydr3 ] );
step = 100;
step_r = 0.10;
st_r = 0.50;

%Find optimal values
c = find(yd==min(min(yd)));
min_yd = min(yd);
v_max = max(yd);

%Plot
[AX,H1,H2] = plotyy(xd , yd , xd , ydr , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters DS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Dietmayer Segmentation - Overall - Metric A - Small clusters removed');
xlabel('C0 [m]');
hold on
plot(xd(c),min_yd,'r*');
axes(AX(2));
lh=line([xd(1) xd(end)],[1 1] , 'Color','r');


% Find optimal values
c = find(yd1==min(min(yd1)));
min_yd1 = min(yd1);
v_max = max(yd1);

figure;
[AX,H1,H2] = plotyy(xd , yd1 , xd , ydr1 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters DS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Dietmayer Segmentation - Scene 1 - Metric A - Small clusters removed');
xlabel('C0 [m]');
hold on
plot(xd(c),min_yd1,'r*');
axes(AX(2));
lh=line([xd(1) xd(end)],[1 1] , 'Color','r');

% Find optimal values
c = find(yd2==min(min(yd2)));
min_yd2 = min(yd2);

v_max = max(yd2);
step = 20;
step_r = 0.10;

figure;
[AX,H1,H2] = plotyy(xd , yd2 , xd , ydr2 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters DS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Dietmayer Segmentation - Scene 2 - Metric A - Small clusters removed');
xlabel('C0 [m]');
hold on
plot(xd(c),min_yd2,'r*');



% Find optimal values
c = find(yd3==min(min(yd3)));
min_yd3 = min(yd3);

v_max = max(yd3);
step = 20;
step_r = 0.10;

figure;
[AX,H1,H2] = plotyy(xd , yd3 , xd , ydr3 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters DS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Dietmayer Segmentation - Scene 3 - Metric A - Small clusters removed');
xlabel('C0 [m]');
hold on
plot(xd(c),min_yd3,'r*');
axes(AX(2));
lh=line([xd(1) xd(end)],[1 1] , 'Color','r');


%% ABD -- METRIC
% Ratios + Metrics

clear all, close all, clc;

x_abd = linspace(4,22,21);

%ratios -- No small CLusters
y_abdr = [1.1410030219	1.1182093564	1.1058006413	1.113231331	1.0863426378	1.0623786424	1.0453287555	1.0206398316	0.9944944383	0.9672135848	0.9391960484	0.8974375513	0.8667164325	0.8398331003	0.8172093875	0.7890722526	0.7574697497	0.7250825617	0.6899668604	0.6536217624	0.617564015];
y_abdr1 = [1.0792750363	1.0303662833	1.0115853002	0.993639431	0.9623893293	0.943204586	0.9248408305	0.8933028571	0.8566100799	0.8144288692	0.775379339	0.7274110073	0.7035803317	0.6726982397	0.6412820751	0.6060208039	0.5663411017	0.5177264286	0.4736270363	0.4256539685	0.3812788063];
y_abdr2 = [0.9351915185	0.8763565741	0.8775914167	0.8857836019	0.8685114444	0.8522283241	0.8456082963	0.8412333426	0.8432390093	0.8420881759	0.8201975	0.7903408056	0.7719184074	0.7558051944	0.7388819167	0.7142382778	0.6940703704	0.6671835093	0.6444106852	0.6182866204	0.5841430648];
y_abdr3 = [1.2789258555	1.2985541243	1.2894929306	1.3269764451	1.3022920173	1.2702258092	1.2514886474	1.2286342572	1.2062916243	1.1886402659	1.1718785462	1.1338173526	1.0910326069	1.0655605896	1.0516525867	1.0309286618	1.0053982601	0.9906640087	0.9624189249	0.9367630751	0.9100358468];

%Metric - ultimate centers

y_abd = [551.6702035952	443.6959693218	515.7918190657	540.1595391569	571.9544729366	570.1378796563	557.1993548558	538.109132316	543.4898501303	575.9846704671	590.4024486932	575.2560302411	546.7750701857	598.0516290807	751.6491274014	857.6592795502	854.7552615859	847.009490316	897.1110474175	922.7456977255	1020.0325954222];
y_abd1 = [661.6139865981	400.5928515521	531.1875970533	516.3085034528	612.1833458329	640.109259339	646.8915773293	614.5712581913	617.4837363656	696.6335742034	734.039539063	741.4181645908	742.2587505424	887.9024534019	1192.8638923559	1434.0574699419	1459.3884043027	1469.0447168596	1575.5676357482	1637.740779201	1848.081737017];
y_abd2 = [195.9838531296	211.219472787	293.0407636296	358.3544862407	385.0441033426	386.275527037	338.3480679537	330.7411183426	368.0957679537	391.1661463704	391.4932036389	350.2496643056	339.2463199167	352.6277380556	428.5052200093	390.3126156389	357.3800298148	279.578695787	312.1464072222	308.201173463	304.0485616481];
y_abd3 = [531.4602136243	567.7105625723	566.9440088873	625.3775260376	582.2775810578	544.0076896965	518.4509476416	511.5683448092	509.9149537717	489.6621368237	481.0385184711	447.1511921908	378.2153737746	328.6799232601	325.8625495751	315.5234040318	288.2900508613	281.6394823584	269.8671461474	261.1209577572	255.1250236561];

st_v = 19;

x_abd = x_abd(1:st_v);
y_abdr = y_abdr(1:st_v);
y_abdr1 = y_abdr1(1:st_v);
y_abdr2 = y_abdr2(1:st_v);
y_abdr3 = y_abdr3(1:st_v);

y_abd = y_abd(1:st_v);
y_abd1 = y_abd1(1:st_v);
y_abd2 = y_abd2(1:st_v);
y_abd3 = y_abd3(1:st_v);

%Find the max value - all paths
v_max = max( [y_abd y_abd1 y_abd2 y_abd3 ] );
vr_max = max( [y_abdr y_abdr1 y_abdr2 y_abdr3 ] );
step = 200;
step_r = 0.10;
st_r = 0.40;

%Find optimal values
c = find(y_abd==min(min(y_abd)));
min_y_abd = min(y_abd);
v_max = max(y_abd);

%Plot
[AX,H1,H2] = plotyy(x_abd , y_abd , x_abd , y_abdr , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters ABD');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('ABD Segmentation - Overall - Metric A');
xlabel('Lamda [degrees]');
hold on
plot(x_abd(c),min_y_abd,'r*');
axes(AX(2));
lh=line([x_abd(1) x_abd(end)],[1 1] , 'Color','r');

%Find optimal values
c = find(y_abd1==min(min(y_abd1)));
min_y_abd1 = min(y_abd1);
v_max = max(y_abd1);

%Plot
figure;
[AX,H1,H2] = plotyy(x_abd , y_abd1 , x_abd , y_abdr1 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters ABD');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('ABD Segmentation - Scene1 - Metric A');
xlabel('Lamda [degrees]');
hold on
plot(x_abd(c),min_y_abd1,'r*');
axes(AX(2));
lh=line([x_abd(1) x_abd(end)],[1 1] , 'Color','r');

%Find optimal values
c = find(y_abd2==min(min(y_abd2)));
min_y_abd2 = min(y_abd2);

v_max = max(y_abd2);
step = 30;
%step_r = 0.20;

%Plot
figure;
[AX,H1,H2] = plotyy(x_abd , y_abd2 , x_abd , y_abdr2 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters ABD');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('ABD Segmentation - Scene2 - Metric A');
xlabel('Lamda [degrees]');
hold on
plot(x_abd(c),min_y_abd2,'r*');


%Find optimal values
c = find(y_abd3==min(min(y_abd3)));
min_y_abd3 = min(y_abd3);

v_max = max(y_abd3);
step = 50;

%Plot
figure;
[AX,H1,H2] = plotyy(x_abd , y_abd3 , x_abd , y_abdr3 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters ABD');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('ABD Segmentation - Scene3 - Metric A');
xlabel('Lamda [degrees]');
hold on
plot(x_abd(c),min_y_abd3,'r*');
axes(AX(2));
lh=line([x_abd(1) x_abd(end)],[1 1] , 'Color','r');


%%  ABD Metric 1  Small clusters removed 

% Ratios + Metrics

clear all, close all, clc;

x_abd = linspace(4,22,21);

y_abdr = [0.9630849873	0.9355387739	0.9348255006	0.9361147116	0.9248887705	0.9054220035	0.8933180334	0.8837323506	0.8663813541	0.8459043818	0.8200763749	0.787197489	0.7623201776	0.7304469135	0.7051629677	0.6800778178	0.6515734867	0.6280217382	0.5969003333	0.5624722376	0.5237963518];
y_abdr1 = [1.0245784068	0.9595074746	0.9313858935	0.9017432591	0.8595290508	0.830546385	0.8069457458	0.7782586465	0.741890937	0.69946154	0.6583996199	0.6094133826	0.5730096489	0.5276887627	0.4869915109	0.4517441743	0.4205815666	0.3835837433	0.350403937	0.3147413123	0.2826801961];
y_abdr2 = [0.8788566574	0.8040205556	0.7791170648	0.765538537	0.7379148241	0.7169278056	0.7069514167	0.6843401111	0.6747259907	0.6659415093	0.6498239907	0.6304035926	0.6114374074	0.5947253889	0.5771217407	0.5571705741	0.5399203611	0.5170739907	0.4953268611	0.4741091759	0.4473195093];
y_abdr3 = [0.9159748064	0.9479806647	0.9875337919	1.0303853382	1.0612666618	1.0536329971	1.0545878295	1.0718681936	1.0748013584	1.0768780347	1.066202841	1.0483494451	1.0353854595	1.0148314249	1.0055481243	0.9909904682	0.9621463208	0.9544241908	0.9228337052	0.8857557139	0.8354740145];

y_abd = [118.4782210185	115.2603877082	123.8003303818	131.6126236424	152.2857523645	168.9235194268	167.4894008224	173.6814247543	201.5008935779	234.6199768097	247.1992323045	247.2297074268	256.3780052076	265.5235064867	285.295287361	306.1900434337	332.4853729988	368.0867289585	377.4830520473	398.4679471096	471.9523025444];
y_abd1 = [154.5126733293	106.042105753	103.1794073317	100.3964497724	121.272965017	142.5035994334	136.255330615	141.0115946344	197.1018434552	263.4791244358	285.6513293148	292.6422460678	310.435149862	329.2821259007	368.4730761501	398.138602586	450.0437365012	510.5356495472	536.4190280654	576.0526069976	744.7187572542];
y_abd2 = [106.9288084074	106.7232110093	86.4539159352	86.9604754537	90.8856418981	100.1905115463	105.2994914352	110.3961575278	115.5361406667	112.0383968426	116.536902963	118.3485905	119.9711733704	123.5485455833	123.1105859259	121.4961751574	137.1454436389	146.1807335648	141.5717263889	178.4157421296	187.7345656667];
y_abd3 = [79.0710180058	128.9284961792	160.071584659	182.8112126965	208.469258448	221.9136691619	224.1835661676	232.4312764884	233.5847115405	238.4349556156	242.085982315	233.2521992457	234.4310025954	233.7344485665	236.6349433873	254.0865256416	253.1359748613	267.3192240434	261.4075175838	255.1823216879	235.0822730289];


st_v = 19;

x_abd = x_abd(1:st_v);
y_abdr = y_abdr(1:st_v);
y_abdr1 = y_abdr1(1:st_v);
y_abdr2 = y_abdr2(1:st_v);
y_abdr3 = y_abdr3(1:st_v);

y_abd = y_abd(1:st_v);
y_abd1 = y_abd1(1:st_v);
y_abd2 = y_abd2(1:st_v);
y_abd3 = y_abd3(1:st_v);

%Find the max value - all paths
v_max = max( [y_abd y_abd1 y_abd2 y_abd3 ] );
vr_max = max( [y_abdr y_abdr1 y_abdr2 y_abdr3 ] );
step = 50;
step_r = 0.20;
st_r = 0.10;

%Find optimal values
c = find(y_abd==min(min(y_abd)));
min_y_abd = min(y_abd);
v_max = max(y_abd);

%Plot
[AX,H1,H2] = plotyy(x_abd , y_abd , x_abd , y_abdr , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters ABD');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('ABD Segmentation - Overall - Metric A - Small clusters removed');
xlabel('Lamda [degrees]');
hold on
plot(x_abd(c),min_y_abd,'r*');

%Find optimal values
c = find(y_abd1==min(min(y_abd1)));
min_y_abd1 = min(y_abd1);
v_max = max(y_abd1);

%Plot
figure;
[AX,H1,H2] = plotyy(x_abd , y_abd1 , x_abd , y_abdr1 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters ABD');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('ABD Segmentation - Scene1 - Metric A - Small clusters removed');
xlabel('Lamda [degrees]');
hold on
plot(x_abd(c),min_y_abd1,'r*');
axes(AX(2));
lh=line([x_abd(1) x_abd(end)],[1 1] , 'Color','r');

%Find optimal values
c = find(y_abd2==min(min(y_abd2)));
min_y_abd2 = min(y_abd2);
v_max = max(y_abd2);
step = 30;

%Plot
figure;
[AX,H1,H2] = plotyy(x_abd , y_abd2 , x_abd , y_abdr2 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters ABD');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('ABD Segmentation - Scene2 - Metric A - Small clusters removed');
xlabel('Lamda [degrees]');
hold on
plot(x_abd(c),min_y_abd2,'r*');


%Find optimal values
c = find(y_abd3==min(min(y_abd3)));
min_y_abd3 = min(y_abd3);
v_max = max(y_abd3);
step = 25;

%Plot
figure;
[AX,H1,H2] = plotyy(x_abd , y_abd3 , x_abd , y_abdr3 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters ABD');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('ABD Segmentation - Scene3 - Metric A - Small clusters removed');
xlabel('Lamda [degrees]');
hold on
plot(x_abd(c),min_y_abd3,'r*');
axes(AX(2));
lh=line([x_abd(1) x_abd(end)],[1 1] , 'Color','r');


%% SNN SEGMENTATION - Metric 1
% Ratios + Metrics

clear ll, close all, clc;

x_nn = linspace(0.5,3,21);

%Ratio results -- Ultimate centers
y_nnr = [0.3721719735	0.4651955675	0.5652047347	0.6685645848	0.7461939689	0.7951843299	0.8270252999	0.8480950254	0.8638526078	0.8759672468	0.8864453276	0.8997043633	0.9171613587	0.9322308155	0.9483712388	0.9667146828	0.9849866517	1.0052971107	1.0260628201	1.0377019354	1.0502509123];
y_nnr1 = [0.1522174334	0.2459748886	0.3839097845	0.5339905593	0.6433783923	0.7127435036	0.7557654068	0.7832103317	0.8077693898	0.8255758838	0.8360735981	0.8500551646	0.870764431	0.8883477627	0.9052608402	0.9227145206	0.9421993196	0.9700092421	0.9962762349	1.0081099831	1.0212628717];
y_nnr2 = [0.3189895278	0.4358631389	0.5575906204	0.6644523796	0.7318428148	0.7735024907	0.8157852593	0.8380898056	0.8504882593	0.8594516852	0.8705860741	0.8793024444	0.8984193333	0.9182209722	0.9576019074	0.9976955556	1.0229760278	1.0348946481	1.0485025463	1.0698718241	1.0944986574];
y_nnr3 = [0.6513191676	0.7360222803	0.7839825925	0.8304813208	0.8733984711	0.9003568728	0.915592526	0.9286671127	0.9349674017	0.9412716214	0.9515214075	0.9653359422	0.9783927746	0.9889844682	0.9969483555	1.0095647775	1.0242014364	1.0381796416	1.0546130202	1.0629826532	1.0710408092];


%Metric results -- Ultiamte centers
y_nn = [1356.1212434371	743.1878514844	541.0377616309	303.6610869919	215.6274269008	166.0268115779	108.0871594844	73.5185640588	64.2737878489	63.2819434256	65.3044215386	69.464511699	58.6696068005	55.9564881984	55.8833802042	58.7364444025	66.6961194717	83.3242554025	109.7819108558	123.2800647693	137.8505815433];
y_nn1 = [2733.9886644818	1467.2188631041	1065.9439477724	578.2016145036	397.5193463559	296.3188039201	179.6348707603	107.6268996199	88.6228304455	85.924838431	85.7057796707	91.6721653729	65.6942676489	57.3225808039	55.3398895182	54.3617946634	54.5582346416	72.7092069709	88.1089503511	90.7775121283	103.6173685472];
y_nn2 = [235.6895661389	174.1670498519	114.9089578611	104.9983428519	95.6700340833	89.9435434352	74.1495490093	65.7759150833	65.209948463	64.7764010278	63.2327224167	62.5773776296	61.1770340741	60.3234647037	56.1849024722	55.4774787685	62.795165787	66.3257748704	69.2126511204	111.9883633889	158.2457030741];
y_nn3 = [61.1714638324	56.5677323439	47.4991949566	37.968423711	35.9570098179	34.253401526	33.2780761734	35.2222158006	34.9175452514	35.7879634884	41.5991689277	45.1062732948	49.5020140954	52.9627607168	56.4379965289	64.9754577861	82.4020426879	101.3007030896	148.3148956532	165.6010416185	172.3466620116];

st_v = 3;

x_nn = x_nn(st_v:end);
y_nnr = y_nnr(st_v:end);
y_nnr1 = y_nnr1(st_v:end);
y_nnr2 = y_nnr2(st_v:end);
y_nnr3 = y_nnr3(st_v:end);

y_nn = y_nn(st_v:end);
y_nn1 = y_nn1(st_v:end);
y_nn2 = y_nn2(st_v:end);
y_nn3 = y_nn3(st_v:end);


%Find the max value - all paths
v_max = max( [y_nn y_nn1 y_nn2 y_nn3 ] );
vr_max = max( [y_nnr y_nnr1 y_nnr2 y_nnr3 ] );
step = 100;
step_r = 0.10;
st_r = 0.30;

%Find optimal values
c = find(y_nn==min(min(y_nn)));
min_y_nn = min(y_nn);
v_max = max(y_nn);
step = 50;

%Plot
[AX,H1,H2] = plotyy(x_nn , y_nn , x_nn , y_nnr , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SNN');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('SNN Segmentation - Overall - Metric A');
xlabel('Threshold [m]');
hold on
plot(x_nn(c),min_y_nn,'r*');
axes(AX(2));
lh=line([x_nn(1) x_nn(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_nn1==min(min(y_nn1)));
min_y_nn1 = min(y_nn1);
v_max = max(y_nn1);
step = 100;

figure;
[AX,H1,H2] = plotyy(x_nn , y_nn1 , x_nn , y_nnr1 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SNN');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('SNN Segmentation - Scene 1 - Metric A');
xlabel('Threshold [m]');
hold on
plot(x_nn(c),min_y_nn1,'r*');
axes(AX(2));
lh=line([x_nn(1) x_nn(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_nn2==min(min(y_nn2)));
min_y_nn2 = min(y_nn2);

v_max = max(y_nn2);
step = 15;

figure;
[AX,H1,H2] = plotyy(x_nn , y_nn2 , x_nn , y_nnr2 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SNN');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('SNN Segmentation - Scene 2 - Metric A');
xlabel('Threshold [m]');
hold on
plot(x_nn(c),min_y_nn2,'r*');
axes(AX(2));
lh=line([x_nn(1) x_nn(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_nn3==min(min(y_nn3)));
min_y_nn3 = min(y_nn3);
v_max = max(y_nn3);
step = 15;

figure;
[AX,H1,H2] = plotyy(x_nn , y_nn3 , x_nn , y_nnr3 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SNN');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('SNN Segmentation - Scene 3 - Metric A');
xlabel('Threshold [m]');
hold on
plot(x_nn(c),min_y_nn3,'r*');
axes(AX(2));
lh=line([x_nn(1) x_nn(end)],[1 1] , 'Color','r');



%% SNN Metric 1  Small clusters removed 

% Ratios + Metrics

clear all, close all, clc;

x_nn = linspace(0.5,3,21);

y_nnr = [0.2495988408	0.3405208074	0.4414688627	0.5538764464	0.6492687555	0.7133451638	0.7567475294	0.7849322722	0.8060869965	0.8239427578	0.8420403483	0.8593764348	0.8839244475	0.9053765779	0.928374173	0.9512475663	0.9710070173	0.9952512018	1.0180683506	1.0269880807	1.0362474648];
y_nnr1 = [0.112092184	0.1902247191	0.3142376828	0.4585844891	0.5731663341	0.6507009661	0.7016830291	0.7339209274	0.7634210484	0.7841153656	0.7969942421	0.8136669346	0.8393001259	0.8599187143	0.8802413632	0.9008676344	0.9238633317	0.9578288935	0.9917848499	1.0064452906	1.0187334262];
y_nnr2 = [0.250884713	0.3562449352	0.4764994815	0.5910140463	0.6695188889	0.7200119259	0.7706911944	0.7988591204	0.8081433241	0.8188423426	0.8332005833	0.8433763796	0.8670086759	0.8904368796	0.9430624907	0.9938224815	1.0243907315	1.0399818333	1.058495	1.0638962407	1.0822323796];
y_nnr3 = [0.4133311387	0.5150123642	0.5824028815	0.6560288092	0.733786922	0.7860389306	0.8181224509	0.8414744277	0.8563729884	0.8730744277	0.8985684884	0.9189314335	0.9424699624	0.9643002341	0.9812427052	0.9980938699	1.0106165578	1.0259578642	1.0368227081	1.0399883439	1.0427992775];

y_nn = [2321.1497710323	1212.0716448051	427.0982811338	268.4784363564	148.309911932	103.2563809839	55.9392306367	50.2399558524	42.9355406159	35.6111343679	32.7549887313	32.4405927889	30.670100669	29.2235787486	28.6471697082	28.278625233	25.6686738489	28.4459807255	31.3819553887	31.5706509954	34.2861439977];
y_nn1 = [4699.7978857579	2418.2547041986	825.3057858087	517.4528086077	268.0501526852	176.761462816	86.9539326998	75.4164907554	60.9531869952	45.7137256053	41.2491670363	40.6388229613	37.091291937	36.1389311622	35.1896401041	31.8896016707	27.5467666804	31.9532616949	37.4689274988	37.6610011162	37.9549010702];
y_nn2 = [436.5380379259	333.673644713	154.2500775556	79.4613775648	66.2671055741	60.2811197315	30.7247169907	29.9338701204	29.1374081667	31.5648686389	30.2006027778	31.5653142222	29.9950230833	29.4506969815	27.8032484074	30.6756984537	33.841847787	37.1564615741	38.5629173241	40.1423098611	59.2505831481];
y_nn3 = [70.1566952919	46.5033802977	36.9477220462	30.2918080607	30.9917144046	28.9318995347	26.7892176936	26.5265117601	25.7358595462	24.8152572514	23.4133067746	22.928052396	23.2162173902	20.8982339046	21.1012252775	23.2201940867	20.8757399624	21.5406657775	21.8748358208	21.625408922	22.114594578];

st_v = 3;

x_nn = x_nn(st_v:end);
y_nnr = y_nnr(st_v:end);
y_nnr1 = y_nnr1(st_v:end);
y_nnr2 = y_nnr2(st_v:end);
y_nnr3 = y_nnr3(st_v:end);

y_nn = y_nn(st_v:end);
y_nn1 = y_nn1(st_v:end);
y_nn2 = y_nn2(st_v:end);
y_nn3 = y_nn3(st_v:end);


%Find the max value - all paths
v_max = max( [y_nn y_nn1 y_nn2 y_nn3 ] );
vr_max = max( [y_nnr y_nnr1 y_nnr2 y_nnr3 ] );
step = 50;
step_r = 0.10;
st_r = 0.30;

%Find optimal values
c = find(y_nn==min(min(y_nn)));
min_y_nn = min(y_nn);
v_max = max(y_nn);

%Plot
[AX,H1,H2] = plotyy(x_nn , y_nn , x_nn , y_nnr , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SNN');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('SNN Segmentation - Overall - Metric A - Small clusters removed');
xlabel('Threshold [m]');
hold on
plot(x_nn(c),min_y_nn,'r*');
axes(AX(2));
lh=line([x_nn(1) x_nn(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_nn1==min(min(y_nn1)));
min_y_nn1 = min(y_nn1);
v_max = max(y_nn1);
step = 75;

figure;
[AX,H1,H2] = plotyy(x_nn , y_nn1 , x_nn , y_nnr1 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SNN');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('SNN Segmentation - Scene 1 - Metric A - Small clusters removed');
xlabel('Threshold [m]');
hold on
plot(x_nn(c),min_y_nn1,'r*');
axes(AX(2));
lh=line([x_nn(1) x_nn(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_nn2==min(min(y_nn2)));
min_y_nn2 = min(y_nn2);

v_max = max(y_nn2);
step = 15;

figure;
[AX,H1,H2] = plotyy(x_nn , y_nn2 , x_nn , y_nnr2 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SNN');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('SNN Segmentation - Scene 2 -Metric A - Small clusters removed');
xlabel('Threshold [m]');
hold on
plot(x_nn(c),min_y_nn2,'r*');
axes(AX(2));
lh=line([x_nn(1) x_nn(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_nn3==min(min(y_nn3)));
min_y_nn3 = min(y_nn3);
v_max = max(y_nn3);
step = 5.0;

figure;
[AX,H1,H2] = plotyy(x_nn , y_nn3 , x_nn , y_nnr3 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SNN');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('SNN Segmentation - Scene 3 - Metric A - Small clusters removed');
xlabel('Threshold [m]');
hold on
plot(x_nn(c),min_y_nn3,'r*');
axes(AX(2));
lh=line([x_nn(1) x_nn(end)],[1 1] , 'Color','r');


%% MS SEGMENTATION - Metric 1
% Ratios + Metrics

clear ll, close all, clc;

x_p = linspace(0.55, 0.97,21);

%Rations - ultimate centers
y_pr = [0.8774846344	0.8698991396	0.8643295006	0.8581383599	0.8508440565	0.8448296586	0.8391629804	0.8303822295	0.8230282526	0.8129415582	0.804683594	0.7964613576	0.7867366309	0.7755597728	0.7596366367	0.7409245386	0.7134355248	0.6755108997	0.6431821845	0.6044012618	0.5402888685];
y_pr1 = [0.9166214528	0.9088463656	0.9022981792	0.8942638814	0.8837576998	0.8744349201	0.8656780678	0.850352954	0.8386082494	0.8233401913	0.8103312446	0.7981647579	0.7833461913	0.7653374794	0.7439719128	0.7162405109	0.6739044165	0.6133758111	0.5650973341	0.5078646901	0.4203124165];
y_pr2 = [0.7760605556	0.7638733241	0.7556117222	0.744564787	0.7303704722	0.7238022593	0.7215535278	0.7190498148	0.714723963	0.7108231389	0.7091033056	0.7052024907	0.7013623796	0.6953129815	0.6873540648	0.6792330185	0.6652204259	0.6301402407	0.6072659167	0.5849974352	0.5495701389];
y_pr3 = [0.8624276821	0.8565048728	0.8529435347	0.8504681445	0.8491614335	0.8472689249	0.8442239335	0.8412955	0.8382372832	0.8324044306	0.8277766329	0.8229135347	0.817432211	0.812809659	0.8008968931	0.7896447341	0.7756713006	0.7638398671	0.7475983699	0.7256880173	0.680600711];

%Metric - ultimate centers
y_p = [60.3065224498	59.9665323702	63.1702851822	63.3949002399	64.5040986667	65.0079891176	64.5062189792	65.3247473806	71.694067158	88.6877945213	100.4768451915	109.400480925	118.5407313725	129.0452469504	129.7757076205	133.5486325536	143.6977353922	164.7965897059	184.3737381523	205.4188465836	256.3660985986];
y_p1 = [61.4769007893	62.3937097264	69.3681875182	70.666939586	70.7744343196	71.9193309201	71.3154655884	73.1762679709	86.1316139153	118.5201861525	144.3653909443	162.3933801937	181.323860276	202.7740637748	207.5584832978	217.7408619952	240.1815431259	282.8389596441	322.3442313511	364.6003718257	466.0844377167];
y_p2 = [93.0965686481	93.7904519815	95.4968135	96.0775677037	96.022346463	94.3037880741	91.9373226759	90.7527301667	90.8025021852	97.0862231019	97.3990082778	98.205651787	99.3737446759	100.5378333519	98.7708008611	98.2198392315	99.8625798241	102.2488178426	107.5521359722	111.831930787	125.7686567685];
y_p3 = [48.6744668324	46.5115972081	45.6818495665	44.5131651647	47.1814992832	47.6139704711	47.8161090116	48.0157874249	48.4963278699	50.457135474	49.0503625058	49.640290604	49.5829346272	49.9377305347	46.6087815376	44.080768159	42.213296685	43.4198861705	43.6654125578	44.6252540434	46.8020802803];

st_v = 19;

x_p = x_p(1:st_v);
y_pr = y_pr(1:st_v);
y_pr1 = y_pr1(1:st_v);
y_pr2 = y_pr2(1:st_v);
y_pr3 = y_pr3(1:st_v);

y_p = y_p(1:st_v);
y_p1 = y_p1(1:st_v);
y_p2 = y_p2(1:st_v);
y_p3 = y_p3(1:st_v);

%Find the max value - all paths
v_max = max( [y_p y_p1 y_p2 y_p3 ] );
v_min = min( [y_p y_p1 y_p2 y_p3 ] );
vr_max = max( [y_pr y_pr1 y_pr2 y_pr3 ] );
vr_min = 0;
step = 15;
step_r = 0.1;
st_r = 0.50;

%Find optimal values
c = find(y_p==min(min(y_p)));
min_y_p = min(y_p);
v_max = max(y_p); 

%Plot
[AX,H1,H2] = plotyy(x_p , y_p , x_p , y_pr , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters MS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Multivariable Segmentation - Overall - Metric A');
xlabel('Cosine distance threshold');
hold on
plot(x_p(c),min_y_p,'r*');
axes(AX(2));
lh=line([x_p(1) x_p(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_p1==min(min(y_p1)));
min_y_p1 = min(y_p1);
v_max = max(y_p1);
step = 30;

%Plot
figure;
[AX,H1,H2] = plotyy(x_p , y_p1 , x_p , y_pr1 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters MS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Multivariable Segmentation - Scene 1 - Metric A');
xlabel('Cosine distance threshold');
hold on
plot(x_p(c),min_y_p1,'r*');
axes(AX(2));
lh=line([x_p(1) x_p(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_p2==min(min(y_p2)));
min_y_p2 = min(y_p2);
v_max = max(y_p2);
step = 10;

%Plot
figure;
[AX,H1,H2] = plotyy(x_p , y_p2 , x_p , y_pr2 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters MS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Multivariable Segmentation - Scene 2 - Metric A');
xlabel('Cosine distance threshold');
hold on
plot(x_p(c),min_y_p2,'r*');
axes(AX(2));
lh=line([x_p(1) x_p(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_p3==min(min(y_p3)));
min_y_p3 = min(y_p3);
v_max = max(y_p3);
step = 5;

%Plot
figure;
[AX,H1,H2] = plotyy(x_p , y_p3 , x_p , y_pr3 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters MS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Multivariable Segmentation - Scene 3 - Metric A');
xlabel('Cosine distance threshold');
hold on
plot(x_p(c),min_y_p3,'r*');
axes(AX(2));
lh=line([x_p(1) x_p(end)],[1 1] , 'Color','r');


%% MS Metric 1  Small clusters removed 

% Ratios + Metrics

clear all, close all, clc;

x_p = linspace(0.55, 0.97,21);

y_pr = [0.8517919273	0.8375220946	0.827473714	0.8165620773	0.8054399112	0.7968691834	0.7876188604	0.7747467151	0.7628153979	0.7469467116	0.7342202849	0.7215773206	0.7065898847	0.6913949446	0.6660146563	0.638623083	0.6016096067	0.5543945594	0.5144032088	0.4682835363	0.3983829469];
y_pr1 = [0.8962111937	0.8853124286	0.8765025206	0.8662607918	0.8525596877	0.8405381816	0.8294403002	0.8104956731	0.7952009588	0.7764038087	0.7595285666	0.7455541162	0.7266673947	0.7056029322	0.6805566295	0.6489575811	0.599032523	0.5312743269	0.4792083947	0.4200879879	0.3352853801];
y_pr2 = [0.7363310556	0.7219528796	0.7126523148	0.6988033333	0.6807883611	0.67179375	0.6690952593	0.6656163704	0.6599762778	0.6546031019	0.6526642037	0.6473450741	0.6422230463	0.6356656296	0.6260983519	0.6165136574	0.5986536111	0.5589702037	0.5324349444	0.5069154815	0.4698211759];
y_pr3 = [0.834811052	0.8165511908	0.8047910954	0.7939966879	0.7881043613	0.7837849364	0.7746948555	0.766139078	0.7562587168	0.7406095116	0.729468078	0.7161283786	0.7027159162	0.6918309769	0.6611161763	0.633188604	0.6056084017	0.5805635954	0.5507848006	0.5137532225	0.4514001908];

%Results NO centers
y_p = [34.245808684	33.5659920496	34.0611753437	34.4578666436	35.5304743529	36.8439189873	37.5181111845	39.5488994567	46.2111844637	60.5113974187	61.8234860611	65.7791847336	76.2186066067	84.1141826217	86.2701016574	89.4977402042	93.5266811003	103.9158372468	120.3146168039	137.0339009965	177.9499652353];
y_p1 = [35.7486520024	35.9505048039	36.57943523	37.6772998523	39.336836184	39.495802615	40.5629733172	41.8705112857	55.4273886077	83.9194744988	85.6127565956	87.1142946513	108.4209065642	125.228931908	128.4496033341	133.1683321889	145.9925666368	164.3057054818	197.7398847676	234.8882752688	320.5400955327];
y_p2 = [64.7074152685	63.0610537407	63.9310990463	63.9527300278	64.0761084537	62.5471685741	61.6524490278	60.6969820556	60.8062118519	61.2512489259	61.5018190093	62.1391466667	62.3136159537	62.8741225648	63.5048844815	64.1251231667	65.6548486481	69.1492514537	72.5509271296	76.6907287963	77.0795684352];
y_p3 = [22.943705211	21.5131873382	21.7317155376	21.4085280289	22.0768445289	25.6555406821	26.3503754971	30.1765913468	30.6546663439	32.3396060983	33.528027422	41.4489064538	42.1208872659	41.6677520751	43.028798948	45.2904805231	39.6010949104	42.6839750636	42.8054919942	39.0662883468	39.2340087081];

st_v = 19;

x_p = x_p(1:st_v);
y_pr = y_pr(1:st_v);
y_pr1 = y_pr1(1:st_v);
y_pr2 = y_pr2(1:st_v);
y_pr3 = y_pr3(1:st_v);

y_p = y_p(1:st_v);
y_p1 = y_p1(1:st_v);
y_p2 = y_p2(1:st_v);
y_p3 = y_p3(1:st_v);

%Find the max value - all paths
v_max = max( [y_p y_p1 y_p2 y_p3 ] );
v_min = min( [y_p y_p1 y_p2 y_p3 ] );
vr_max = max( [y_pr y_pr1 y_pr2 y_pr3 ] );
vr_min = 0;
step = 15;
step_r = 0.1;
st_r = 0.45;

%Find optimal values
c = find(y_p==min(min(y_p)));
min_y_p = min(y_p);
v_max = max(y_p); 

%Plot
[AX,H1,H2] = plotyy(x_p , y_p , x_p , y_pr , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters MS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Multivariable Segmentation - Overall - Metric A - Small clusters removed');
xlabel('Cosine distance threshold');
hold on
plot(x_p(c),min_y_p,'r*');
axes(AX(2));
lh=line([x_p(1) x_p(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_p1==min(min(y_p1)));
min_y_p1 = min(y_p1);
v_max = max(y_p1);
step = 30;

%Plot
figure;
[AX,H1,H2] = plotyy(x_p , y_p1 , x_p , y_pr1 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters MS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Multivariable Segmentation - Scene 1 - Metric A - Small clusters removed');
xlabel('Cosine distance threshold');
hold on
plot(x_p(c),min_y_p1,'r*');
axes(AX(2));
lh=line([x_p(1) x_p(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_p2==min(min(y_p2)));
min_y_p2 = min(y_p2);
v_max = max(y_p2);
step = 10;

%Plot
figure;
[AX,H1,H2] = plotyy(x_p , y_p2 , x_p , y_pr2 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters MS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Multivariable Segmentation - Scene 2 - Metric A - Small clusters removed');
xlabel('Cosine distance threshold');
hold on
plot(x_p(c),min_y_p2,'r*');
axes(AX(2));
lh=line([x_p(1) x_p(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_p3==min(min(y_p3)));
min_y_p3 = min(y_p3);
v_max = max(y_p3);
step = 5;

%Plot
figure;
[AX,H1,H2] = plotyy(x_p , y_p3 , x_p , y_pr3 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters MS');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Multivariable Segmentation - Scene 3 - Metric A - Small clusters removed');
xlabel('Cosine distance threshold');
hold on
plot(x_p(c),min_y_p3,'r*');
axes(AX(2));
lh=line([x_p(1) x_p(end)],[1 1] , 'Color','r');


%% SANTOS C0 SEGMENTATION
% Ratios + Metrics
% Beta = 15º

clear ll, close all, clc;

xc = linspace(0.5,3,21);

ycr = [0.3993036205	0.4951136505	0.5922630138	0.6713109123	0.7166042676	0.7459573426	0.7627008454	0.7808891984	0.7939549573	0.8033390046	0.8151139008	0.8295624937	0.845306481	0.858390564	0.8699522099	0.8854579919	0.8972273645	0.9063915779	0.9154727474	0.9260355767	0.9337137197];
ycr1 = [0.1839471041	0.3051392421	0.4531945133	0.5705497821	0.6421968523	0.6912400242	0.7191428305	0.7515770969	0.7717765061	0.7877746053	0.8060007554	0.8277895521	0.8537556126	0.8735803947	0.8878198136	0.9102344649	0.9243708814	0.9365888741	0.9480179637	0.958759816	0.9686295278];
ycr2 = [0.3537465926	0.4700848889	0.5717725648	0.6436158241	0.6812775093	0.698718713	0.7047863148	0.7093527593	0.7137544815	0.7169672037	0.7238389815	0.7319296111	0.7355833056	0.7496420926	0.7712748611	0.7847319815	0.8088698426	0.8237902315	0.8395152037	0.8568937037	0.8652296019];
ycr3 = [0.6705822341	0.7296874566	0.7646568266	0.8002283006	0.8164469046	0.8260152168	0.8327708728	0.8382066358	0.8454617543	0.8488773035	0.8544821676	0.8621537543	0.8694700983	0.8742039595	0.8794257168	0.8873242514	0.8924075376	0.8961299075	0.9003347139	0.9085564191	0.9134133035];

%Ultimate centers --

yc = [900.5843333518	636.0970426586	412.894227857	295.1185427243	246.5740770704	183.246691075	141.1411033725	130.2553631419	126.6148904844	125.8279714371	123.7168415848	102.9925108212	93.3717757809	78.0064907855	67.664969707	69.0273028973	66.5461069262	64.4850675652	69.6605145017	86.9872025848	100.8332783472];
yc1 = [1804.290211276	1268.9870884189	808.7416254286	563.1712113535	461.7611626634	331.2643737845	244.6816572615	222.4329075327	215.4937479588	214.1493156949	209.9791905642	162.8372685254	131.383513017	94.1984788039	68.3534263559	66.9981864818	61.6273576416	57.326475138	68.1264247337	80.0666614358	94.6534340847];
yc2 = [190.1553400278	129.1409975741	109.5255060648	102.6181139352	98.127307287	95.1399315278	93.1863985463	92.8278058333	91.3457407407	90.4575421852	89.8442083056	90.0606459722	89.7770701667	88.3966166296	89.6402555463	95.7105959352	93.8306819907	92.9800956204	95.0690408426	101.477940463	102.3489882963];
yc3 = [43.6357891214	38.8937593353	35.0874265751	35.2465605434	36.0538018873	34.0681566012	32.5193096301	31.9110000087	31.5342547486	31.4444199364	31.3234724104	35.5958532254	49.1214310636	55.4358993324	59.983861422	63.1204515434	63.9007582197	64.1354595954	63.5606827948	90.7247280289	107.7366858786];

st_v = 5;

xc = xc(st_v:end);
ycr = ycr(st_v:end);
ycr1 = ycr1(st_v:end);
ycr2 = ycr2(st_v:end);
ycr3 = ycr3(st_v:end);

yc = yc(st_v:end);
yc1 = yc1(st_v:end);
yc2 = yc2(st_v:end);
yc3 = yc3(st_v:end);

%Find the max value - all paths
v_max = max( [yc yc1 yc2 yc3 ] );
v_min = min( [yc yc1 yc2 yc3 ] );
vr_max = max( [ycr1 ycr1 ycr2 ycr3 ] );
vr_min = 0;
step = 15;
step_r = 0.1;
st_r = 0.50;

%Find optimal values
c = find(yc==min(min(yc)));
min_y_c = min(yc);
v_max = max(yc); 
step = 25;
step_r = 0.1;

%Plot
[AX,H1,H2] = plotyy(xc , yc , xc , ycr , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SA');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Santos Approach - Overall - Metric A');
xlabel('C0 [m]');
hold on
plot(xc(c),min_y_c,'r*');


%Find optimal values
c = find(yc1==min(min(yc1)));
min_y_c1 = min(yc1);
v_max = max(yc1); 
step = 50;
step_r = 0.1;

%Plot
figure;
[AX,H1,H2] = plotyy(xc , yc1 , xc , ycr1, 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SA');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Santos Approach - Scene 1 - Metric A');
xlabel('C0 [m]');
hold on
plot(xc(c),min_y_c1,'r*');

%Find optimal values
c = find(yc2==min(min(yc2)));
min_y_c2 = min(yc2);
v_max = max(yc2); 
step = 10;
step_r = 0.1;

%Plot
figure;
[AX,H1,H2] = plotyy(xc , yc2 , xc , ycr2, 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SA');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Santos Approach - Scene 2 - Metric A');
xlabel('C0 [m]');
hold on
plot(xc(c),min_y_c2,'r*');
% axes(AX(2));
% lh=line([xc(1) xc(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(yc3==min(min(yc3)));
min_y_c3 = min(yc3);
v_max = max(yc3); 
step = 10;
step_r = 0.1;

%Plot
figure;
[AX,H1,H2] = plotyy(xc , yc3 , xc , ycr3, 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SA');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Santos Approach - Scene 3 - Metric A');
xlabel('C0 [m]');
hold on
plot(xc(c),min_y_c3,'r*');


%% SANTOS C0 SEGMENTATION  Small clusters Removed
% Ratios + Metrics
% Beta = 15º

clear ll, close all, clc;

xc = linspace(0.5,3,21);

ycr = [0.2790521453	0.377953925	0.4833507532	0.5766151961	0.6329762122	0.6724948547	0.695230391	0.7204669112	0.741094421	0.7580225225	0.7789223587	0.7990904498	0.8213783772	0.8378608858	0.854114323	0.8763529666	0.8932647901	0.9059318674	0.9172670796	0.9280884048	0.935679917];
ycr1 = [0.1374284673	0.2419612203	0.3778514383	0.4935667845	0.5693553075	0.6261639225	0.6580851695	0.6969133438	0.7203790557	0.7397195617	0.7617024479	0.7884008644	0.8195117433	0.8430367724	0.8597953293	0.8873780581	0.9046185666	0.9206856707	0.9353951138	0.9478408257	0.9584455763];
ycr2 = [0.2811781296	0.3904754352	0.49367725	0.5741381944	0.6166228333	0.6390089815	0.6469420741	0.6533611111	0.6601816667	0.6644170556	0.6731376481	0.6833271481	0.6880912407	0.7054315093	0.73354475	0.7493372963	0.7780524444	0.7909535833	0.7985135463	0.8142464352	0.8237266944];
ycr3 = [0.4474364595	0.5363720289	0.6060558266	0.6765184046	0.7140212948	0.7382496214	0.7546411272	0.7695277486	0.7910771474	0.8090875896	0.8324962659	0.8479841936	0.8652105462	0.8730190694	0.8849676705	0.9028394682	0.915674685	0.924210289	0.9326962803	0.9400455809	0.943450815];

yc = [1375.7641710323	554.1596640946	310.5656392284	254.5876663091	184.5357711742	127.0638594567	88.2680410727	80.8379919539	76.3081116021	70.4704216978	67.0881031211	51.1511803587	45.7685230623	42.9006286874	43.0091734556	39.6555289031	38.6125585963	38.8332145767	39.5698773899	40.0180471326	39.3724571038];
yc1 = [2758.9251715375	1039.6373038305	583.5468524939	470.8490472567	326.6327250799	210.1672613995	128.6084167748	116.6945925739	108.0098279177	95.8131275811	88.0233583777	61.4155237579	50.283426063	42.7062642518	40.7269153729	34.1677760266	33.7181128184	34.7177012446	34.3611600678	35.4165869177	35.4532948523];
yc2 = [340.8339571019	312.6566472593	115.5875794167	75.1270683056	67.9280928981	64.5557090926	64.1171147222	63.6283999167	62.5103446296	61.4072274722	60.6631079444	59.1224244537	58.6783502222	57.1292225185	58.5424414444	63.4768613611	64.9127961574	65.4591400093	71.5843067778	68.3069287315	62.459039787];
yc3 = [47.8074366272	50.0557930173	45.5841056474	52.4656843815	51.3207055405	47.3793948237	47.6545291676	43.4098413295	42.7744410665	43.0492871503	44.1044009769	36.411098896	36.3496899046	38.6913349827	40.8848371705	38.7703787081	36.2454500694	35.4346800665	35.7942758873	36.6804860231	36.8443157168];

st_v = 5;

xc = xc(st_v:end);
ycr = ycr(st_v:end);
ycr1 = ycr1(st_v:end);
ycr2 = ycr2(st_v:end);
ycr3 = ycr3(st_v:end);

yc = yc(st_v:end);
yc1 = yc1(st_v:end);
yc2 = yc2(st_v:end);
yc3 = yc3(st_v:end);

%Find the max value - all paths
v_max = max( [yc yc1 yc2 yc3 ] );
v_min = min( [yc yc1 yc2 yc3 ] );
vr_max = max( [ycr1 ycr1 ycr2 ycr3 ] );
vr_min = 0;
step = 15;
step_r = 0.1;
st_r = 0.50;

%Find optimal values
c = find(yc==min(min(yc)));
min_y_c = min(yc);
v_max = max(yc); 
step = 15;
step_r = 0.1;

%Plot
[AX,H1,H2] = plotyy(xc , yc , xc , ycr , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SA');
set(AX(1),'YLim',[0 (v_max+step/2)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Santos Approach - Overall - Metric A - Small clusters removed');
xlabel('C0 [m]');
hold on
plot(xc(c),min_y_c,'r*');


%Find optimal values
c = find(yc1==min(min(yc1)));
min_y_c1 = min(yc1);
v_max = max(yc1); 
step = 30;
step_r = 0.1;

%Plot
figure;
[AX,H1,H2] = plotyy(xc , yc1 , xc , ycr1, 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SA');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Santos Approach - Scene 1 - Metric A - Small clusters removed');
xlabel('C0 [m]');
hold on
plot(xc(c),min_y_c1,'r*');

%Find optimal values
c = find(yc2==min(min(yc2)));
min_y_c2 = min(yc2);
v_max = max(yc2); 
step = 10;
step_r = 0.1;

%Plot
figure;
[AX,H1,H2] = plotyy(xc , yc2 , xc , ycr2, 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SA');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Santos Approach - Scene 2 - Metric A - Small clusters removed');
xlabel('C0 [m]');
hold on
plot(xc(c),min_y_c2,'r*');
% axes(AX(2));
% lh=line([xc(1) xc(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(yc3==min(min(yc3)));
min_y_c3 = min(yc3);
v_max = max(yc3); 
step = 5;
step_r = 0.1;

%Plot
figure;
[AX,H1,H2] = plotyy(xc , yc3 , xc , ycr3, 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SA');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Santos Approach - Scene 3 - Metric A - Small clusters removed');
xlabel('C0 [m]');
hold on
plot(xc(c),min_y_c3,'r*');


%% SANTOS BETA SEGMENTATION
% Ratios + Metrics
%  METRIC 1 %
%  C0 = 1.0 m

clear ll, close all, clc;

xb = linspace(5,40,21);

%Ratios -- ultimate centers
ybr = [0.6996839193	0.7036546644	0.7062285144	0.7092054775	0.7120300946	0.7145381534	0.7170083022	0.7198788328	0.7220176724	0.7243994014	0.7272439377	0.7300261038	0.7350453691	0.7391001176	0.7435575975	0.7486060265	0.756091609	0.7707196101	0.7878634798	0.802434895	0.8147453114];
ybr1 = [0.6204459564	0.6247708668	0.6280281598	0.6315988087	0.6354346925	0.6385813995	0.6425258959	0.6473057312	0.6498917385	0.6531627772	0.6575855496	0.6613986586	0.6688519007	0.6752259613	0.6824203995	0.6893241646	0.6994196126	0.7238882179	0.7497564867	0.7681697409	0.7835742446];
ybr2 = [0.6654775093	0.6675741296	0.6700910556	0.6743708796	0.676920963	0.6806306852	0.6812775093	0.6831469722	0.6855089167	0.6867522315	0.6882364815	0.6894730926	0.6905162315	0.6915527778	0.6926828796	0.6941142685	0.696437463	0.6985252778	0.7196823333	0.7350151944	0.7482948519];
ybr3 = [0.8049427948	0.8090757803	0.8108516127	0.8127132543	0.814416474	0.8157871301	0.8170665665	0.8179705434	0.8195059855	0.8211815405	0.8225668266	0.8246008439	0.8279559162	0.8301843353	0.8324134711	0.8363763121	0.8423580318	0.8491541561	0.8546315491	0.8643795087	0.8726941561];

yb = [257.875830098	254.5367242238	253.0456237416	249.9163547105	247.5991080415	245.5152987855	246.4588743426	245.6277739123	243.7797313714	239.4267415317	231.9964290692	217.4124603783	219.6821823564	220.9673241661	220.6311615779	230.8766762814	249.8804862537	284.8082265063	328.9238359423	366.6623885952	374.2777402687];
yb1 = [484.1169464165	477.6690596852	474.8626308765	468.4227074528	463.8496126538	459.4710002857	461.5162220484	459.9383154649	455.8230614213	446.9306278983	431.3137005521	400.8959408136	405.0840898765	407.8114816392	399.5583370823	411.4512609734	434.0916176053	492.9364980533	574.0138590799	640.0710343656	649.6984716199];
yb2 = [99.480984287	99.3580065648	98.794920963	98.4156056574	98.0120172037	98.2702206852	98.127307287	97.7285918056	97.6621488889	96.1124113148	96.0722341111	95.9776483426	95.8702183148	95.5920288611	95.4574386574	95.3356556574	95.6214486019	95.3924817963	110.1380418148	149.3539877037	154.694769713];
yb3 = [37.2661835896	36.6339697775	36.4232305434	36.3873293208	36.1654300231	36.0897603902	36.0575004682	35.9828548555	36.2849440809	36.4754195491	36.5102439046	36.3034496272	37.0252583988	37.0768468728	46.1278049075	57.6429385636	78.1486331272	95.5016492803	104.6657036012	108.1408180549	114.064441922];


st_v = 1;

xc = xc(st_v:end);
ycr = ycr(st_v:end);
ycr1 = ycr1(st_v:end);
ycr2 = ycr2(st_v:end);
ycr3 = ycr3(st_v:end);

yc = yc(st_v:end);
yc1 = yc1(st_v:end);
yc2 = yc2(st_v:end);
yc3 = yc3(st_v:end);

%Find the max value - all paths
v_max = max( [yb yb1 yb2 yb3 ] );
v_min = min( [yb yb1 yb2 yb3 ] );
vr_max = max( [ybr1 ybr1 ybr2 ybr3 ] );
vr_min = 0;
step = 15;
step_r = 0.1;
st_r = 0.60;

%Find optimal values
c = find(yb==min(min(yb)));
min_y_b = min(yb);
v_max = max(yb); 
step = 30;
step_r = 0.1;

%Plot
[AX,H1,H2] = plotyy(xb , yb , xb , ybr , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SA');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Santos Approach - Overall - Metric A');
xlabel('Beta [degrees]');
hold on
plot(xb(c),min_y_b,'r*');


%Find optimal values
c = find(yb1==min(min(yb1)));
min_y_b1 = min(yb1);
v_max = max(yb1); 
step = 50;
step_r = 0.1;

%Plot
figure
[AX,H1,H2] = plotyy(xb , yb1 , xb , ybr1 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SA');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Santos Approach - Scene 1 - Metric A');
xlabel('Beta [degrees]');
hold on
plot(xb(c),min_y_b1,'r*');


%Find optimal values
c = find(yb2==min(min(yb2)));
min_y_b2 = min(yb2);
v_max = max(yb2); 
step = 10;
step_r = 0.1;

%Plot
figure
[AX,H1,H2] = plotyy(xb , yb2 , xb , ybr2 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SA');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Santos Approach - Scene 2 - Metric A');
xlabel('Beta [degrees]');
hold on
plot(xb(c),min_y_b2,'r*');


%Find optimal values
c = find(yb3==min(min(yb3)));
min_y_b3 = min(yb3);
v_max = max(yb3); 
step = 10;
step_r = 0.1;

%Plot
figure
[AX,H1,H2] = plotyy(xb , yb3 , xb , ybr3 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SA');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Santos Approach - Scene 3 - Metric A');
xlabel('Beta [degrees]');
hold on
plot(xb(c),min_y_b3,'r*');



%% SANTOS BETA SEGMENTATION -- Small clusters remove
% Ratios + Metrics
%  METRIC 1 %
%  C0 = 1.0 m

clear ll, close all, clc;

xb = linspace(5,40,21);

%Ratios -- NO small clusters
ybr = [0.6078533576	0.614611985	0.6180861995	0.6226279562	0.6266293899	0.6302812388	0.6339411719	0.6382964971	0.6418298754	0.6456400104	0.6494388374	0.6539009181	0.6622829942	0.6683818051	0.6816840784	0.6938977716	0.7143305606	0.7428477855	0.7663836482	0.7831717509	0.8007587036];
ybr1 = [0.5472568668	0.5518008838	0.5545599274	0.5583306465	0.5621286562	0.5654488838	0.5696522058	0.5750781768	0.5780743269	0.5814423729	0.5858017748	0.5901004237	0.601359046	0.6091255738	0.6180721816	0.6332835642	0.66883854	0.716930937	0.7423548281	0.7595380605	0.7737939613];
ybr2 = [0.5997395648	0.6020758148	0.6049044259	0.6094854815	0.6121602778	0.6159024537	0.6166228333	0.6185234444	0.6214482315	0.626769713	0.6299227778	0.6318928333	0.63327825	0.6345509167	0.6364764074	0.6382671296	0.6409574167	0.6444975741	0.6482967685	0.6505598796	0.6539560926];
ybr3 = [0.6827164798	0.6934989538	0.6980283439	0.7034781763	0.7081365202	0.7121560116	0.7160848815	0.7199284509	0.724293052	0.7281591329	0.7314904017	0.7369254191	0.7440578584	0.7496724393	0.7717249509	0.7836138902	0.7915343295	0.8044821243	0.8319249364	0.8527752081	0.8787677225];

yb = [207.9201325006	199.6486730577	196.9553665375	195.7282475236	190.5709709723	185.9756368039	184.2784193299	175.8529038881	172.2812312757	170.1905160173	168.135447564	161.2441050046	159.1290602191	160.009467932	155.2329040392	152.5952028408	154.4093384729	151.9046637255	156.0070685363	155.0603326609	155.929392985];
yb1 = [378.7642815811	361.4793326877	355.401788201	351.2761510969	340.0039883753	330.1994989346	326.6030845472	308.4121022373	303.6860596053	298.5193052881	293.6596559153	289.0598736102	284.9160539249	286.5990948862	269.3000953002	256.6434102276	255.9674998329	244.7103472542	237.1580062107	230.1646503729	218.7350445763];
yb2 = [69.9728791574	69.7606090278	68.7504239167	68.0462585741	67.496940537	67.2811672593	67.9280928981	67.5870529167	67.4392851481	68.9108499907	68.5406276481	67.2029267685	67.1176316204	66.769271463	66.4317114259	66.3686742315	67.2245541204	68.4756984167	70.0664324907	70.6283213241	72.1127463611];
yb3 = [47.0521261156	47.0239577052	47.8448510925	49.9142898092	50.6176735723	50.8731733671	50.7112185202	51.4186928671	48.1561910491	48.6252384509	49.3919287832	38.0317778815	37.7047997803	38.0106970838	46.7958485809	55.3135711618	60.3990380925	67.1693485694	85.9673327948	91.7671939595	107.1243748584];

st_v = 1;

xc = xc(st_v:end);
ycr = ycr(st_v:end);
ycr1 = ycr1(st_v:end);
ycr2 = ycr2(st_v:end);
ycr3 = ycr3(st_v:end);

yc = yc(st_v:end);
yc1 = yc1(st_v:end);
yc2 = yc2(st_v:end);
yc3 = yc3(st_v:end);

%Find the max value - all paths
v_max = max( [yb yb1 yb2 yb3 ] );
v_min = min( [yb yb1 yb2 yb3 ] );
vr_max = max( [ybr1 ybr1 ybr2 ybr3 ] );
vr_min = 0;
step = 15;
step_r = 0.1;
st_r = 0.50;

%Find optimal values
c = find(yb==min(min(yb)));
min_y_b = min(yb);
v_max = max(yb); 
step = 25;
step_r = 0.1;

%Plot
[AX,H1,H2] = plotyy(xb , yb , xb , ybr , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SA');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Santos Approach - Overall - Metric A - Small clusters removed');
xlabel('Beta [degrees]');
hold on
plot(xb(c),min_y_b,'r*');


%Find optimal values
c = find(yb1==min(min(yb1)));
min_y_b1 = min(yb1);
v_max = max(yb1); 
step = 50;
step_r = 0.1;

%Plot
figure
[AX,H1,H2] = plotyy(xb , yb1 , xb , ybr1 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SA');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Santos Approach - Scene 1 - Metric A - Small clusters removed');
xlabel('Beta [degrees]');
hold on
plot(xb(c),min_y_b1,'r*');


%Find optimal values
c = find(yb2==min(min(yb2)));
min_y_b2 = min(yb2);
v_max = max(yb2); 
step = 10;
step_r = 0.1;

%Plot
figure
[AX,H1,H2] = plotyy(xb , yb2 , xb , ybr2 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SA');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Santos Approach - Scene 2 - Metric A - Small clusters removed');
xlabel('Beta [degrees]');
hold on
plot(xb(c),min_y_b2,'r*');


%Find optimal values
c = find(yb3==min(min(yb3)));
min_y_b3 = min(yb3);
v_max = max(yb3); 
step = 10;
step_r = 0.1;

%Plot
figure
[AX,H1,H2] = plotyy(xb , yb3 , xb , ybr3 , 'plot');
set(H1,'LineStyle','-');
set(H2,'LineStyle','-');
set(H1, 'Color', 'b');
set(H2, 'Color', 'g');
set(H1, 'Marker', 'o');
set(H2, 'Marker', 'o');
set(get(AX(1),'Ylabel'),'String','Medium Energy'); 
set(get(AX(2),'Ylabel'),'String','#clusters ground truth/#clusters SA');
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Santos Approach - Scene 3 - Metric A - Small clusters removed');
xlabel('Beta [degrees]');
hold on
plot(xb(c),min_y_b3,'r*');






