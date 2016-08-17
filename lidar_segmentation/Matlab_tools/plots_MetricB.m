%% Simple Segmentation -- METRIC B
% Ratios + Metrics

clear all, close all, clc;

xs = linspace(0.5,3.2,21);

ysr = [0.3600569123	0.4520463887	0.5490273241	0.6415356759	0.70240809	0.7412226955	0.7602669827	0.777757391	0.7905924048	0.8014123322	0.8134991142	0.8291913356	0.8449269954	0.861036451	0.8741983264	0.8885503576	0.8994146275	0.9065365306	0.9161304245	0.9232031326	0.9371049666];
ysr1 = [0.1492532107	0.249242276	0.393149	0.5342866368	0.6264984019	0.6863824794	0.7164910024	0.7478083874	0.771520908	0.7868846271	0.8054891937	0.8295680412	0.85430323	0.8778057966	0.8967740218	0.9145568983	0.9284986344	0.9358796295	0.9474861501	0.9586842203	0.9818278717];
ysr2 = [0.3074804259	0.4210264444	0.5287428796	0.6170023241	0.6664164537	0.693012	0.7042186111	0.7077651667	0.7129448426	0.7177192963	0.7207194259	0.7307838704	0.7351837037	0.7616843426	0.7763321667	0.8005668333	0.8174161852	0.8254811667	0.8403022315	0.847160037	0.8542915926];
ysr3 = [0.6280921416	0.7038043439	0.7414217399	0.7772104017	0.8042514364	0.821730685	0.8300146821	0.835353052	0.8375937486	0.8448770434	0.8520202225	0.8594584653	0.867990263	0.8720315029	0.8777988555	0.8849709335	0.8902936358	0.8968119046	0.902371841	0.9045874249	0.9095711069];

% Metric - ultimate Boundaries

ys = [275.7635586897	194.5110046378	134.9856283022	104.084742707	90.9373626251	84.7315960161	81.9107158997	79.5258340323	78.1642058997	77.5478113552	76.6119647059	75.3749586943	74.2970945317	73.1307781292	72.4848155652	71.7555808062	71.2690300127	71.3179362065	71.8620526494	72.3028849654	74.843922293]; 
ys1 = [482.6834891768	327.2700340654	211.1334661065	152.0028860194	127.8146860847	116.3888313148	111.2408533317	106.5498589976	103.9465472228	103.0632558257	101.3193093196	99.0488080218	96.6098726755	94.30466846	92.6030785908	91.0712808499	90.1416611017	89.9410168765	90.0902967312	90.5789002615	94.8664346295];
ys2 = [183.0525703796	142.6391017963	117.2410955185	102.4510585093	95.0046782222	92.1654305278	90.6486388704	90.4092173981	90.1114772963	89.5301173889	89.3534466389	88.256034213	87.8197603889	86.2123145463	85.1782456296	82.8349056111	81.6351837963	81.6441679074	82.0148606204	82.3341894815	82.4694713426];
ys3 = [57.7140079566	52.2355316705	49.6311558382	47.3975886763	45.6494878468	44.6238147861	44.1735990202	43.8717076908	43.6601241734	43.3513731532	43.1431399249	43.0961843353	43.4426861908	43.773487263	44.5087084075	45.2412709884	45.5061940347	45.8654352225	46.9349773121	47.3567138526	48.563989078];

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
step = 10;
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
title('Simple Segmentation - Overall - Metric B');
hold('on', AX(1));
hold('on', AX(2));
plot(xs(c),min_ys,'r*','Parent' , AX(1));

%Find optimal values
c = find(ys1==min(min(ys1)));
min_ys1 = min(ys1);
v_max = max(ys1);
step = 20;

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
title('Simple Segmentation - Scene 1 - Metric B');
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
title('Simple Segmentation - Scene 2 - Metric B');
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
title('Simple Segmentation - Scene 3 - Metric B');
hold on
plot(xs(c),min_ys3,'r*');


%%  Simple Segmentation Metric B  Small clusters removed 

clear all, close all, clc;

xs = linspace(0.5,3.2,21);

%ratios -- No small CLusters
ysr = [0.2397510196	0.3280651961	0.4258516597	0.5317902653	0.6090198639	0.6613588708	0.6867737093	0.7108174937	0.7279312376	0.746730489	0.7655617612	0.7886916805	0.8129918258	0.8360560058	0.8568192076	0.8777924175	0.8951966125	0.9054630219	0.915999391	0.9260803045	0.9434885213];
ysr1 = [0.1097316683	0.1930721356	0.3220164576	0.4579046513	0.5539697458	0.6213452954	0.6557170557	0.6924802809	0.7205414383	0.7401567433	0.7615583245	0.7913653414	0.8204475448	0.8488193462	0.8717260339	0.8937832155	0.9119286077	0.9225016513	0.9361349564	0.9498870581	0.9802192639];
ysr2 = [0.2411258704	0.3431615741	0.4492457407	0.5436432407	0.6007829722	0.6327096389	0.6461919907	0.6512056204	0.6587381852	0.6654851019	0.6693249352	0.6820573704	0.6875760648	0.7230078981	0.740891213	0.7673511481	0.7875727593	0.7967475	0.8147679907	0.8231925093	0.8318818333];
ysr3 = [0.3945183844	0.4844863671	0.5424914798	0.6162834364	0.6773010289	0.7180632746	0.7365113497	0.7513127283	0.758349841	0.7799370173	0.8003796705	0.8187849855	0.8432394855	0.8561078439	0.8752114162	0.8931781329	0.9088181792	0.9190593295	0.9235629827	0.9297788382	0.934481948];


% Metric - ultimate Boundaries

ys = [416.322620752	275.9614063749	179.8405841569	130.5661845467	110.0703896805	99.7621192191	95.0037230773	91.2386474521	88.6012539262	86.6170644856	84.7691749285	82.3882088627	80.6088812953	78.612588301	77.4498707036	75.6780048005	74.6144199666	74.3177410208	73.8487146667	74.0117686205	76.3576600185]; 
ys1 = [737.0611729104	473.5305890872	288.59987354	195.7226550097	158.3654086586	140.4266627869	132.1689930799	125.2831761017	120.3871342978	117.233674816	114.0806682446	109.8032324334	106.0241645763	102.5094470073	99.8051442131	97.0177328789	95.189952201	94.6523357845	93.7482303245	93.8832320557	98.8165978523];
ys2 = [244.6758245463	182.2413326944	141.4424725926	120.4879108056	110.7896585185	105.4380951204	103.9050831759	103.4046404537	102.6392105093	101.6912277963	101.1273448056	99.2365705741	98.4413683519	95.763930787	93.7818878889	90.1455264352	88.6506683889	88.5961565556	88.4669940093	88.2739656481	87.9075843796];
ys3 = [87.0533489277	69.388271974	62.0065076647	55.938529237	52.1989334017	49.4515357197	47.8632508179	46.8042035896	46.2785143179	45.3665740116	44.675738263	44.4054695578	44.7059315925	44.7347049653	45.6678307746	45.6901434335	45.6733516358	45.588675948	45.5328934653	45.8405788006	45.9445584104];

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
step = 20;
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
title('Simple Segmentation - Overall - Metric B - Small clusters removed');
hold('on', AX(1));
hold('on', AX(2));
plot(xs(c),min_ys,'r*','Parent' , AX(1));

%Find optimal values
c = find(ys1==min(min(ys1)));
min_ys1 = min(ys1);
v_max = max(ys1);
step = 50;

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
title('Simple Segmentation - Scene 1 - Metric B - Small clusters removed');
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
title('Simple Segmentation - Scene 2 - Metric B - Small clusters removed');
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
title('Simple Segmentation - Scene 3 - Metric B - Small clusters removed');
hold on
plot(xs(c),min_ys3,'r*');


%% Dietmayer Segmentation -- METRIC B
% Ratios + Metrics

clear all, close all, clc;

xd = linspace(0.5,3,21);

%Ratios

ydr = [0.5561711915	0.6593033518	0.7210270473	0.7537444637	0.7822403702	0.8060756851	0.8249652953	0.8445960035	0.8571253068	0.871839654	0.8886672987	0.9078001107	0.9284550346	0.95071706	0.9721474614	0.9889777324	1.004362083	1.0157879792	1.0361289619	1.0518823622	1.0645621765];
ydr1 = [0.3764877022	0.5284536707	0.6258138136	0.6779579225	0.7240162179	0.7645056828	0.7934927893	0.8224463995	0.8403951065	0.8636813995	0.8886158765	0.9147041743	0.9388506102	0.9700941235	1.0021342591	1.02510346	1.0448681598	1.0597965738	1.0816448814	1.0948792591	1.1062505351];
ydr2 = [0.5397551111	0.6537073889	0.705723213	0.7221526204	0.7440065278	0.755896787	0.7638287407	0.775153537	0.7878505185	0.794022037	0.8016177222	0.8100051481	0.8339865	0.8598724074	0.8768398333	0.8957380926	0.913237463	0.9206890278	0.9360418056	0.9505513241	0.9671018241];
ydr3 = [0.7757729769	0.8172376936	0.8394544451	0.8540674711	0.8636734046	0.8713581474	0.8816152746	0.8927103757	0.8987185145	0.905867578	0.9159002225	0.9300847283	0.9455337312	0.9559439249	0.9661031734	0.974960263	0.9844558671	0.9929415549	1.0130403439	1.0321888179	1.0452223671];

% Metric - ultimate results

yd = [144.4206566205	111.013303248	98.3616529608	93.6349675698	90.9948708973	89.5360910035	88.9691246701	88.451190917	88.5796905882	89.3482659573	90.1926210796	91.3148864106	93.0178750588	94.592868511	96.7232653529	98.7602647382	101.836678737	103.9822796459	108.5634342261	112.7603734441	116.017532812]; 
yd1 = [218.3404912906	153.6549796804	128.5980495109	118.7096921719	112.52425954	108.503497276	104.942980155	102.5002590484	101.1030811768	101.1027794237	100.6882340339	99.5342162518	98.4503621937	98.2450219879	99.7341160266	101.7069058354	105.6548141453	108.4979916634	112.3494301889	114.9457831162	117.5045485932];
yd2 = [124.3200561111	109.5430843889	103.8737141759	102.6776778704	103.0805414259	104.1283747407	106.9421096759	109.3454726204	110.5801283981	111.3380091111	111.7143751296	112.6236783889	115.5608239537	116.3586091111	116.4452206667	114.2835282222	114.5232451852	114.6176093889	114.9765930463	116.9831854259	121.6189010833];
yd3 = [62.4610414075	60.5733358208	60.5497038671	60.8821526185	61.524090763	62.3409885925	64.2920012717	65.159723974	66.7640616965	68.4537390145	70.9468477052	74.8526530058	79.496904922	83.4395755318	86.9734026301	90.3976195665	93.3192246908	95.2724395434	102.0425166387	108.8336743439	112.4941649191];

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
step = 20;
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
title('Dietmayer Segmentation - Overall - Metric B');
xlabel('C0 [m]');
hold on
plot(xd(c),min_yd,'r*');
axes(AX(2));
lh=line([xd(1) xd(end)],[1 1] , 'Color','r');


% Find optimal values
c = find(yd1==min(min(yd1)));
min_yd1 = min(yd1);
v_max = max(yd1);
step = 20;

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
title('Dietmayer Segmentation - Scene 1 - Metric B');
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
title('Dietmayer Segmentation - Scene 2 - Metric B');
xlabel('C0 [m]');
hold on
plot(xd(c),min_yd2,'r*');
% axes(AX(2));
% lh=line([xd(1) xd(end)],[1 1] , 'Color','r');


% Find optimal values
c = find(yd3==min(min(yd3)));
min_yd3 = min(yd3);

v_max = max(yd3);
step = 10;
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
title('Dietmayer Segmentation - Scene 3 - Metric B');
xlabel('C0 [m]');
hold on
plot(xd(c),min_yd3,'r*');
axes(AX(2));
lh=line([xd(1) xd(end)],[1 1] , 'Color','r');


%%  Dietmayer Segmentation Metric B  Small clusters removed 

clear all, close all, clc;

xd = linspace(0.5,3,21);

%Ratios - NO small clusters
ydr = [0.4538301269	0.5736247347	0.6477034118	0.691183143	0.7300153991	0.7633615479	0.789866925	0.8170682145	0.8362844464	0.859158639	0.8825938939	0.9076950438	0.9336391153	0.9656983691	0.9984231869	1.0258173333	1.046764767	1.0643178985	1.0841373322	1.0973925536	1.1107888028];
ydr1 = [0.306456385	0.4503854746	0.5513598523	0.6105289855	0.6687438257	0.7200259782	0.7576199225	0.79263146	0.819400368	0.8556813705	0.8903337191	0.9246656683	0.9579666828	0.9986773172	1.0402815303	1.0749333535	1.1014927893	1.1239663148	1.1495236005	1.1673202107	1.1851376102];
ydr2 = [0.4475173704	0.5621416296	0.6177684352	0.6367129074	0.6532843148	0.6624931204	0.6683963333	0.6773324352	0.6870102778	0.6943139907	0.7005918796	0.7083883241	0.72988225	0.7583451111	0.7775301944	0.7944829537	0.8084111111	0.8142698333	0.8288315463	0.843898037	0.8557514444];
ydr3 = [0.6317120145	0.7243125665	0.7720469595	0.8044575723	0.8271024422	0.8465736301	0.8662739653	0.8898538902	0.9030322341	0.9147636503	0.9301651936	0.9496495462	0.9682011272	0.9910563064	1.0174085838	1.0393988266	1.0558385289	1.0711687514	1.0857803844	1.093049448	1.1016500087];


% Metric - No centers
yd = [186.4571339677	133.5883970911	113.4890556955	105.8949462295	100.9393945548	98.1233331396	96.4605009354	94.6240669954	94.1205330023	94.1115550069	93.3790832168	92.7248727543	92.7807080358	94.9725109977	98.127922677	101.2878381603	103.9325376032	106.4208848212	108.9137424221	111.0406802411	113.0091714925]; 
yd1 = [300.2001248668	199.0663544189	159.4051359685	144.5953389467	134.9412036973	128.9879014843	124.8380901162	120.9982534915	119.8158064479	119.1493340775	116.9395748596	114.6790225496	113.6033092567	112.9608811598	114.3759353027	117.7595891308	120.3497783632	122.9720703196	125.749746678	128.5904516465	131.4342911646];
yd2 = [143.1019209259	117.9826969444	108.9552395833	106.7037718704	105.100708463	103.8323482593	103.9323934537	102.8420146759	101.7356526019	101.7845310463	101.4544593426	101.2050045278	99.6769524815	97.9647639259	96.8486275278	94.0982926296	93.8959576574	93.9324628056	93.7136696852	94.0659972037	94.6953828056];
yd3 = [64.2216072832	60.3023255289	60.0968909191	59.4480810231	59.0545128266	59.5000951069	60.2555624277	60.5776006012	61.0726114711	61.8303868902	62.7356622052	63.8725083121	65.7733996416	72.5668456994	79.1329361503	83.8706062717	87.4690410723	90.5628211416	93.5621184277	95.3909986734	96.7326245376];

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
step = 50;
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
title('Dietmayer Segmentation - Overall - Metric B - Small clusters removed');
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
title('Dietmayer Segmentation - Scene 1 - Metric B - Small clusters removed');
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
title('Dietmayer Segmentation - Scene 2 - Metric B - Small clusters removed');
xlabel('C0 [m]');
hold on
plot(xd(c),min_yd2,'r*');
% axes(AX(2));
% lh=line([xd(1) xd(end)],[1 1] , 'Color','r');


% Find optimal values
c = find(yd3==min(min(yd3)));
min_yd3 = min(yd3);

v_max = max(yd3);
step = 10;
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
title('Dietmayer Segmentation - Scene 3 - Metric B - Small clusters removed');
xlabel('C0 [m]');
hold on
plot(xd(c),min_yd3,'r*');
axes(AX(2));
lh=line([xd(1) xd(end)],[1 1] , 'Color','r');


%% ABD -- METRIC B
% Ratios + Metrics

clear all, close all, clc;

x_abd = linspace(4,22,21);

%ratios -- No small CLusters
y_abdr = [1.1410030219	1.1182093564	1.1058006413	1.113231331	1.0863426378	1.0623786424	1.0453287555	1.0206398316	0.9944944383	0.9672135848	0.9391960484	0.8974375513	0.8667164325	0.8398331003	0.8172093875	0.7890722526	0.7574697497	0.7250825617	0.6899668604	0.6536217624	0.617564015];
y_abdr1 = [1.0792750363	1.0303662833	1.0115853002	0.993639431	0.9623893293	0.943204586	0.9248408305	0.8933028571	0.8566100799	0.8144288692	0.775379339	0.7274110073	0.7035803317	0.6726982397	0.6412820751	0.6060208039	0.5663411017	0.5177264286	0.4736270363	0.4256539685	0.3812788063];
y_abdr2 = [0.9351915185	0.8763565741	0.8775914167	0.8857836019	0.8685114444	0.8522283241	0.8456082963	0.8412333426	0.8432390093	0.8420881759	0.8201975	0.7903408056	0.7719184074	0.7558051944	0.7388819167	0.7142382778	0.6940703704	0.6671835093	0.6444106852	0.6182866204	0.5841430648];
y_abdr3 = [1.2789258555	1.2985541243	1.2894929306	1.3269764451	1.3022920173	1.2702258092	1.2514886474	1.2286342572	1.2062916243	1.1886402659	1.1718785462	1.1338173526	1.0910326069	1.0655605896	1.0516525867	1.0309286618	1.0053982601	0.9906640087	0.9624189249	0.9367630751	0.9100358468];

%Metric - ultimate Boundaies

y_abd = [189.3588534095	190.2810475098	193.9118631811	211.0714445709	206.1546499089	198.2218034348	197.3875656321	191.6550251061	188.6288921569	189.1429803518	190.016950782	179.5380017935	172.5775672122	173.9793970842	183.3856818397	191.7199995917	195.5858689089	204.3955997785	212.285595917	221.7120236078	235.2503404533];
y_abd1 = [129.8838304455	118.0490678499	119.7949550121	120.7528551017	121.9309636392	122.5605769564	124.4182664891	124.4072593632	124.985691293	131.7419684697	136.3555351864	140.6449603123	147.5096329492	157.5158470436	172.6985461646	188.6604863027	203.316850569	218.4535439177	237.2362391114	260.2192117942	290.6796752518];
y_abd2 = [114.7985485185	124.3306831111	143.0234906574	153.1672663704	158.557698713	163.2206688056	166.0743879444	175.6299009444	176.643751963	178.2464807778	176.5961228796	166.0550999259	162.7676244074	166.7917035926	173.0509167222	171.9797638333	174.4592623889	175.7380403889	186.8518687222	188.42694325	196.3335174815];
y_abd3 = [283.6238748324	297.0858074942	298.2651212919	336.9536084335	321.544398922	299.4594019191	294.2608134249	276.9268189249	268.3370918613	261.0604365983	258.2586098988	230.1708906185	205.5617599104	195.8745331069	199.3681722052	201.5336540723	192.9522795405	196.5605867283	190.4423210549	186.1378038324	181.235027185];

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
step = 25;
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
title('ABD Segmentation - Overall - Metric B');
xlabel('Lamda [degrees]');
hold on
plot(x_abd(c),min_y_abd,'r*');
axes(AX(2));
lh=line([x_abd(1) x_abd(end)],[1 1] , 'Color','r');

%Find optimal values
c = find(y_abd1==min(min(y_abd1)));
min_y_abd1 = min(y_abd1);
v_max = max(y_abd1);
step = 25;

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
title('ABD Segmentation - Scene1 - Metric B');
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
title('ABD Segmentation - Scene2 - Metric B');
xlabel('Lamda [degrees]');
hold on
plot(x_abd(c),min_y_abd2,'r*');


%Find optimal values
c = find(y_abd3==min(min(y_abd3)));
min_y_abd3 = min(y_abd3);

v_max = max(y_abd3);
step = 30;

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
title('ABD Segmentation - Scene3 - Metric B');
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

y_abd = [97.2117994187	101.2881776886	106.2223853033	109.8222938166	116.4762220761	119.2242547947	121.4969289193	125.962199436	131.569862797	137.2532188789	141.9536801707	145.7771646275	151.0792806424	157.9152182422	166.3579762099	178.1149001592	185.5687797682	196.8881937497	205.6118174833	216.5257677439	229.226690301];
y_abd1 = [114.2114699613	108.4790481792	109.6737809298	109.7748470775	112.868082322	115.4018828015	116.1537354843	119.3516438644	122.7484496344	130.6218836683	136.7783251356	144.535263724	153.206616385	162.7983207433	173.2691343898	184.1393609952	197.7491029976	214.6532209588	234.2609924673	259.2704442906	292.2650794867];
y_abd2 = [106.2667376944	111.1636608241	108.2708617037	107.6765609352	110.4889630833	117.6439708056	122.3361544352	126.7233555463	131.0002581944	131.3659973889	136.0100990741	138.1376576759	142.614773287	146.1119371852	149.821425213	152.9890129259	158.5879714259	162.5578955926	164.4883553056	168.1930250093	173.4285162222];
y_abd3 = [74.0938882399	89.6223346503	101.4632470231	110.548694552	122.6519032601	124.280064922	127.6128321936	133.6152473873	142.2772643382	147.0062864017	149.9864211763	149.6441352168	151.1821047254	155.7708050087	163.2702281734	178.7666154653	179.4515942399	186.3989335289	184.2511950751	180.5904637023	171.3982165058];

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
step = 30;
step_r = 0.10;
st_r = 0.30;

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
title('ABD Segmentation - Overall - Metric B - Small clusters removed');
xlabel('Lamda [degrees]');
hold on
plot(x_abd(c),min_y_abd,'r*');

%Find optimal values
c = find(y_abd1==min(min(y_abd1)));
min_y_abd1 = min(y_abd1);
v_max = max(y_abd1);
step = 25;

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
set(AX(1),'YLim',[0 (v_max+step/2)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('ABD Segmentation - Scene1 - Metric B - Small clusters removed');
xlabel('Lamda [degrees]');
hold on
plot(x_abd(c),min_y_abd1,'r*');
axes(AX(2));
lh=line([x_abd(1) x_abd(end)],[1 1] , 'Color','r');

%Find optimal values
c = find(y_abd2==min(min(y_abd2)));
min_y_abd2 = min(y_abd2);
v_max = max(y_abd2);
step = 25;

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
title('ABD Segmentation - Scene2 - Metric B - Small clusters removed');
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
title('ABD Segmentation - Scene3 - Metric B - Small clusters removed');
xlabel('Lamda [degrees]');
hold on
plot(x_abd(c),min_y_abd3,'r*');
axes(AX(2));
lh=line([x_abd(1) x_abd(end)],[1 1] , 'Color','r');


%% SNN SEGMENTATION - Metric B
% Ratios + Metrics

clear ll, close all, clc;

x_nn = linspace(0.5,3,21);

%Ratio results -- Ultimate boundaries
y_nnr = [0.3721719735	0.4651955675	0.5652047347	0.6685645848	0.7461939689	0.7951843299	0.8270252999	0.8480950254	0.8638526078	0.8759672468	0.8864453276	0.8997043633	0.9171613587	0.9322308155	0.9483712388	0.9667146828	0.9849866517	1.0052971107	1.0260628201	1.0377019354	1.0502509123];
y_nnr1 = [0.1522174334	0.2459748886	0.3839097845	0.5339905593	0.6433783923	0.7127435036	0.7557654068	0.7832103317	0.8077693898	0.8255758838	0.8360735981	0.8500551646	0.870764431	0.8883477627	0.9052608402	0.9227145206	0.9421993196	0.9700092421	0.9962762349	1.0081099831	1.0212628717];
y_nnr2 = [0.3189895278	0.4358631389	0.5575906204	0.6644523796	0.7318428148	0.7735024907	0.8157852593	0.8380898056	0.8504882593	0.8594516852	0.8705860741	0.8793024444	0.8984193333	0.9182209722	0.9576019074	0.9976955556	1.0229760278	1.0348946481	1.0485025463	1.0698718241	1.0944986574];
y_nnr3 = [0.6513191676	0.7360222803	0.7839825925	0.8304813208	0.8733984711	0.9003568728	0.915592526	0.9286671127	0.9349674017	0.9412716214	0.9515214075	0.9653359422	0.9783927746	0.9889844682	0.9969483555	1.0095647775	1.0242014364	1.0381796416	1.0546130202	1.0629826532	1.0710408092];


%Metric results -- Ultiamte boundaries
y_nn = [275.270980579	197.9751720923	138.1722700415	103.9897803403	89.3867311038	82.3103275998	78.1915409527	75.8129321153	74.0070836367	72.7878914371	72.0314603379	70.9666902168	69.7759190577	69.2415939204	68.8988794856	69.4415485202	70.0711822122	71.1110724083	73.6286022168	75.4645896298	77.2808221753];
y_nn1 = [483.9123926562	336.8799790315	221.0677023462	155.728484	129.2179343245	116.379962293	109.0914561671	104.88721223	101.5241280775	99.2312098136	98.1207301017	96.5351460363	94.3216012349	92.8139685642	91.3948500218	90.123323322	89.1666694479	89.7318497337	91.8011391114	92.1705374383	92.7217149225];
y_nn2 = [178.4557764815	137.9077403796	110.8330433241	94.0603120093	85.1573904074	81.3070218611	77.6359735741	75.7794540278	74.8955139444	73.9645960463	73.0117015556	72.6426516481	70.9910797963	69.6596141759	69.4262056852	71.3032486204	72.6752800093	73.775700287	75.8646868241	80.2832910185	85.6553549167];
y_nn3 = [56.4476824711	50.9219852688	47.7584635202	45.3316819827	43.1626902514	41.9565642803	41.4815302399	41.1191111503	40.8842806676	40.8567510116	40.5842509046	39.9239847601	40.0978725289	40.9741461879	41.8821538757	44.1738126705	46.4651741474	48.0528040665	51.2391372601	54.0195717254	56.2359139653];

st_v = 5;

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
step = 20;
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
title('SNN Segmentation - Overall - Metric B');
xlabel('Threshold [m]');
hold on
plot(x_nn(c),min_y_nn,'r*');
axes(AX(2));
lh=line([x_nn(1) x_nn(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_nn1==min(min(y_nn1)));
min_y_nn1 = min(y_nn1);
v_max = max(y_nn1);
step = 20;

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
title('SNN Segmentation - Scene 1 - Metric B');
xlabel('Threshold [m]');
hold on
plot(x_nn(c),min_y_nn1,'r*');
axes(AX(2));
lh=line([x_nn(1) x_nn(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_nn2==min(min(y_nn2)));
min_y_nn2 = min(y_nn2);

v_max = max(y_nn2);
step = 10;

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
title('SNN Segmentation - Scene 2 - Metric B');
xlabel('Threshold [m]');
hold on
plot(x_nn(c),min_y_nn2,'r*');
axes(AX(2));
lh=line([x_nn(1) x_nn(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_nn3==min(min(y_nn3)));
min_y_nn3 = min(y_nn3);
v_max = max(y_nn3);
step = 5;

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
title('SNN Segmentation - Scene 3 - Metric B');
xlabel('Threshold [m]');
hold on
plot(x_nn(c),min_y_nn3,'r*');
axes(AX(2));
lh=line([x_nn(1) x_nn(end)],[1 1] , 'Color','r');



%% SNN Metric B  Small clusters removed 

% Ratios + Metrics

clear all, close all, clc;

x_nn = linspace(0.5,3,21);

y_nnr = [0.2495988408	0.3405208074	0.4414688627	0.5538764464	0.6492687555	0.7133451638	0.7567475294	0.7849322722	0.8060869965	0.8239427578	0.8420403483	0.8593764348	0.8839244475	0.9053765779	0.928374173	0.9512475663	0.9710070173	0.9952512018	1.0180683506	1.0269880807	1.0362474648];
y_nnr1 = [0.112092184	0.1902247191	0.3142376828	0.4585844891	0.5731663341	0.6507009661	0.7016830291	0.7339209274	0.7634210484	0.7841153656	0.7969942421	0.8136669346	0.8393001259	0.8599187143	0.8802413632	0.9008676344	0.9238633317	0.9578288935	0.9917848499	1.0064452906	1.0187334262];
y_nnr2 = [0.250884713	0.3562449352	0.4764994815	0.5910140463	0.6695188889	0.7200119259	0.7706911944	0.7988591204	0.8081433241	0.8188423426	0.8332005833	0.8433763796	0.8670086759	0.8904368796	0.9430624907	0.9938224815	1.0243907315	1.0399818333	1.058495	1.0638962407	1.0822323796];
y_nnr3 = [0.4133311387	0.5150123642	0.5824028815	0.6560288092	0.733786922	0.7860389306	0.8181224509	0.8414744277	0.8563729884	0.8730744277	0.8985684884	0.9189314335	0.9424699624	0.9643002341	0.9812427052	0.9980938699	1.0106165578	1.0259578642	1.0368227081	1.0399883439	1.0427992775];

y_nn = [415.6378349031	280.7559367935	184.692696639	130.1537771834	105.9856061107	94.3360260358	87.6203564694	83.9384735582	80.9619782491	78.9190306228	77.5057184025	76.0368483391	73.9307121788	72.7894554544	71.6856118858	71.2230006713	70.4923835375	70.6246427682	71.829968639	72.0824564037	72.7184234083];
y_nn1 = [740.3669242082	488.3559634552	304.3402625908	201.5912350533	157.6187716634	137.1779439395	126.0791623535	119.7622792906	114.3675410073	110.7836322228	108.9888220073	106.8295502591	103.4250020605	101.1105247482	98.883587201	96.8347422324	94.6692898789	94.1805774722	95.8168001283	95.8163284649	95.8695130799];
y_nn2 = [236.2594333333	173.7762925463	132.1800480926	108.0360991296	95.6706179722	89.9294500185	84.2300500833	82.1957865093	81.456827	80.3671853241	78.2197528333	77.1162481759	74.7820513519	72.4291101389	71.5387835093	71.9229417778	72.7218425278	73.9219528333	75.6926688519	76.7335568241	80.4749427963];
y_nn3 = [84.0186253266	66.3483950809	58.2676137052	51.7868382514	47.5738180925	44.5735928439	42.7725710925	41.7216482225	40.9332467919	40.4320908295	39.7032978613	38.944475289	38.459364341	39.0967261936	39.2668074335	40.4332812948	40.9379214277	41.4780805723	41.9925899451	42.3009303728	42.6632091647];

st_v = 5;

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
step = 20;
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
title('SNN Segmentation - Overall - Metric B - Small clusters removed');
xlabel('Threshold [m]');
hold on
plot(x_nn(c),min_y_nn,'r*');
axes(AX(2));
lh=line([x_nn(1) x_nn(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_nn1==min(min(y_nn1)));
min_y_nn1 = min(y_nn1);
v_max = max(y_nn1);
step = 20;

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
title('SNN Segmentation - Scene 1 - Metric B - Small clusters removed');
xlabel('Threshold [m]');
hold on
plot(x_nn(c),min_y_nn1,'r*');
axes(AX(2));
lh=line([x_nn(1) x_nn(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_nn2==min(min(y_nn2)));
min_y_nn2 = min(y_nn2);

v_max = max(y_nn2);
step = 10;

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
title('SNN Segmentation - Scene 2 -Metric B - Small clusters removed');
xlabel('Threshold [m]');
hold on
plot(x_nn(c),min_y_nn2,'r*');
axes(AX(2));
lh=line([x_nn(1) x_nn(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_nn3==min(min(y_nn3)));
min_y_nn3 = min(y_nn3);
v_max = max(y_nn3);
step = 5;

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
title('SNN Segmentation - Scene 3 - Metric B - Small clusters removed');
xlabel('Threshold [m]');
hold on
plot(x_nn(c),min_y_nn3,'r*');
axes(AX(2));
lh=line([x_nn(1) x_nn(end)],[1 1] , 'Color','r');


%% MS SEGMENTATION - Metric B
% Ratios + Metrics

clear ll, close all, clc;

x_p = linspace(0.55, 0.97,21);

%Rations - ultimate boundaries
y_pr = [0.8774846344	0.8698991396	0.8643295006	0.8581383599	0.8508440565	0.8448296586	0.8391629804	0.8303822295	0.8230282526	0.8129415582	0.804683594	0.7964613576	0.7867366309	0.7755597728	0.7596366367	0.7409245386	0.7134355248	0.6755108997	0.6431821845	0.6044012618	0.5402888685];
y_pr1 = [0.9166214528	0.9088463656	0.9022981792	0.8942638814	0.8837576998	0.8744349201	0.8656780678	0.850352954	0.8386082494	0.8233401913	0.8103312446	0.7981647579	0.7833461913	0.7653374794	0.7439719128	0.7162405109	0.6739044165	0.6133758111	0.5650973341	0.5078646901	0.4203124165];
y_pr2 = [0.7760605556	0.7638733241	0.7556117222	0.744564787	0.7303704722	0.7238022593	0.7215535278	0.7190498148	0.714723963	0.7108231389	0.7091033056	0.7052024907	0.7013623796	0.6953129815	0.6873540648	0.6792330185	0.6652204259	0.6301402407	0.6072659167	0.5849974352	0.5495701389];
y_pr3 = [0.8624276821	0.8565048728	0.8529435347	0.8504681445	0.8491614335	0.8472689249	0.8442239335	0.8412955	0.8382372832	0.8324044306	0.8277766329	0.8229135347	0.817432211	0.812809659	0.8008968931	0.7896447341	0.7756713006	0.7638398671	0.7475983699	0.7256880173	0.680600711];

%Metric - ultimate boundaries
y_p = [79.9136202411	79.534622887	78.8832124106	78.2924267082	76.8690741615	75.5140236782	75.0481084729	74.960806293	74.9731596782	75.5683288397	76.0574348039	76.7082999008	77.3661587774	78.3675480438	79.969039887	82.0394227693	85.0263236251	89.8359100473	94.7873973206	100.8609119366	112.6986540565];
y_p1 = [94.783044816	94.6127417554	94.1293226271	94.1384029225	93.7450698692	93.9180953123	94.0499951162	94.8897828329	95.1812063584	96.176497293	97.2120416295	98.2482139031	99.6479295521	101.4996067942	104.1691627119	107.8991679153	113.2017432712	121.5937400605	130.1866751404	141.1719047385	162.2498174237];
y_p2 = [94.6945136574	95.2215368148	95.2903433148	95.9843616759	95.7680979537	94.4376488611	93.7743094815	91.0076751296	90.0078266481	90.3303705463	90.5437270926	90.9779856296	91.5009351574	92.0997018241	92.9079586481	93.7410409259	95.3070369722	99.5780916759	103.0947964537	105.8300993426	110.2498941759];
y_p3 = [57.5511669509	56.6402593121	55.5635197601	53.8556719306	50.826066078	47.639361526	46.5214931763	46.1639012486	46.1590923035	46.3617563671	46.2846828208	46.5432405318	46.3576987225	46.4698229798	47.0440284624	47.5196265087	48.1859613266	48.8875302457	49.9402850491	51.1929574075	54.3166760116];

st_v = 21;

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
vr_max = max( [y_pr1 y_pr1 y_pr2 y_pr3 ] );
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
title('Multivariable Segmentation - Overall - Metric B');
xlabel('Cosine distance threshold');
hold on
plot(x_p(c),min_y_p,'r*');
axes(AX(2));
lh=line([x_p(1) x_p(end)],[1 1] , 'Color','r');


%Find optimal values
c = find(y_p1==min(min(y_p1)));
min_y_p1 = min(y_p1);
v_max = max(y_p1);
step = 25;

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
title('Multivariable Segmentation - Scene 1 - Metric B');
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
title('Multivariable Segmentation - Scene 2 - Metric B');
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
title('Multivariable Segmentation - Scene 3 - Metric B');
xlabel('Cosine distance threshold');
hold on
plot(x_p(c),min_y_p3,'r*');
axes(AX(2));
lh=line([x_p(1) x_p(end)],[1 1] , 'Color','r');


%% MS Metric B  Small clusters removed 

% Ratios + Metrics

clear all, close all, clc;

x_p = linspace(0.55, 0.97,21);

y_pr = [0.8517919273	0.8375220946	0.827473714	0.8165620773	0.8054399112	0.7968691834	0.7876188604	0.7747467151	0.7628153979	0.7469467116	0.7342202849	0.7215773206	0.7065898847	0.6913949446	0.6660146563	0.638623083	0.6016096067	0.5543945594	0.5144032088	0.4682835363	0.3983829469];
y_pr1 = [0.8962111937	0.8853124286	0.8765025206	0.8662607918	0.8525596877	0.8405381816	0.8294403002	0.8104956731	0.7952009588	0.7764038087	0.7595285666	0.7455541162	0.7266673947	0.7056029322	0.6805566295	0.6489575811	0.599032523	0.5312743269	0.4792083947	0.4200879879	0.3352853801];
y_pr2 = [0.7363310556	0.7219528796	0.7126523148	0.6988033333	0.6807883611	0.67179375	0.6690952593	0.6656163704	0.6599762778	0.6546031019	0.6526642037	0.6473450741	0.6422230463	0.6356656296	0.6260983519	0.6165136574	0.5986536111	0.5589702037	0.5324349444	0.5069154815	0.4698211759];
y_pr3 = [0.834811052	0.8165511908	0.8047910954	0.7939966879	0.7881043613	0.7837849364	0.7746948555	0.766139078	0.7562587168	0.7406095116	0.729468078	0.7161283786	0.7027159162	0.6918309769	0.6611161763	0.633188604	0.6056084017	0.5805635954	0.5507848006	0.5137532225	0.4514001908];


%Results NO centers
y_p = [83.5067807197	83.5321632745	83.3842857393	83.2624769273	81.8697769666	81.7214013991	82.0693708316	82.1344791603	82.7675350669	84.0891383126	85.3364600819	86.5831808489	88.0042173114	89.7616684348	92.9245059504	96.5909814175	101.2061791015	108.9897408674	115.6809850127	125.6941442134	146.7794780334];
y_p1 = [96.8824070847	97.3393061138	97.9361830242	98.8975621211	100.3035114358	101.5720095545	102.5563337506	104.4640762567	106.206380092	108.443270615	110.4478845085	112.3186417942	114.7463349104	117.6624092252	122.3314132228	128.1931569153	135.7040262131	148.7848121283	159.1547902518	176.2675275835	211.8395965375];
y_p2 = [108.9746377778	110.6048580278	110.9012887593	110.71712325	110.7409395463	109.0617993519	108.1713411574	104.9755900648	103.8160788519	104.3310479167	104.6913427222	105.5243296667	105.2949825	106.6322313796	107.1501972593	107.947365213	110.1383789259	116.8950711019	121.5566343889	125.9644531574	134.3649610648];
y_p3 = [59.5915719017	58.6009464364	57.4254131821	56.0301301416	50.8546963468	49.4928923035	49.4678434306	48.3513474306	48.2198884653	48.7007484913	49.3210103324	49.9519974884	50.6865953208	51.1922269769	53.3828083642	55.3245423121	57.2400275462	59.0211278728	61.9548240405	65.2432751445	72.996064422];

st_v = 21;

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
vr_max = max( [y_pr1 y_pr1 y_pr2 y_pr3 ] );
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
title('Multivariable Segmentation - Overall - Metric B - Small clusters removed');
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
title('Multivariable Segmentation - Scene 1 - Metric B - Small clusters removed');
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
title('Multivariable Segmentation - Scene 2 - Metric B - Small clusters removed');
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
title('Multivariable Segmentation - Scene 3 - Metric B - Small clusters removed');
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

%Ultimate Boundaries --

yc = [235.3002208858	165.4198364844	119.0311948351	98.1758319839	88.8394352664	83.9863306667	81.6059281096	79.190140323	77.8231434487	77.1635536932	76.2972520173	75.2113198789	74.3951961592	73.519208812	73.1795926355	72.5625624925	72.0713917163	71.7823800704	71.9782832514	72.8804539942	73.8459854775];
yc1 = [405.121599155	271.036832707	180.9295057506	142.0643952203	124.5081746077	115.2400890993	110.8004212058	106.1051228886	103.5781712179	102.441621523	100.9694761646	98.8911551768	96.5521862397	94.4754299734	93.2804209976	91.510818276	90.7999232881	90.1440529395	89.9823928765	90.3198357409	91.1378020678];
yc2 = [163.9188061296	130.2866128333	109.6758792222	98.3370211852	93.5056988519	91.5008492685	90.6468743889	90.3279610463	90.1054878426	89.6456991019	89.1404387315	88.0908136296	87.76887775	86.8235073519	85.7547844074	84.6186062315	82.1587486296	81.5509783056	82.5489291296	83.1591716389	83.9365327222];
yc3 = [54.8752601012	50.3174339249	48.0669511301	45.7383029364	44.8072219277	44.3349715867	43.9361366445	43.5867517746	43.2470750578	43.0944389017	42.8386024855	42.9258941474	43.7732466936	44.3522042399	45.2612029682	46.1819776301	46.5675822197	46.8159653179	47.188320737	48.8556962861	50.0561029509];

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
step = 10;
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
title('Santos Approach - Overall - Metric B');
xlabel('C0 [m]');
hold on
plot(xc(c),min_y_c,'r*');


%Find optimal values
c = find(yc1==min(min(yc1)));
min_y_c1 = min(yc1);
v_max = max(yc1); 
step = 10;
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
title('Santos Approach - Scene 1 - Metric B');
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
title('Santos Approach - Scene 2 - Metric B');
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
title('Santos Approach - Scene 3 - Metric B');
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

yc = [348.9883192076	232.3653544002	156.8826602895	123.7257350704	108.8373655663	101.1883564279	96.9539255087	93.2190763287	90.5027395952	89.2113118581	87.4478267393	85.303379872	83.460716361	81.8903583276	80.7938280161	79.2335847958	78.1846805721	77.4950194913	77.2030613172	76.9587120923	77.1488378431];
yc1 = [608.4460383196	386.136537	240.3893270944	180.1185398741	152.6058551961	138.9490866368	131.3300150121	124.5194047022	119.7967497119	117.3552112155	113.6842327385	110.0812053487	106.3048358571	103.0822320751	100.8974727772	97.7082706513	96.1919895617	94.856525322	93.9341302978	93.407767908	93.7044970412];
yc2 = [213.1693605556	162.6103590556	131.1358642593	115.0549089815	108.1461185833	104.6270679167	103.5782141296	102.965905963	102.2495473611	101.6313044537	100.6523856019	98.966644963	98.330606037	96.8096183889	94.8303040093	92.8966434074	88.9822672778	88.2118048519	88.6934892778	88.1151640463	88.1363563981];
yc3 = [81.6831444711	70.5909066647	65.2421995405	59.1194369017	56.8092397197	55.0422222746	53.8534973699	52.8153387023	51.8695563092	51.7408215838	52.0093064075	51.4627019769	51.5515561387	51.9379769855	52.4159531792	52.9166611301	53.3200621416	53.4264798179	53.6455534884	53.8420737601	53.9576839335];

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
set(AX(1),'YLim',[0 (v_max+step)]);
set(AX(1),'YTick',[0:step:(v_max+step)]);
set(AX(2),'YLim',[st_r (vr_max+step_r/2)]);
set(AX(2),'YTick',[st_r:step_r:(vr_max+step_r)]);
title('Santos Approach - Overall - Metric B - Small clusters removed');
xlabel('C0 [m]');
hold on
plot(xc(c),min_y_c,'r*');


%Find optimal values
c = find(yc1==min(min(yc1)));
min_y_c1 = min(yc1);
v_max = max(yc1); 
step = 10;
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
title('Santos Approach - Scene 1 - Metric B - Small clusters removed');
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
title('Santos Approach - Scene 2 - Metric B - Small clusters removed');
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
title('Santos Approach - Scene 3 - Metric B - Small clusters removed');
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

yb = [91.5650590704	90.9761328916	90.5351260496	90.0444350035	89.5656910092	89.1307598489	88.7705704452	88.3967548235	88.2152888039	88.0835953552	87.8325351684	87.8768555779	87.6083558731	87.5069000081	87.0855450657	87.721245323	89.2812054833	90.9208830969	93.8519682595	98.0504983172	100.6508699331];
yb1 = [129.1484831768	128.2046197869	127.4413362349	126.615476092	125.8048791792	125.101549586	124.4111316416	123.5369233123	123.0298952203	122.21137654	121.2766984189	120.3693888765	119.1439444407	118.2820966731	116.7033815714	115.8466250436	114.5773008281	112.7182578305	113.2716434722	115.00479154	116.1227641138];
yb2 = [95.0940916481	94.8547478333	94.5997091574	94.0709618981	93.8694949167	93.4749839722	93.5056988519	93.3189362315	92.777978287	92.6543101296	92.4393883889	92.2899491389	92.1832018241	92.1768596759	92.1080358241	92.0242657778	92.3441569074	92.4830326759	101.1807327315	109.4522399352	112.9882899259];
yb3 = [45.6023721503	45.327995604	45.213623789	45.1348833439	44.9657039104	44.8385275751	44.7504963353	44.915583789	45.2349335607	45.920558289	46.474212711	47.7149181301	48.5381782977	49.3146691792	50.1647488844	52.8064822197	58.1306675318	64.4150220578	68.3842444855	74.254223185	78.3319865347];

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
step = 10;
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
title('Santos Approach - Overall - Metric B');
xlabel('Beta [degrees]');
hold on
plot(xb(c),min_y_b,'r*');


%Find optimal values
c = find(yb1==min(min(yb1)));
min_y_b1 = min(yb1);
v_max = max(yb1); 
step = 20;
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
title('Santos Approach - Scene 1 - Metric B');
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
title('Santos Approach - Scene 2 - Metric B');
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
title('Santos Approach - Scene 3 - Metric B');
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

yb = [112.6506047013	111.4903241292	110.897469564	110.5683347624	109.8268740727	109.1526534867	108.6430998731	108.1320835698	108.1187874083	107.8391622203	107.4091261176	107.2925055813	106.4165407855	105.8447436909	107.0386219896	107.7567342607	108.761353173	109.0726260992	111.9415952826	113.5371689873	117.292619925];
yb1 = [160.668490753	159.1235072954	157.8264017506	156.5705695182	154.7868579443	153.5420223341	152.4211044092	150.7772706465	150.1124767385	149.0727309709	147.7551269443	146.7683292881	144.933343431	143.6358280266	141.6491733874	141.344781109	141.4608666683	138.4493386295	135.6811241356	134.4707730533	133.7941347869];
yb2 = [110.9550866296	110.5265784352	110.2005470278	109.2937305556	108.8476117685	108.2751333611	108.1461185833	107.6991363796	107.2266674259	107.6960615741	108.4209434352	107.682342963	107.4557157037	107.2153741019	106.970152	106.6653511204	106.8661990463	107.5273383889	108.1002366111	108.5921458519	109.2171224537];
yb3 = [55.8636943324	54.9341966358	55.0986795087	56.0560061503	56.4664318468	56.4415634277	56.5429787919	57.3641530318	58.2718373064	58.6657257341	58.9346330202	60.0508361358	60.1169153988	60.3079057861	65.7473995376	68.005306604	70.3214039595	74.4897035491	84.8041424509	90.0934811821	100.1164005289];

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
step = 10;
step_r = 0.1;
st_r = 0.50;

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
title('Santos Approach - Overall - Metric B - Small clusters removed');
xlabel('Beta [degrees]');
hold on
plot(xb(c),min_y_b,'r*');


%Find optimal values
c = find(yb1==min(min(yb1)));
min_y_b1 = min(yb1);
v_max = max(yb1); 
step = 20;
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
title('Santos Approach - Scene 1 - Metric B - Small clusters removed');
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
title('Santos Approach - Scene 2 - Metric B - Small clusters removed');
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
title('Santos Approach - Scene 3 - Metric B - Small clusters removed');
xlabel('Beta [degrees]');
hold on
plot(xb(c),min_y_b3,'r*');

