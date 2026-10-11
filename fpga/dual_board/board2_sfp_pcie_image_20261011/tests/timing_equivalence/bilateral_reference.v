// Exact Gaussian range weights, with one registered ROM read per pixel.
// Runtime sigma selects a ROM address bank before the registered lookup.
module Bilateral_Range_LUT_reference#(
    parameter   RUNTIME_CONTROL                  =   0,
    parameter   SIGMA                            =   32
)(
    input                   clk                              ,
    input   [159 : 0]       cfg_data                         ,
    input   [7 : 0]         Pre_Data                         ,
    output reg  [9 : 0]     Post_Data
);

generate
    if(RUNTIME_CONTROL)begin : g_runtime_rom
        (* ram_style = "block" *)
        reg     [9 : 0]         range_table[0 : 767]             ;
        wire    [1 : 0]         sigma_bank                       ;
        wire    [9 : 0]         lookup_address                   ;

        assign  sigma_bank                       =   cfg_data[78 : 73] == 6'd16 ? 2'd0 :
                                                     cfg_data[78 : 73] == 6'd48 ? 2'd2 : 2'd1;
        assign  lookup_address                   =   {sigma_bank, Pre_Data};

        initial begin
            // Sigma 16.
            range_table[0  ]                 =   10'd1023;
            range_table[1  ]                 =   10'd1021;
            range_table[2  ]                 =   10'd1015;
            range_table[3  ]                 =   10'd1005;
            range_table[4  ]                 =   10'd991;
            range_table[5  ]                 =   10'd974;
            range_table[6  ]                 =   10'd953;
            range_table[7  ]                 =   10'd929;
            range_table[8  ]                 =   10'd902;
            range_table[9  ]                 =   10'd873;
            range_table[10 ]                 =   10'd841;
            range_table[11 ]                 =   10'd807;
            range_table[12 ]                 =   10'd772;
            range_table[13 ]                 =   10'd735;
            range_table[14 ]                 =   10'd697;
            range_table[15 ]                 =   10'd659;
            range_table[16 ]                 =   10'd620;
            range_table[17 ]                 =   10'd581;
            range_table[18 ]                 =   10'd543;
            range_table[19 ]                 =   10'd505;
            range_table[20 ]                 =   10'd468;
            range_table[21 ]                 =   10'd432;
            range_table[22 ]                 =   10'd397;
            range_table[23 ]                 =   10'd364;
            range_table[24 ]                 =   10'd332;
            range_table[25 ]                 =   10'd301;
            range_table[26 ]                 =   10'd273;
            range_table[27 ]                 =   10'd246;
            range_table[28 ]                 =   10'd221;
            range_table[29 ]                 =   10'd197;
            range_table[30 ]                 =   10'd176;
            range_table[31 ]                 =   10'd156;
            range_table[32 ]                 =   10'd138;
            range_table[33 ]                 =   10'd121;
            range_table[34 ]                 =   10'd106;
            range_table[35 ]                 =   10'd93;
            range_table[36 ]                 =   10'd81;
            range_table[37 ]                 =   10'd70;
            range_table[38 ]                 =   10'd60;
            range_table[39 ]                 =   10'd52;
            range_table[40 ]                 =   10'd44;
            range_table[41 ]                 =   10'd38;
            range_table[42 ]                 =   10'd32;
            range_table[43 ]                 =   10'd27;
            range_table[44 ]                 =   10'd23;
            range_table[45 ]                 =   10'd19;
            range_table[46 ]                 =   10'd16;
            range_table[47 ]                 =   10'd13;
            range_table[48 ]                 =   10'd11;
            range_table[49 ]                 =   10'd9;
            range_table[50 ]                 =   10'd7;
            range_table[51 ]                 =   10'd6;
            range_table[52 ]                 =   10'd5;
            range_table[53 ]                 =   10'd4;
            range_table[54 ]                 =   10'd3;
            range_table[55 ]                 =   10'd2;
            range_table[56 ]                 =   10'd2;
            range_table[57 ]                 =   10'd1;
            range_table[58 ]                 =   10'd1;
            range_table[59 ]                 =   10'd1;
            range_table[60 ]                 =   10'd0;
            range_table[61 ]                 =   10'd0;
            range_table[62 ]                 =   10'd0;
            range_table[63 ]                 =   10'd0;
            range_table[64 ]                 =   10'd0;
            range_table[65 ]                 =   10'd0;
            range_table[66 ]                 =   10'd0;
            range_table[67 ]                 =   10'd0;
            range_table[68 ]                 =   10'd0;
            range_table[69 ]                 =   10'd0;
            range_table[70 ]                 =   10'd0;
            range_table[71 ]                 =   10'd0;
            range_table[72 ]                 =   10'd0;
            range_table[73 ]                 =   10'd0;
            range_table[74 ]                 =   10'd0;
            range_table[75 ]                 =   10'd0;
            range_table[76 ]                 =   10'd0;
            range_table[77 ]                 =   10'd0;
            range_table[78 ]                 =   10'd0;
            range_table[79 ]                 =   10'd0;
            range_table[80 ]                 =   10'd0;
            range_table[81 ]                 =   10'd0;
            range_table[82 ]                 =   10'd0;
            range_table[83 ]                 =   10'd0;
            range_table[84 ]                 =   10'd0;
            range_table[85 ]                 =   10'd0;
            range_table[86 ]                 =   10'd0;
            range_table[87 ]                 =   10'd0;
            range_table[88 ]                 =   10'd0;
            range_table[89 ]                 =   10'd0;
            range_table[90 ]                 =   10'd0;
            range_table[91 ]                 =   10'd0;
            range_table[92 ]                 =   10'd0;
            range_table[93 ]                 =   10'd0;
            range_table[94 ]                 =   10'd0;
            range_table[95 ]                 =   10'd0;
            range_table[96 ]                 =   10'd0;
            range_table[97 ]                 =   10'd0;
            range_table[98 ]                 =   10'd0;
            range_table[99 ]                 =   10'd0;
            range_table[100]                 =   10'd0;
            range_table[101]                 =   10'd0;
            range_table[102]                 =   10'd0;
            range_table[103]                 =   10'd0;
            range_table[104]                 =   10'd0;
            range_table[105]                 =   10'd0;
            range_table[106]                 =   10'd0;
            range_table[107]                 =   10'd0;
            range_table[108]                 =   10'd0;
            range_table[109]                 =   10'd0;
            range_table[110]                 =   10'd0;
            range_table[111]                 =   10'd0;
            range_table[112]                 =   10'd0;
            range_table[113]                 =   10'd0;
            range_table[114]                 =   10'd0;
            range_table[115]                 =   10'd0;
            range_table[116]                 =   10'd0;
            range_table[117]                 =   10'd0;
            range_table[118]                 =   10'd0;
            range_table[119]                 =   10'd0;
            range_table[120]                 =   10'd0;
            range_table[121]                 =   10'd0;
            range_table[122]                 =   10'd0;
            range_table[123]                 =   10'd0;
            range_table[124]                 =   10'd0;
            range_table[125]                 =   10'd0;
            range_table[126]                 =   10'd0;
            range_table[127]                 =   10'd0;
            range_table[128]                 =   10'd0;
            range_table[129]                 =   10'd0;
            range_table[130]                 =   10'd0;
            range_table[131]                 =   10'd0;
            range_table[132]                 =   10'd0;
            range_table[133]                 =   10'd0;
            range_table[134]                 =   10'd0;
            range_table[135]                 =   10'd0;
            range_table[136]                 =   10'd0;
            range_table[137]                 =   10'd0;
            range_table[138]                 =   10'd0;
            range_table[139]                 =   10'd0;
            range_table[140]                 =   10'd0;
            range_table[141]                 =   10'd0;
            range_table[142]                 =   10'd0;
            range_table[143]                 =   10'd0;
            range_table[144]                 =   10'd0;
            range_table[145]                 =   10'd0;
            range_table[146]                 =   10'd0;
            range_table[147]                 =   10'd0;
            range_table[148]                 =   10'd0;
            range_table[149]                 =   10'd0;
            range_table[150]                 =   10'd0;
            range_table[151]                 =   10'd0;
            range_table[152]                 =   10'd0;
            range_table[153]                 =   10'd0;
            range_table[154]                 =   10'd0;
            range_table[155]                 =   10'd0;
            range_table[156]                 =   10'd0;
            range_table[157]                 =   10'd0;
            range_table[158]                 =   10'd0;
            range_table[159]                 =   10'd0;
            range_table[160]                 =   10'd0;
            range_table[161]                 =   10'd0;
            range_table[162]                 =   10'd0;
            range_table[163]                 =   10'd0;
            range_table[164]                 =   10'd0;
            range_table[165]                 =   10'd0;
            range_table[166]                 =   10'd0;
            range_table[167]                 =   10'd0;
            range_table[168]                 =   10'd0;
            range_table[169]                 =   10'd0;
            range_table[170]                 =   10'd0;
            range_table[171]                 =   10'd0;
            range_table[172]                 =   10'd0;
            range_table[173]                 =   10'd0;
            range_table[174]                 =   10'd0;
            range_table[175]                 =   10'd0;
            range_table[176]                 =   10'd0;
            range_table[177]                 =   10'd0;
            range_table[178]                 =   10'd0;
            range_table[179]                 =   10'd0;
            range_table[180]                 =   10'd0;
            range_table[181]                 =   10'd0;
            range_table[182]                 =   10'd0;
            range_table[183]                 =   10'd0;
            range_table[184]                 =   10'd0;
            range_table[185]                 =   10'd0;
            range_table[186]                 =   10'd0;
            range_table[187]                 =   10'd0;
            range_table[188]                 =   10'd0;
            range_table[189]                 =   10'd0;
            range_table[190]                 =   10'd0;
            range_table[191]                 =   10'd0;
            range_table[192]                 =   10'd0;
            range_table[193]                 =   10'd0;
            range_table[194]                 =   10'd0;
            range_table[195]                 =   10'd0;
            range_table[196]                 =   10'd0;
            range_table[197]                 =   10'd0;
            range_table[198]                 =   10'd0;
            range_table[199]                 =   10'd0;
            range_table[200]                 =   10'd0;
            range_table[201]                 =   10'd0;
            range_table[202]                 =   10'd0;
            range_table[203]                 =   10'd0;
            range_table[204]                 =   10'd0;
            range_table[205]                 =   10'd0;
            range_table[206]                 =   10'd0;
            range_table[207]                 =   10'd0;
            range_table[208]                 =   10'd0;
            range_table[209]                 =   10'd0;
            range_table[210]                 =   10'd0;
            range_table[211]                 =   10'd0;
            range_table[212]                 =   10'd0;
            range_table[213]                 =   10'd0;
            range_table[214]                 =   10'd0;
            range_table[215]                 =   10'd0;
            range_table[216]                 =   10'd0;
            range_table[217]                 =   10'd0;
            range_table[218]                 =   10'd0;
            range_table[219]                 =   10'd0;
            range_table[220]                 =   10'd0;
            range_table[221]                 =   10'd0;
            range_table[222]                 =   10'd0;
            range_table[223]                 =   10'd0;
            range_table[224]                 =   10'd0;
            range_table[225]                 =   10'd0;
            range_table[226]                 =   10'd0;
            range_table[227]                 =   10'd0;
            range_table[228]                 =   10'd0;
            range_table[229]                 =   10'd0;
            range_table[230]                 =   10'd0;
            range_table[231]                 =   10'd0;
            range_table[232]                 =   10'd0;
            range_table[233]                 =   10'd0;
            range_table[234]                 =   10'd0;
            range_table[235]                 =   10'd0;
            range_table[236]                 =   10'd0;
            range_table[237]                 =   10'd0;
            range_table[238]                 =   10'd0;
            range_table[239]                 =   10'd0;
            range_table[240]                 =   10'd0;
            range_table[241]                 =   10'd0;
            range_table[242]                 =   10'd0;
            range_table[243]                 =   10'd0;
            range_table[244]                 =   10'd0;
            range_table[245]                 =   10'd0;
            range_table[246]                 =   10'd0;
            range_table[247]                 =   10'd0;
            range_table[248]                 =   10'd0;
            range_table[249]                 =   10'd0;
            range_table[250]                 =   10'd0;
            range_table[251]                 =   10'd0;
            range_table[252]                 =   10'd0;
            range_table[253]                 =   10'd0;
            range_table[254]                 =   10'd0;
            range_table[255]                 =   10'd0;
            // Sigma 32.
            range_table[256]                 =   10'd1023;
            range_table[257]                 =   10'd1022;
            range_table[258]                 =   10'd1021;
            range_table[259]                 =   10'd1018;
            range_table[260]                 =   10'd1015;
            range_table[261]                 =   10'd1010;
            range_table[262]                 =   10'd1005;
            range_table[263]                 =   10'd998;
            range_table[264]                 =   10'd991;
            range_table[265]                 =   10'd983;
            range_table[266]                 =   10'd974;
            range_table[267]                 =   10'd964;
            range_table[268]                 =   10'd953;
            range_table[269]                 =   10'd941;
            range_table[270]                 =   10'd929;
            range_table[271]                 =   10'd916;
            range_table[272]                 =   10'd902;
            range_table[273]                 =   10'd888;
            range_table[274]                 =   10'd873;
            range_table[275]                 =   10'd857;
            range_table[276]                 =   10'd841;
            range_table[277]                 =   10'd824;
            range_table[278]                 =   10'd807;
            range_table[279]                 =   10'd790;
            range_table[280]                 =   10'd772;
            range_table[281]                 =   10'd753;
            range_table[282]                 =   10'd735;
            range_table[283]                 =   10'd716;
            range_table[284]                 =   10'd697;
            range_table[285]                 =   10'd678;
            range_table[286]                 =   10'd659;
            range_table[287]                 =   10'd639;
            range_table[288]                 =   10'd620;
            range_table[289]                 =   10'd601;
            range_table[290]                 =   10'd581;
            range_table[291]                 =   10'd562;
            range_table[292]                 =   10'd543;
            range_table[293]                 =   10'd524;
            range_table[294]                 =   10'd505;
            range_table[295]                 =   10'd486;
            range_table[296]                 =   10'd468;
            range_table[297]                 =   10'd450;
            range_table[298]                 =   10'd432;
            range_table[299]                 =   10'd414;
            range_table[300]                 =   10'd397;
            range_table[301]                 =   10'd380;
            range_table[302]                 =   10'd364;
            range_table[303]                 =   10'd347;
            range_table[304]                 =   10'd332;
            range_table[305]                 =   10'd316;
            range_table[306]                 =   10'd301;
            range_table[307]                 =   10'd287;
            range_table[308]                 =   10'd273;
            range_table[309]                 =   10'd259;
            range_table[310]                 =   10'd246;
            range_table[311]                 =   10'd233;
            range_table[312]                 =   10'd221;
            range_table[313]                 =   10'd209;
            range_table[314]                 =   10'd197;
            range_table[315]                 =   10'd186;
            range_table[316]                 =   10'd176;
            range_table[317]                 =   10'd166;
            range_table[318]                 =   10'd156;
            range_table[319]                 =   10'd147;
            range_table[320]                 =   10'd138;
            range_table[321]                 =   10'd129;
            range_table[322]                 =   10'd121;
            range_table[323]                 =   10'd114;
            range_table[324]                 =   10'd106;
            range_table[325]                 =   10'd100;
            range_table[326]                 =   10'd93;
            range_table[327]                 =   10'd87;
            range_table[328]                 =   10'd81;
            range_table[329]                 =   10'd75;
            range_table[330]                 =   10'd70;
            range_table[331]                 =   10'd65;
            range_table[332]                 =   10'd60;
            range_table[333]                 =   10'd56;
            range_table[334]                 =   10'd52;
            range_table[335]                 =   10'd48;
            range_table[336]                 =   10'd44;
            range_table[337]                 =   10'd41;
            range_table[338]                 =   10'd38;
            range_table[339]                 =   10'd35;
            range_table[340]                 =   10'd32;
            range_table[341]                 =   10'd30;
            range_table[342]                 =   10'd27;
            range_table[343]                 =   10'd25;
            range_table[344]                 =   10'd23;
            range_table[345]                 =   10'd21;
            range_table[346]                 =   10'd19;
            range_table[347]                 =   10'd17;
            range_table[348]                 =   10'd16;
            range_table[349]                 =   10'd14;
            range_table[350]                 =   10'd13;
            range_table[351]                 =   10'd12;
            range_table[352]                 =   10'd11;
            range_table[353]                 =   10'd10;
            range_table[354]                 =   10'd9;
            range_table[355]                 =   10'd8;
            range_table[356]                 =   10'd7;
            range_table[357]                 =   10'd7;
            range_table[358]                 =   10'd6;
            range_table[359]                 =   10'd5;
            range_table[360]                 =   10'd5;
            range_table[361]                 =   10'd4;
            range_table[362]                 =   10'd4;
            range_table[363]                 =   10'd3;
            range_table[364]                 =   10'd3;
            range_table[365]                 =   10'd3;
            range_table[366]                 =   10'd2;
            range_table[367]                 =   10'd2;
            range_table[368]                 =   10'd2;
            range_table[369]                 =   10'd2;
            range_table[370]                 =   10'd1;
            range_table[371]                 =   10'd1;
            range_table[372]                 =   10'd1;
            range_table[373]                 =   10'd1;
            range_table[374]                 =   10'd1;
            range_table[375]                 =   10'd1;
            range_table[376]                 =   10'd0;
            range_table[377]                 =   10'd0;
            range_table[378]                 =   10'd0;
            range_table[379]                 =   10'd0;
            range_table[380]                 =   10'd0;
            range_table[381]                 =   10'd0;
            range_table[382]                 =   10'd0;
            range_table[383]                 =   10'd0;
            range_table[384]                 =   10'd0;
            range_table[385]                 =   10'd0;
            range_table[386]                 =   10'd0;
            range_table[387]                 =   10'd0;
            range_table[388]                 =   10'd0;
            range_table[389]                 =   10'd0;
            range_table[390]                 =   10'd0;
            range_table[391]                 =   10'd0;
            range_table[392]                 =   10'd0;
            range_table[393]                 =   10'd0;
            range_table[394]                 =   10'd0;
            range_table[395]                 =   10'd0;
            range_table[396]                 =   10'd0;
            range_table[397]                 =   10'd0;
            range_table[398]                 =   10'd0;
            range_table[399]                 =   10'd0;
            range_table[400]                 =   10'd0;
            range_table[401]                 =   10'd0;
            range_table[402]                 =   10'd0;
            range_table[403]                 =   10'd0;
            range_table[404]                 =   10'd0;
            range_table[405]                 =   10'd0;
            range_table[406]                 =   10'd0;
            range_table[407]                 =   10'd0;
            range_table[408]                 =   10'd0;
            range_table[409]                 =   10'd0;
            range_table[410]                 =   10'd0;
            range_table[411]                 =   10'd0;
            range_table[412]                 =   10'd0;
            range_table[413]                 =   10'd0;
            range_table[414]                 =   10'd0;
            range_table[415]                 =   10'd0;
            range_table[416]                 =   10'd0;
            range_table[417]                 =   10'd0;
            range_table[418]                 =   10'd0;
            range_table[419]                 =   10'd0;
            range_table[420]                 =   10'd0;
            range_table[421]                 =   10'd0;
            range_table[422]                 =   10'd0;
            range_table[423]                 =   10'd0;
            range_table[424]                 =   10'd0;
            range_table[425]                 =   10'd0;
            range_table[426]                 =   10'd0;
            range_table[427]                 =   10'd0;
            range_table[428]                 =   10'd0;
            range_table[429]                 =   10'd0;
            range_table[430]                 =   10'd0;
            range_table[431]                 =   10'd0;
            range_table[432]                 =   10'd0;
            range_table[433]                 =   10'd0;
            range_table[434]                 =   10'd0;
            range_table[435]                 =   10'd0;
            range_table[436]                 =   10'd0;
            range_table[437]                 =   10'd0;
            range_table[438]                 =   10'd0;
            range_table[439]                 =   10'd0;
            range_table[440]                 =   10'd0;
            range_table[441]                 =   10'd0;
            range_table[442]                 =   10'd0;
            range_table[443]                 =   10'd0;
            range_table[444]                 =   10'd0;
            range_table[445]                 =   10'd0;
            range_table[446]                 =   10'd0;
            range_table[447]                 =   10'd0;
            range_table[448]                 =   10'd0;
            range_table[449]                 =   10'd0;
            range_table[450]                 =   10'd0;
            range_table[451]                 =   10'd0;
            range_table[452]                 =   10'd0;
            range_table[453]                 =   10'd0;
            range_table[454]                 =   10'd0;
            range_table[455]                 =   10'd0;
            range_table[456]                 =   10'd0;
            range_table[457]                 =   10'd0;
            range_table[458]                 =   10'd0;
            range_table[459]                 =   10'd0;
            range_table[460]                 =   10'd0;
            range_table[461]                 =   10'd0;
            range_table[462]                 =   10'd0;
            range_table[463]                 =   10'd0;
            range_table[464]                 =   10'd0;
            range_table[465]                 =   10'd0;
            range_table[466]                 =   10'd0;
            range_table[467]                 =   10'd0;
            range_table[468]                 =   10'd0;
            range_table[469]                 =   10'd0;
            range_table[470]                 =   10'd0;
            range_table[471]                 =   10'd0;
            range_table[472]                 =   10'd0;
            range_table[473]                 =   10'd0;
            range_table[474]                 =   10'd0;
            range_table[475]                 =   10'd0;
            range_table[476]                 =   10'd0;
            range_table[477]                 =   10'd0;
            range_table[478]                 =   10'd0;
            range_table[479]                 =   10'd0;
            range_table[480]                 =   10'd0;
            range_table[481]                 =   10'd0;
            range_table[482]                 =   10'd0;
            range_table[483]                 =   10'd0;
            range_table[484]                 =   10'd0;
            range_table[485]                 =   10'd0;
            range_table[486]                 =   10'd0;
            range_table[487]                 =   10'd0;
            range_table[488]                 =   10'd0;
            range_table[489]                 =   10'd0;
            range_table[490]                 =   10'd0;
            range_table[491]                 =   10'd0;
            range_table[492]                 =   10'd0;
            range_table[493]                 =   10'd0;
            range_table[494]                 =   10'd0;
            range_table[495]                 =   10'd0;
            range_table[496]                 =   10'd0;
            range_table[497]                 =   10'd0;
            range_table[498]                 =   10'd0;
            range_table[499]                 =   10'd0;
            range_table[500]                 =   10'd0;
            range_table[501]                 =   10'd0;
            range_table[502]                 =   10'd0;
            range_table[503]                 =   10'd0;
            range_table[504]                 =   10'd0;
            range_table[505]                 =   10'd0;
            range_table[506]                 =   10'd0;
            range_table[507]                 =   10'd0;
            range_table[508]                 =   10'd0;
            range_table[509]                 =   10'd0;
            range_table[510]                 =   10'd0;
            range_table[511]                 =   10'd0;
            // Sigma 48.
            range_table[512]                 =   10'd1023;
            range_table[513]                 =   10'd1022;
            range_table[514]                 =   10'd1022;
            range_table[515]                 =   10'd1021;
            range_table[516]                 =   10'd1019;
            range_table[517]                 =   10'd1017;
            range_table[518]                 =   10'd1015;
            range_table[519]                 =   10'd1012;
            range_table[520]                 =   10'd1008;
            range_table[521]                 =   10'd1005;
            range_table[522]                 =   10'd1001;
            range_table[523]                 =   10'd996;
            range_table[524]                 =   10'd991;
            range_table[525]                 =   10'd986;
            range_table[526]                 =   10'd980;
            range_table[527]                 =   10'd974;
            range_table[528]                 =   10'd967;
            range_table[529]                 =   10'd960;
            range_table[530]                 =   10'd953;
            range_table[531]                 =   10'd945;
            range_table[532]                 =   10'd937;
            range_table[533]                 =   10'd929;
            range_table[534]                 =   10'd921;
            range_table[535]                 =   10'd912;
            range_table[536]                 =   10'd902;
            range_table[537]                 =   10'd893;
            range_table[538]                 =   10'd883;
            range_table[539]                 =   10'd873;
            range_table[540]                 =   10'd862;
            range_table[541]                 =   10'd852;
            range_table[542]                 =   10'd841;
            range_table[543]                 =   10'd830;
            range_table[544]                 =   10'd819;
            range_table[545]                 =   10'd807;
            range_table[546]                 =   10'd796;
            range_table[547]                 =   10'd784;
            range_table[548]                 =   10'd772;
            range_table[549]                 =   10'd760;
            range_table[550]                 =   10'd747;
            range_table[551]                 =   10'd735;
            range_table[552]                 =   10'd722;
            range_table[553]                 =   10'd710;
            range_table[554]                 =   10'd697;
            range_table[555]                 =   10'd684;
            range_table[556]                 =   10'd672;
            range_table[557]                 =   10'd659;
            range_table[558]                 =   10'd646;
            range_table[559]                 =   10'd633;
            range_table[560]                 =   10'd620;
            range_table[561]                 =   10'd607;
            range_table[562]                 =   10'd594;
            range_table[563]                 =   10'd581;
            range_table[564]                 =   10'd568;
            range_table[565]                 =   10'd556;
            range_table[566]                 =   10'd543;
            range_table[567]                 =   10'd530;
            range_table[568]                 =   10'd517;
            range_table[569]                 =   10'd505;
            range_table[570]                 =   10'd492;
            range_table[571]                 =   10'd480;
            range_table[572]                 =   10'd468;
            range_table[573]                 =   10'd456;
            range_table[574]                 =   10'd444;
            range_table[575]                 =   10'd432;
            range_table[576]                 =   10'd420;
            range_table[577]                 =   10'd408;
            range_table[578]                 =   10'd397;
            range_table[579]                 =   10'd386;
            range_table[580]                 =   10'd375;
            range_table[581]                 =   10'd364;
            range_table[582]                 =   10'd353;
            range_table[583]                 =   10'd342;
            range_table[584]                 =   10'd332;
            range_table[585]                 =   10'd321;
            range_table[586]                 =   10'd311;
            range_table[587]                 =   10'd301;
            range_table[588]                 =   10'd292;
            range_table[589]                 =   10'd282;
            range_table[590]                 =   10'd273;
            range_table[591]                 =   10'd264;
            range_table[592]                 =   10'd255;
            range_table[593]                 =   10'd246;
            range_table[594]                 =   10'd237;
            range_table[595]                 =   10'd229;
            range_table[596]                 =   10'd221;
            range_table[597]                 =   10'd213;
            range_table[598]                 =   10'd205;
            range_table[599]                 =   10'd197;
            range_table[600]                 =   10'd190;
            range_table[601]                 =   10'd183;
            range_table[602]                 =   10'd176;
            range_table[603]                 =   10'd169;
            range_table[604]                 =   10'd162;
            range_table[605]                 =   10'd156;
            range_table[606]                 =   10'd150;
            range_table[607]                 =   10'd144;
            range_table[608]                 =   10'd138;
            range_table[609]                 =   10'd132;
            range_table[610]                 =   10'd127;
            range_table[611]                 =   10'd121;
            range_table[612]                 =   10'd116;
            range_table[613]                 =   10'd111;
            range_table[614]                 =   10'd106;
            range_table[615]                 =   10'd102;
            range_table[616]                 =   10'd97;
            range_table[617]                 =   10'd93;
            range_table[618]                 =   10'd89;
            range_table[619]                 =   10'd85;
            range_table[620]                 =   10'd81;
            range_table[621]                 =   10'd77;
            range_table[622]                 =   10'd74;
            range_table[623]                 =   10'd70;
            range_table[624]                 =   10'd67;
            range_table[625]                 =   10'd64;
            range_table[626]                 =   10'd60;
            range_table[627]                 =   10'd58;
            range_table[628]                 =   10'd55;
            range_table[629]                 =   10'd52;
            range_table[630]                 =   10'd49;
            range_table[631]                 =   10'd47;
            range_table[632]                 =   10'd44;
            range_table[633]                 =   10'd42;
            range_table[634]                 =   10'd40;
            range_table[635]                 =   10'd38;
            range_table[636]                 =   10'd36;
            range_table[637]                 =   10'd34;
            range_table[638]                 =   10'd32;
            range_table[639]                 =   10'd30;
            range_table[640]                 =   10'd29;
            range_table[641]                 =   10'd27;
            range_table[642]                 =   10'd26;
            range_table[643]                 =   10'd24;
            range_table[644]                 =   10'd23;
            range_table[645]                 =   10'd22;
            range_table[646]                 =   10'd20;
            range_table[647]                 =   10'd19;
            range_table[648]                 =   10'd18;
            range_table[649]                 =   10'd17;
            range_table[650]                 =   10'd16;
            range_table[651]                 =   10'd15;
            range_table[652]                 =   10'd14;
            range_table[653]                 =   10'd13;
            range_table[654]                 =   10'd12;
            range_table[655]                 =   10'd12;
            range_table[656]                 =   10'd11;
            range_table[657]                 =   10'd10;
            range_table[658]                 =   10'd10;
            range_table[659]                 =   10'd9;
            range_table[660]                 =   10'd8;
            range_table[661]                 =   10'd8;
            range_table[662]                 =   10'd7;
            range_table[663]                 =   10'd7;
            range_table[664]                 =   10'd6;
            range_table[665]                 =   10'd6;
            range_table[666]                 =   10'd5;
            range_table[667]                 =   10'd5;
            range_table[668]                 =   10'd5;
            range_table[669]                 =   10'd4;
            range_table[670]                 =   10'd4;
            range_table[671]                 =   10'd4;
            range_table[672]                 =   10'd3;
            range_table[673]                 =   10'd3;
            range_table[674]                 =   10'd3;
            range_table[675]                 =   10'd3;
            range_table[676]                 =   10'd2;
            range_table[677]                 =   10'd2;
            range_table[678]                 =   10'd2;
            range_table[679]                 =   10'd2;
            range_table[680]                 =   10'd2;
            range_table[681]                 =   10'd2;
            range_table[682]                 =   10'd1;
            range_table[683]                 =   10'd1;
            range_table[684]                 =   10'd1;
            range_table[685]                 =   10'd1;
            range_table[686]                 =   10'd1;
            range_table[687]                 =   10'd1;
            range_table[688]                 =   10'd1;
            range_table[689]                 =   10'd1;
            range_table[690]                 =   10'd1;
            range_table[691]                 =   10'd0;
            range_table[692]                 =   10'd0;
            range_table[693]                 =   10'd0;
            range_table[694]                 =   10'd0;
            range_table[695]                 =   10'd0;
            range_table[696]                 =   10'd0;
            range_table[697]                 =   10'd0;
            range_table[698]                 =   10'd0;
            range_table[699]                 =   10'd0;
            range_table[700]                 =   10'd0;
            range_table[701]                 =   10'd0;
            range_table[702]                 =   10'd0;
            range_table[703]                 =   10'd0;
            range_table[704]                 =   10'd0;
            range_table[705]                 =   10'd0;
            range_table[706]                 =   10'd0;
            range_table[707]                 =   10'd0;
            range_table[708]                 =   10'd0;
            range_table[709]                 =   10'd0;
            range_table[710]                 =   10'd0;
            range_table[711]                 =   10'd0;
            range_table[712]                 =   10'd0;
            range_table[713]                 =   10'd0;
            range_table[714]                 =   10'd0;
            range_table[715]                 =   10'd0;
            range_table[716]                 =   10'd0;
            range_table[717]                 =   10'd0;
            range_table[718]                 =   10'd0;
            range_table[719]                 =   10'd0;
            range_table[720]                 =   10'd0;
            range_table[721]                 =   10'd0;
            range_table[722]                 =   10'd0;
            range_table[723]                 =   10'd0;
            range_table[724]                 =   10'd0;
            range_table[725]                 =   10'd0;
            range_table[726]                 =   10'd0;
            range_table[727]                 =   10'd0;
            range_table[728]                 =   10'd0;
            range_table[729]                 =   10'd0;
            range_table[730]                 =   10'd0;
            range_table[731]                 =   10'd0;
            range_table[732]                 =   10'd0;
            range_table[733]                 =   10'd0;
            range_table[734]                 =   10'd0;
            range_table[735]                 =   10'd0;
            range_table[736]                 =   10'd0;
            range_table[737]                 =   10'd0;
            range_table[738]                 =   10'd0;
            range_table[739]                 =   10'd0;
            range_table[740]                 =   10'd0;
            range_table[741]                 =   10'd0;
            range_table[742]                 =   10'd0;
            range_table[743]                 =   10'd0;
            range_table[744]                 =   10'd0;
            range_table[745]                 =   10'd0;
            range_table[746]                 =   10'd0;
            range_table[747]                 =   10'd0;
            range_table[748]                 =   10'd0;
            range_table[749]                 =   10'd0;
            range_table[750]                 =   10'd0;
            range_table[751]                 =   10'd0;
            range_table[752]                 =   10'd0;
            range_table[753]                 =   10'd0;
            range_table[754]                 =   10'd0;
            range_table[755]                 =   10'd0;
            range_table[756]                 =   10'd0;
            range_table[757]                 =   10'd0;
            range_table[758]                 =   10'd0;
            range_table[759]                 =   10'd0;
            range_table[760]                 =   10'd0;
            range_table[761]                 =   10'd0;
            range_table[762]                 =   10'd0;
            range_table[763]                 =   10'd0;
            range_table[764]                 =   10'd0;
            range_table[765]                 =   10'd0;
            range_table[766]                 =   10'd0;
            range_table[767]                 =   10'd0;
        end

        always@(posedge clk)begin
            Post_Data                        <=  range_table[lookup_address];
        end
    end
    else begin : g_static_rom
        (* ram_style = "block" *)
        reg     [9 : 0]         range_table[0 : 255]             ;

        initial begin
            range_table[0  ]                 =   SIGMA == 16 ? 10'd1023 :
                                                     SIGMA == 48 ? 10'd1023 : 10'd1023;
            range_table[1  ]                 =   SIGMA == 16 ? 10'd1021 :
                                                     SIGMA == 48 ? 10'd1022 : 10'd1022;
            range_table[2  ]                 =   SIGMA == 16 ? 10'd1015 :
                                                     SIGMA == 48 ? 10'd1022 : 10'd1021;
            range_table[3  ]                 =   SIGMA == 16 ? 10'd1005 :
                                                     SIGMA == 48 ? 10'd1021 : 10'd1018;
            range_table[4  ]                 =   SIGMA == 16 ? 10'd991 :
                                                     SIGMA == 48 ? 10'd1019 : 10'd1015;
            range_table[5  ]                 =   SIGMA == 16 ? 10'd974 :
                                                     SIGMA == 48 ? 10'd1017 : 10'd1010;
            range_table[6  ]                 =   SIGMA == 16 ? 10'd953 :
                                                     SIGMA == 48 ? 10'd1015 : 10'd1005;
            range_table[7  ]                 =   SIGMA == 16 ? 10'd929 :
                                                     SIGMA == 48 ? 10'd1012 : 10'd998;
            range_table[8  ]                 =   SIGMA == 16 ? 10'd902 :
                                                     SIGMA == 48 ? 10'd1008 : 10'd991;
            range_table[9  ]                 =   SIGMA == 16 ? 10'd873 :
                                                     SIGMA == 48 ? 10'd1005 : 10'd983;
            range_table[10 ]                 =   SIGMA == 16 ? 10'd841 :
                                                     SIGMA == 48 ? 10'd1001 : 10'd974;
            range_table[11 ]                 =   SIGMA == 16 ? 10'd807 :
                                                     SIGMA == 48 ? 10'd996 : 10'd964;
            range_table[12 ]                 =   SIGMA == 16 ? 10'd772 :
                                                     SIGMA == 48 ? 10'd991 : 10'd953;
            range_table[13 ]                 =   SIGMA == 16 ? 10'd735 :
                                                     SIGMA == 48 ? 10'd986 : 10'd941;
            range_table[14 ]                 =   SIGMA == 16 ? 10'd697 :
                                                     SIGMA == 48 ? 10'd980 : 10'd929;
            range_table[15 ]                 =   SIGMA == 16 ? 10'd659 :
                                                     SIGMA == 48 ? 10'd974 : 10'd916;
            range_table[16 ]                 =   SIGMA == 16 ? 10'd620 :
                                                     SIGMA == 48 ? 10'd967 : 10'd902;
            range_table[17 ]                 =   SIGMA == 16 ? 10'd581 :
                                                     SIGMA == 48 ? 10'd960 : 10'd888;
            range_table[18 ]                 =   SIGMA == 16 ? 10'd543 :
                                                     SIGMA == 48 ? 10'd953 : 10'd873;
            range_table[19 ]                 =   SIGMA == 16 ? 10'd505 :
                                                     SIGMA == 48 ? 10'd945 : 10'd857;
            range_table[20 ]                 =   SIGMA == 16 ? 10'd468 :
                                                     SIGMA == 48 ? 10'd937 : 10'd841;
            range_table[21 ]                 =   SIGMA == 16 ? 10'd432 :
                                                     SIGMA == 48 ? 10'd929 : 10'd824;
            range_table[22 ]                 =   SIGMA == 16 ? 10'd397 :
                                                     SIGMA == 48 ? 10'd921 : 10'd807;
            range_table[23 ]                 =   SIGMA == 16 ? 10'd364 :
                                                     SIGMA == 48 ? 10'd912 : 10'd790;
            range_table[24 ]                 =   SIGMA == 16 ? 10'd332 :
                                                     SIGMA == 48 ? 10'd902 : 10'd772;
            range_table[25 ]                 =   SIGMA == 16 ? 10'd301 :
                                                     SIGMA == 48 ? 10'd893 : 10'd753;
            range_table[26 ]                 =   SIGMA == 16 ? 10'd273 :
                                                     SIGMA == 48 ? 10'd883 : 10'd735;
            range_table[27 ]                 =   SIGMA == 16 ? 10'd246 :
                                                     SIGMA == 48 ? 10'd873 : 10'd716;
            range_table[28 ]                 =   SIGMA == 16 ? 10'd221 :
                                                     SIGMA == 48 ? 10'd862 : 10'd697;
            range_table[29 ]                 =   SIGMA == 16 ? 10'd197 :
                                                     SIGMA == 48 ? 10'd852 : 10'd678;
            range_table[30 ]                 =   SIGMA == 16 ? 10'd176 :
                                                     SIGMA == 48 ? 10'd841 : 10'd659;
            range_table[31 ]                 =   SIGMA == 16 ? 10'd156 :
                                                     SIGMA == 48 ? 10'd830 : 10'd639;
            range_table[32 ]                 =   SIGMA == 16 ? 10'd138 :
                                                     SIGMA == 48 ? 10'd819 : 10'd620;
            range_table[33 ]                 =   SIGMA == 16 ? 10'd121 :
                                                     SIGMA == 48 ? 10'd807 : 10'd601;
            range_table[34 ]                 =   SIGMA == 16 ? 10'd106 :
                                                     SIGMA == 48 ? 10'd796 : 10'd581;
            range_table[35 ]                 =   SIGMA == 16 ? 10'd93 :
                                                     SIGMA == 48 ? 10'd784 : 10'd562;
            range_table[36 ]                 =   SIGMA == 16 ? 10'd81 :
                                                     SIGMA == 48 ? 10'd772 : 10'd543;
            range_table[37 ]                 =   SIGMA == 16 ? 10'd70 :
                                                     SIGMA == 48 ? 10'd760 : 10'd524;
            range_table[38 ]                 =   SIGMA == 16 ? 10'd60 :
                                                     SIGMA == 48 ? 10'd747 : 10'd505;
            range_table[39 ]                 =   SIGMA == 16 ? 10'd52 :
                                                     SIGMA == 48 ? 10'd735 : 10'd486;
            range_table[40 ]                 =   SIGMA == 16 ? 10'd44 :
                                                     SIGMA == 48 ? 10'd722 : 10'd468;
            range_table[41 ]                 =   SIGMA == 16 ? 10'd38 :
                                                     SIGMA == 48 ? 10'd710 : 10'd450;
            range_table[42 ]                 =   SIGMA == 16 ? 10'd32 :
                                                     SIGMA == 48 ? 10'd697 : 10'd432;
            range_table[43 ]                 =   SIGMA == 16 ? 10'd27 :
                                                     SIGMA == 48 ? 10'd684 : 10'd414;
            range_table[44 ]                 =   SIGMA == 16 ? 10'd23 :
                                                     SIGMA == 48 ? 10'd672 : 10'd397;
            range_table[45 ]                 =   SIGMA == 16 ? 10'd19 :
                                                     SIGMA == 48 ? 10'd659 : 10'd380;
            range_table[46 ]                 =   SIGMA == 16 ? 10'd16 :
                                                     SIGMA == 48 ? 10'd646 : 10'd364;
            range_table[47 ]                 =   SIGMA == 16 ? 10'd13 :
                                                     SIGMA == 48 ? 10'd633 : 10'd347;
            range_table[48 ]                 =   SIGMA == 16 ? 10'd11 :
                                                     SIGMA == 48 ? 10'd620 : 10'd332;
            range_table[49 ]                 =   SIGMA == 16 ? 10'd9 :
                                                     SIGMA == 48 ? 10'd607 : 10'd316;
            range_table[50 ]                 =   SIGMA == 16 ? 10'd7 :
                                                     SIGMA == 48 ? 10'd594 : 10'd301;
            range_table[51 ]                 =   SIGMA == 16 ? 10'd6 :
                                                     SIGMA == 48 ? 10'd581 : 10'd287;
            range_table[52 ]                 =   SIGMA == 16 ? 10'd5 :
                                                     SIGMA == 48 ? 10'd568 : 10'd273;
            range_table[53 ]                 =   SIGMA == 16 ? 10'd4 :
                                                     SIGMA == 48 ? 10'd556 : 10'd259;
            range_table[54 ]                 =   SIGMA == 16 ? 10'd3 :
                                                     SIGMA == 48 ? 10'd543 : 10'd246;
            range_table[55 ]                 =   SIGMA == 16 ? 10'd2 :
                                                     SIGMA == 48 ? 10'd530 : 10'd233;
            range_table[56 ]                 =   SIGMA == 16 ? 10'd2 :
                                                     SIGMA == 48 ? 10'd517 : 10'd221;
            range_table[57 ]                 =   SIGMA == 16 ? 10'd1 :
                                                     SIGMA == 48 ? 10'd505 : 10'd209;
            range_table[58 ]                 =   SIGMA == 16 ? 10'd1 :
                                                     SIGMA == 48 ? 10'd492 : 10'd197;
            range_table[59 ]                 =   SIGMA == 16 ? 10'd1 :
                                                     SIGMA == 48 ? 10'd480 : 10'd186;
            range_table[60 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd468 : 10'd176;
            range_table[61 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd456 : 10'd166;
            range_table[62 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd444 : 10'd156;
            range_table[63 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd432 : 10'd147;
            range_table[64 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd420 : 10'd138;
            range_table[65 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd408 : 10'd129;
            range_table[66 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd397 : 10'd121;
            range_table[67 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd386 : 10'd114;
            range_table[68 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd375 : 10'd106;
            range_table[69 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd364 : 10'd100;
            range_table[70 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd353 : 10'd93;
            range_table[71 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd342 : 10'd87;
            range_table[72 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd332 : 10'd81;
            range_table[73 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd321 : 10'd75;
            range_table[74 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd311 : 10'd70;
            range_table[75 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd301 : 10'd65;
            range_table[76 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd292 : 10'd60;
            range_table[77 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd282 : 10'd56;
            range_table[78 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd273 : 10'd52;
            range_table[79 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd264 : 10'd48;
            range_table[80 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd255 : 10'd44;
            range_table[81 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd246 : 10'd41;
            range_table[82 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd237 : 10'd38;
            range_table[83 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd229 : 10'd35;
            range_table[84 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd221 : 10'd32;
            range_table[85 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd213 : 10'd30;
            range_table[86 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd205 : 10'd27;
            range_table[87 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd197 : 10'd25;
            range_table[88 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd190 : 10'd23;
            range_table[89 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd183 : 10'd21;
            range_table[90 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd176 : 10'd19;
            range_table[91 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd169 : 10'd17;
            range_table[92 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd162 : 10'd16;
            range_table[93 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd156 : 10'd14;
            range_table[94 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd150 : 10'd13;
            range_table[95 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd144 : 10'd12;
            range_table[96 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd138 : 10'd11;
            range_table[97 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd132 : 10'd10;
            range_table[98 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd127 : 10'd9;
            range_table[99 ]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd121 : 10'd8;
            range_table[100]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd116 : 10'd7;
            range_table[101]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd111 : 10'd7;
            range_table[102]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd106 : 10'd6;
            range_table[103]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd102 : 10'd5;
            range_table[104]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd97 : 10'd5;
            range_table[105]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd93 : 10'd4;
            range_table[106]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd89 : 10'd4;
            range_table[107]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd85 : 10'd3;
            range_table[108]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd81 : 10'd3;
            range_table[109]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd77 : 10'd3;
            range_table[110]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd74 : 10'd2;
            range_table[111]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd70 : 10'd2;
            range_table[112]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd67 : 10'd2;
            range_table[113]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd64 : 10'd2;
            range_table[114]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd60 : 10'd1;
            range_table[115]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd58 : 10'd1;
            range_table[116]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd55 : 10'd1;
            range_table[117]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd52 : 10'd1;
            range_table[118]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd49 : 10'd1;
            range_table[119]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd47 : 10'd1;
            range_table[120]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd44 : 10'd0;
            range_table[121]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd42 : 10'd0;
            range_table[122]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd40 : 10'd0;
            range_table[123]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd38 : 10'd0;
            range_table[124]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd36 : 10'd0;
            range_table[125]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd34 : 10'd0;
            range_table[126]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd32 : 10'd0;
            range_table[127]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd30 : 10'd0;
            range_table[128]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd29 : 10'd0;
            range_table[129]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd27 : 10'd0;
            range_table[130]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd26 : 10'd0;
            range_table[131]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd24 : 10'd0;
            range_table[132]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd23 : 10'd0;
            range_table[133]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd22 : 10'd0;
            range_table[134]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd20 : 10'd0;
            range_table[135]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd19 : 10'd0;
            range_table[136]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd18 : 10'd0;
            range_table[137]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd17 : 10'd0;
            range_table[138]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd16 : 10'd0;
            range_table[139]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd15 : 10'd0;
            range_table[140]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd14 : 10'd0;
            range_table[141]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd13 : 10'd0;
            range_table[142]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd12 : 10'd0;
            range_table[143]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd12 : 10'd0;
            range_table[144]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd11 : 10'd0;
            range_table[145]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd10 : 10'd0;
            range_table[146]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd10 : 10'd0;
            range_table[147]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd9 : 10'd0;
            range_table[148]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd8 : 10'd0;
            range_table[149]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd8 : 10'd0;
            range_table[150]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd7 : 10'd0;
            range_table[151]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd7 : 10'd0;
            range_table[152]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd6 : 10'd0;
            range_table[153]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd6 : 10'd0;
            range_table[154]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd5 : 10'd0;
            range_table[155]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd5 : 10'd0;
            range_table[156]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd5 : 10'd0;
            range_table[157]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd4 : 10'd0;
            range_table[158]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd4 : 10'd0;
            range_table[159]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd4 : 10'd0;
            range_table[160]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd3 : 10'd0;
            range_table[161]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd3 : 10'd0;
            range_table[162]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd3 : 10'd0;
            range_table[163]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd3 : 10'd0;
            range_table[164]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd2 : 10'd0;
            range_table[165]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd2 : 10'd0;
            range_table[166]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd2 : 10'd0;
            range_table[167]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd2 : 10'd0;
            range_table[168]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd2 : 10'd0;
            range_table[169]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd2 : 10'd0;
            range_table[170]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd1 : 10'd0;
            range_table[171]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd1 : 10'd0;
            range_table[172]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd1 : 10'd0;
            range_table[173]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd1 : 10'd0;
            range_table[174]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd1 : 10'd0;
            range_table[175]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd1 : 10'd0;
            range_table[176]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd1 : 10'd0;
            range_table[177]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd1 : 10'd0;
            range_table[178]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd1 : 10'd0;
            range_table[179]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[180]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[181]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[182]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[183]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[184]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[185]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[186]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[187]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[188]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[189]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[190]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[191]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[192]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[193]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[194]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[195]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[196]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[197]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[198]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[199]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[200]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[201]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[202]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[203]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[204]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[205]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[206]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[207]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[208]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[209]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[210]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[211]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[212]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[213]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[214]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[215]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[216]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[217]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[218]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[219]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[220]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[221]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[222]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[223]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[224]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[225]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[226]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[227]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[228]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[229]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[230]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[231]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[232]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[233]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[234]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[235]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[236]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[237]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[238]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[239]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[240]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[241]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[242]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[243]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[244]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[245]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[246]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[247]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[248]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[249]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[250]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[251]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[252]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[253]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[254]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
            range_table[255]                 =   SIGMA == 16 ? 10'd0 :
                                                     SIGMA == 48 ? 10'd0 : 10'd0;
        end

        always@(posedge clk)begin
            Post_Data                        <=  range_table[Pre_Data];
        end
    end
endgenerate

`ifndef SYNTHESIS

initial
    if(SIGMA != 16 && SIGMA != 32 && SIGMA != 48)
        $error("Denoise sigma must be 16, 32 or 48");
`endif

endmodule
