// Exact Gaussian range weights, with one registered ROM read per pixel.
// Runtime sigma selects a ROM address bank before the registered lookup.
module Bilateral_Range_LUT#(
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
        wire    [1 : 0]         sigma_bank                       ;
        wire    [9 : 0]         lookup_address                   ;
        reg     [9 : 0]         lookup_weight /* synthesis syn_romstyle = "lut_rom" */;

        assign  sigma_bank                       =   cfg_data[78 : 73] == 6'd16 ? 2'd0 :
                                                     cfg_data[78 : 73] == 6'd48 ? 2'd2 : 2'd1;
        assign  lookup_address                   =   {sigma_bank, Pre_Data};

        // Same exact weights and one-clock lookup; logic ROM isolates the
        // next multiplier from the embedded memory's long clock-to-output.
        always@(*)begin
            case(lookup_address)
                10'd0   : lookup_weight          =   10'd1023;
                10'd1   : lookup_weight          =   10'd1021;
                10'd2   : lookup_weight          =   10'd1015;
                10'd3   : lookup_weight          =   10'd1005;
                10'd4   : lookup_weight          =   10'd991;
                10'd5   : lookup_weight          =   10'd974;
                10'd6   : lookup_weight          =   10'd953;
                10'd7   : lookup_weight          =   10'd929;
                10'd8   : lookup_weight          =   10'd902;
                10'd9   : lookup_weight          =   10'd873;
                10'd10  : lookup_weight          =   10'd841;
                10'd11  : lookup_weight          =   10'd807;
                10'd12  : lookup_weight          =   10'd772;
                10'd13  : lookup_weight          =   10'd735;
                10'd14  : lookup_weight          =   10'd697;
                10'd15  : lookup_weight          =   10'd659;
                10'd16  : lookup_weight          =   10'd620;
                10'd17  : lookup_weight          =   10'd581;
                10'd18  : lookup_weight          =   10'd543;
                10'd19  : lookup_weight          =   10'd505;
                10'd20  : lookup_weight          =   10'd468;
                10'd21  : lookup_weight          =   10'd432;
                10'd22  : lookup_weight          =   10'd397;
                10'd23  : lookup_weight          =   10'd364;
                10'd24  : lookup_weight          =   10'd332;
                10'd25  : lookup_weight          =   10'd301;
                10'd26  : lookup_weight          =   10'd273;
                10'd27  : lookup_weight          =   10'd246;
                10'd28  : lookup_weight          =   10'd221;
                10'd29  : lookup_weight          =   10'd197;
                10'd30  : lookup_weight          =   10'd176;
                10'd31  : lookup_weight          =   10'd156;
                10'd32  : lookup_weight          =   10'd138;
                10'd33  : lookup_weight          =   10'd121;
                10'd34  : lookup_weight          =   10'd106;
                10'd35  : lookup_weight          =   10'd93;
                10'd36  : lookup_weight          =   10'd81;
                10'd37  : lookup_weight          =   10'd70;
                10'd38  : lookup_weight          =   10'd60;
                10'd39  : lookup_weight          =   10'd52;
                10'd40  : lookup_weight          =   10'd44;
                10'd41  : lookup_weight          =   10'd38;
                10'd42  : lookup_weight          =   10'd32;
                10'd43  : lookup_weight          =   10'd27;
                10'd44  : lookup_weight          =   10'd23;
                10'd45  : lookup_weight          =   10'd19;
                10'd46  : lookup_weight          =   10'd16;
                10'd47  : lookup_weight          =   10'd13;
                10'd48  : lookup_weight          =   10'd11;
                10'd49  : lookup_weight          =   10'd9;
                10'd50  : lookup_weight          =   10'd7;
                10'd51  : lookup_weight          =   10'd6;
                10'd52  : lookup_weight          =   10'd5;
                10'd53  : lookup_weight          =   10'd4;
                10'd54  : lookup_weight          =   10'd3;
                10'd55  : lookup_weight          =   10'd2;
                10'd56  : lookup_weight          =   10'd2;
                10'd57  : lookup_weight          =   10'd1;
                10'd58  : lookup_weight          =   10'd1;
                10'd59  : lookup_weight          =   10'd1;
                10'd256 : lookup_weight          =   10'd1023;
                10'd257 : lookup_weight          =   10'd1022;
                10'd258 : lookup_weight          =   10'd1021;
                10'd259 : lookup_weight          =   10'd1018;
                10'd260 : lookup_weight          =   10'd1015;
                10'd261 : lookup_weight          =   10'd1010;
                10'd262 : lookup_weight          =   10'd1005;
                10'd263 : lookup_weight          =   10'd998;
                10'd264 : lookup_weight          =   10'd991;
                10'd265 : lookup_weight          =   10'd983;
                10'd266 : lookup_weight          =   10'd974;
                10'd267 : lookup_weight          =   10'd964;
                10'd268 : lookup_weight          =   10'd953;
                10'd269 : lookup_weight          =   10'd941;
                10'd270 : lookup_weight          =   10'd929;
                10'd271 : lookup_weight          =   10'd916;
                10'd272 : lookup_weight          =   10'd902;
                10'd273 : lookup_weight          =   10'd888;
                10'd274 : lookup_weight          =   10'd873;
                10'd275 : lookup_weight          =   10'd857;
                10'd276 : lookup_weight          =   10'd841;
                10'd277 : lookup_weight          =   10'd824;
                10'd278 : lookup_weight          =   10'd807;
                10'd279 : lookup_weight          =   10'd790;
                10'd280 : lookup_weight          =   10'd772;
                10'd281 : lookup_weight          =   10'd753;
                10'd282 : lookup_weight          =   10'd735;
                10'd283 : lookup_weight          =   10'd716;
                10'd284 : lookup_weight          =   10'd697;
                10'd285 : lookup_weight          =   10'd678;
                10'd286 : lookup_weight          =   10'd659;
                10'd287 : lookup_weight          =   10'd639;
                10'd288 : lookup_weight          =   10'd620;
                10'd289 : lookup_weight          =   10'd601;
                10'd290 : lookup_weight          =   10'd581;
                10'd291 : lookup_weight          =   10'd562;
                10'd292 : lookup_weight          =   10'd543;
                10'd293 : lookup_weight          =   10'd524;
                10'd294 : lookup_weight          =   10'd505;
                10'd295 : lookup_weight          =   10'd486;
                10'd296 : lookup_weight          =   10'd468;
                10'd297 : lookup_weight          =   10'd450;
                10'd298 : lookup_weight          =   10'd432;
                10'd299 : lookup_weight          =   10'd414;
                10'd300 : lookup_weight          =   10'd397;
                10'd301 : lookup_weight          =   10'd380;
                10'd302 : lookup_weight          =   10'd364;
                10'd303 : lookup_weight          =   10'd347;
                10'd304 : lookup_weight          =   10'd332;
                10'd305 : lookup_weight          =   10'd316;
                10'd306 : lookup_weight          =   10'd301;
                10'd307 : lookup_weight          =   10'd287;
                10'd308 : lookup_weight          =   10'd273;
                10'd309 : lookup_weight          =   10'd259;
                10'd310 : lookup_weight          =   10'd246;
                10'd311 : lookup_weight          =   10'd233;
                10'd312 : lookup_weight          =   10'd221;
                10'd313 : lookup_weight          =   10'd209;
                10'd314 : lookup_weight          =   10'd197;
                10'd315 : lookup_weight          =   10'd186;
                10'd316 : lookup_weight          =   10'd176;
                10'd317 : lookup_weight          =   10'd166;
                10'd318 : lookup_weight          =   10'd156;
                10'd319 : lookup_weight          =   10'd147;
                10'd320 : lookup_weight          =   10'd138;
                10'd321 : lookup_weight          =   10'd129;
                10'd322 : lookup_weight          =   10'd121;
                10'd323 : lookup_weight          =   10'd114;
                10'd324 : lookup_weight          =   10'd106;
                10'd325 : lookup_weight          =   10'd100;
                10'd326 : lookup_weight          =   10'd93;
                10'd327 : lookup_weight          =   10'd87;
                10'd328 : lookup_weight          =   10'd81;
                10'd329 : lookup_weight          =   10'd75;
                10'd330 : lookup_weight          =   10'd70;
                10'd331 : lookup_weight          =   10'd65;
                10'd332 : lookup_weight          =   10'd60;
                10'd333 : lookup_weight          =   10'd56;
                10'd334 : lookup_weight          =   10'd52;
                10'd335 : lookup_weight          =   10'd48;
                10'd336 : lookup_weight          =   10'd44;
                10'd337 : lookup_weight          =   10'd41;
                10'd338 : lookup_weight          =   10'd38;
                10'd339 : lookup_weight          =   10'd35;
                10'd340 : lookup_weight          =   10'd32;
                10'd341 : lookup_weight          =   10'd30;
                10'd342 : lookup_weight          =   10'd27;
                10'd343 : lookup_weight          =   10'd25;
                10'd344 : lookup_weight          =   10'd23;
                10'd345 : lookup_weight          =   10'd21;
                10'd346 : lookup_weight          =   10'd19;
                10'd347 : lookup_weight          =   10'd17;
                10'd348 : lookup_weight          =   10'd16;
                10'd349 : lookup_weight          =   10'd14;
                10'd350 : lookup_weight          =   10'd13;
                10'd351 : lookup_weight          =   10'd12;
                10'd352 : lookup_weight          =   10'd11;
                10'd353 : lookup_weight          =   10'd10;
                10'd354 : lookup_weight          =   10'd9;
                10'd355 : lookup_weight          =   10'd8;
                10'd356 : lookup_weight          =   10'd7;
                10'd357 : lookup_weight          =   10'd7;
                10'd358 : lookup_weight          =   10'd6;
                10'd359 : lookup_weight          =   10'd5;
                10'd360 : lookup_weight          =   10'd5;
                10'd361 : lookup_weight          =   10'd4;
                10'd362 : lookup_weight          =   10'd4;
                10'd363 : lookup_weight          =   10'd3;
                10'd364 : lookup_weight          =   10'd3;
                10'd365 : lookup_weight          =   10'd3;
                10'd366 : lookup_weight          =   10'd2;
                10'd367 : lookup_weight          =   10'd2;
                10'd368 : lookup_weight          =   10'd2;
                10'd369 : lookup_weight          =   10'd2;
                10'd370 : lookup_weight          =   10'd1;
                10'd371 : lookup_weight          =   10'd1;
                10'd372 : lookup_weight          =   10'd1;
                10'd373 : lookup_weight          =   10'd1;
                10'd374 : lookup_weight          =   10'd1;
                10'd375 : lookup_weight          =   10'd1;
                10'd512 : lookup_weight          =   10'd1023;
                10'd513 : lookup_weight          =   10'd1022;
                10'd514 : lookup_weight          =   10'd1022;
                10'd515 : lookup_weight          =   10'd1021;
                10'd516 : lookup_weight          =   10'd1019;
                10'd517 : lookup_weight          =   10'd1017;
                10'd518 : lookup_weight          =   10'd1015;
                10'd519 : lookup_weight          =   10'd1012;
                10'd520 : lookup_weight          =   10'd1008;
                10'd521 : lookup_weight          =   10'd1005;
                10'd522 : lookup_weight          =   10'd1001;
                10'd523 : lookup_weight          =   10'd996;
                10'd524 : lookup_weight          =   10'd991;
                10'd525 : lookup_weight          =   10'd986;
                10'd526 : lookup_weight          =   10'd980;
                10'd527 : lookup_weight          =   10'd974;
                10'd528 : lookup_weight          =   10'd967;
                10'd529 : lookup_weight          =   10'd960;
                10'd530 : lookup_weight          =   10'd953;
                10'd531 : lookup_weight          =   10'd945;
                10'd532 : lookup_weight          =   10'd937;
                10'd533 : lookup_weight          =   10'd929;
                10'd534 : lookup_weight          =   10'd921;
                10'd535 : lookup_weight          =   10'd912;
                10'd536 : lookup_weight          =   10'd902;
                10'd537 : lookup_weight          =   10'd893;
                10'd538 : lookup_weight          =   10'd883;
                10'd539 : lookup_weight          =   10'd873;
                10'd540 : lookup_weight          =   10'd862;
                10'd541 : lookup_weight          =   10'd852;
                10'd542 : lookup_weight          =   10'd841;
                10'd543 : lookup_weight          =   10'd830;
                10'd544 : lookup_weight          =   10'd819;
                10'd545 : lookup_weight          =   10'd807;
                10'd546 : lookup_weight          =   10'd796;
                10'd547 : lookup_weight          =   10'd784;
                10'd548 : lookup_weight          =   10'd772;
                10'd549 : lookup_weight          =   10'd760;
                10'd550 : lookup_weight          =   10'd747;
                10'd551 : lookup_weight          =   10'd735;
                10'd552 : lookup_weight          =   10'd722;
                10'd553 : lookup_weight          =   10'd710;
                10'd554 : lookup_weight          =   10'd697;
                10'd555 : lookup_weight          =   10'd684;
                10'd556 : lookup_weight          =   10'd672;
                10'd557 : lookup_weight          =   10'd659;
                10'd558 : lookup_weight          =   10'd646;
                10'd559 : lookup_weight          =   10'd633;
                10'd560 : lookup_weight          =   10'd620;
                10'd561 : lookup_weight          =   10'd607;
                10'd562 : lookup_weight          =   10'd594;
                10'd563 : lookup_weight          =   10'd581;
                10'd564 : lookup_weight          =   10'd568;
                10'd565 : lookup_weight          =   10'd556;
                10'd566 : lookup_weight          =   10'd543;
                10'd567 : lookup_weight          =   10'd530;
                10'd568 : lookup_weight          =   10'd517;
                10'd569 : lookup_weight          =   10'd505;
                10'd570 : lookup_weight          =   10'd492;
                10'd571 : lookup_weight          =   10'd480;
                10'd572 : lookup_weight          =   10'd468;
                10'd573 : lookup_weight          =   10'd456;
                10'd574 : lookup_weight          =   10'd444;
                10'd575 : lookup_weight          =   10'd432;
                10'd576 : lookup_weight          =   10'd420;
                10'd577 : lookup_weight          =   10'd408;
                10'd578 : lookup_weight          =   10'd397;
                10'd579 : lookup_weight          =   10'd386;
                10'd580 : lookup_weight          =   10'd375;
                10'd581 : lookup_weight          =   10'd364;
                10'd582 : lookup_weight          =   10'd353;
                10'd583 : lookup_weight          =   10'd342;
                10'd584 : lookup_weight          =   10'd332;
                10'd585 : lookup_weight          =   10'd321;
                10'd586 : lookup_weight          =   10'd311;
                10'd587 : lookup_weight          =   10'd301;
                10'd588 : lookup_weight          =   10'd292;
                10'd589 : lookup_weight          =   10'd282;
                10'd590 : lookup_weight          =   10'd273;
                10'd591 : lookup_weight          =   10'd264;
                10'd592 : lookup_weight          =   10'd255;
                10'd593 : lookup_weight          =   10'd246;
                10'd594 : lookup_weight          =   10'd237;
                10'd595 : lookup_weight          =   10'd229;
                10'd596 : lookup_weight          =   10'd221;
                10'd597 : lookup_weight          =   10'd213;
                10'd598 : lookup_weight          =   10'd205;
                10'd599 : lookup_weight          =   10'd197;
                10'd600 : lookup_weight          =   10'd190;
                10'd601 : lookup_weight          =   10'd183;
                10'd602 : lookup_weight          =   10'd176;
                10'd603 : lookup_weight          =   10'd169;
                10'd604 : lookup_weight          =   10'd162;
                10'd605 : lookup_weight          =   10'd156;
                10'd606 : lookup_weight          =   10'd150;
                10'd607 : lookup_weight          =   10'd144;
                10'd608 : lookup_weight          =   10'd138;
                10'd609 : lookup_weight          =   10'd132;
                10'd610 : lookup_weight          =   10'd127;
                10'd611 : lookup_weight          =   10'd121;
                10'd612 : lookup_weight          =   10'd116;
                10'd613 : lookup_weight          =   10'd111;
                10'd614 : lookup_weight          =   10'd106;
                10'd615 : lookup_weight          =   10'd102;
                10'd616 : lookup_weight          =   10'd97;
                10'd617 : lookup_weight          =   10'd93;
                10'd618 : lookup_weight          =   10'd89;
                10'd619 : lookup_weight          =   10'd85;
                10'd620 : lookup_weight          =   10'd81;
                10'd621 : lookup_weight          =   10'd77;
                10'd622 : lookup_weight          =   10'd74;
                10'd623 : lookup_weight          =   10'd70;
                10'd624 : lookup_weight          =   10'd67;
                10'd625 : lookup_weight          =   10'd64;
                10'd626 : lookup_weight          =   10'd60;
                10'd627 : lookup_weight          =   10'd58;
                10'd628 : lookup_weight          =   10'd55;
                10'd629 : lookup_weight          =   10'd52;
                10'd630 : lookup_weight          =   10'd49;
                10'd631 : lookup_weight          =   10'd47;
                10'd632 : lookup_weight          =   10'd44;
                10'd633 : lookup_weight          =   10'd42;
                10'd634 : lookup_weight          =   10'd40;
                10'd635 : lookup_weight          =   10'd38;
                10'd636 : lookup_weight          =   10'd36;
                10'd637 : lookup_weight          =   10'd34;
                10'd638 : lookup_weight          =   10'd32;
                10'd639 : lookup_weight          =   10'd30;
                10'd640 : lookup_weight          =   10'd29;
                10'd641 : lookup_weight          =   10'd27;
                10'd642 : lookup_weight          =   10'd26;
                10'd643 : lookup_weight          =   10'd24;
                10'd644 : lookup_weight          =   10'd23;
                10'd645 : lookup_weight          =   10'd22;
                10'd646 : lookup_weight          =   10'd20;
                10'd647 : lookup_weight          =   10'd19;
                10'd648 : lookup_weight          =   10'd18;
                10'd649 : lookup_weight          =   10'd17;
                10'd650 : lookup_weight          =   10'd16;
                10'd651 : lookup_weight          =   10'd15;
                10'd652 : lookup_weight          =   10'd14;
                10'd653 : lookup_weight          =   10'd13;
                10'd654 : lookup_weight          =   10'd12;
                10'd655 : lookup_weight          =   10'd12;
                10'd656 : lookup_weight          =   10'd11;
                10'd657 : lookup_weight          =   10'd10;
                10'd658 : lookup_weight          =   10'd10;
                10'd659 : lookup_weight          =   10'd9;
                10'd660 : lookup_weight          =   10'd8;
                10'd661 : lookup_weight          =   10'd8;
                10'd662 : lookup_weight          =   10'd7;
                10'd663 : lookup_weight          =   10'd7;
                10'd664 : lookup_weight          =   10'd6;
                10'd665 : lookup_weight          =   10'd6;
                10'd666 : lookup_weight          =   10'd5;
                10'd667 : lookup_weight          =   10'd5;
                10'd668 : lookup_weight          =   10'd5;
                10'd669 : lookup_weight          =   10'd4;
                10'd670 : lookup_weight          =   10'd4;
                10'd671 : lookup_weight          =   10'd4;
                10'd672 : lookup_weight          =   10'd3;
                10'd673 : lookup_weight          =   10'd3;
                10'd674 : lookup_weight          =   10'd3;
                10'd675 : lookup_weight          =   10'd3;
                10'd676 : lookup_weight          =   10'd2;
                10'd677 : lookup_weight          =   10'd2;
                10'd678 : lookup_weight          =   10'd2;
                10'd679 : lookup_weight          =   10'd2;
                10'd680 : lookup_weight          =   10'd2;
                10'd681 : lookup_weight          =   10'd2;
                10'd682 : lookup_weight          =   10'd1;
                10'd683 : lookup_weight          =   10'd1;
                10'd684 : lookup_weight          =   10'd1;
                10'd685 : lookup_weight          =   10'd1;
                10'd686 : lookup_weight          =   10'd1;
                10'd687 : lookup_weight          =   10'd1;
                10'd688 : lookup_weight          =   10'd1;
                10'd689 : lookup_weight          =   10'd1;
                10'd690 : lookup_weight          =   10'd1;
                default : lookup_weight          =   10'd0;
            endcase
        end

        always@(posedge clk)begin
            Post_Data                        <=  lookup_weight;
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
