//Copyright (C)2014-2025 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//Tool Version: V1.9.11.01 Education (64-bit)
//Part Number: GW2A-LV18PG256C8/I7
//Device: GW2A-18
//Device Version: C
//Created Time: Sat Apr  4 00:30:01 2026

//Change the instance name and port connections to the signal names
//--------Copy here to design--------

    Gowin_SDPB_vram_2 your_instance_name(
        .dout(dout), //output [31:0] dout
        .clka(clka), //input clka
        .cea(cea), //input cea
        .reseta(reseta), //input reseta
        .clkb(clkb), //input clkb
        .ceb(ceb), //input ceb
        .resetb(resetb), //input resetb
        .oce(oce), //input oce
        .ada(ada), //input [11:0] ada
        .din(din), //input [31:0] din
        .adb(adb) //input [11:0] adb
    );

//--------Copy end-------------------
