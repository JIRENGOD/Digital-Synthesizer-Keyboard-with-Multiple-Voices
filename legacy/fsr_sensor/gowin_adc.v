//Copyright (C)2014-2026 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: IP file
//Tool Version: V1.9.12.04 (64-bit)
//IP Version: 1.0
//Part Number: GW5AST-LV138FPG676AC1/I0
//Device: GW5AST-138B
//Device Version: B
//Created Time: Thu Oct 29 23:23:28 2026

module Gowin_ADC (adcrdy, adcvalue, clk, drstn, adcreqi, fscal_value, offset_value);

output adcrdy;
output [13:0] adcvalue;
input clk;
input drstn;
input adcreqi;
input [9:0] fscal_value;
input [11:0] offset_value;

wire gw_vcc;
wire gw_gnd;

assign gw_vcc = 1'b1;
assign gw_gnd = 1'b0;

ADCLRC adclrc_inst (
    .ADCRDY(adcrdy),
    .ADCVALUE(adcvalue),
    .ADCINBK2A(gw_gnd),
    .ADCINBK2B(gw_gnd),
    .ADCINBK3A(gw_gnd),
    .ADCINBK3B(gw_gnd),
    .ADCINBK4A(gw_gnd),
    .ADCINBK4B(gw_gnd),
    .ADCINBK5A(gw_gnd),
    .ADCINBK5B(gw_gnd),
    .VSENCTL({gw_gnd,gw_gnd,gw_gnd}),
    .ADCEN(gw_vcc),
    .CLK(clk),
    .DRSTN(drstn),
    .ADCREQI(adcreqi),
    .FSCAL_VALUE(fscal_value),
    .OFFSET_VALUE(offset_value)
);

defparam adclrc_inst.CLK_SEL = 1'b1;
defparam adclrc_inst.DIV_CTL = 2'd3;
defparam adclrc_inst.DYN_BKEN = "FALSE";
defparam adclrc_inst.BUF_SERDES_Q1_EN = 3'b000;
defparam adclrc_inst.BUF_BK2_EN = 6'b000000;
defparam adclrc_inst.BUF_BK3_EN = 6'b000000;
defparam adclrc_inst.BUF_BK4_EN = 6'b000000;
defparam adclrc_inst.BUF_BK5_EN = 6'b000000;
defparam adclrc_inst.BUF_BK10_EN = 5'b00000;
defparam adclrc_inst.BUF_BK11_EN = 5'b00000;
defparam adclrc_inst.PIOCLK_SEL = 1'b0;
defparam adclrc_inst.VSEN_CTL = 3'b000;
defparam adclrc_inst.VSEN_CTL_SEL = 1'b0;
defparam adclrc_inst.ADC_MODE = 1'b1;
defparam adclrc_inst.SAMPLE_CNT_SEL = 3'd2;
defparam adclrc_inst.RATE_CHANGE_CTRL = 3'd3;

endmodule //Gowin_ADC
