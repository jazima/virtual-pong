`timescale 1ns/1ps

module physics_core_top(
    input CLK100MHZ,
    input CPU_RESETN,
    input [15:0] SW,
    output [15:0] LED,
    output CA, CB, CC, CD, CE, CF, CG,
    output [7:0]AN,
    input BTNC
);

wire [23:0] pc_gr_ball_X_p1;
wire [23:0] pc_gr_ball_Y_p1;
wire [23:0] pc_gr_ball_Z_p1;

wire [15:0] BALL_X;
wire [15:0] BALL_Y;
wire [15:0] BALL_Z;
assign BALL_X = pc_gr_ball_X_p1[23:8];
assign BALL_Y = pc_gr_ball_Y_p1[23:8];
assign BALL_Z = pc_gr_ball_Z_p1[23:8];

seven_seg_decoder decoder({BALL_Y, BALL_Z}, CLK100MHZ, AN, {CA, CB, CC, CD, CE, CF, CG});

assign LED[7:0] = BALL_Z[7:0];
assign LED[15:8] = BALL_Y[11:4];

wire [23:0] pc_gr_paddle_1_X_p1;
wire [23:0] pc_gr_paddle_1_Y_p1;
wire [23:0] pc_gr_paddle_1_Z_p1;

wire [23:0] pc_gr_paddle_2_X_p1;
wire [23:0] pc_gr_paddle_2_Y_p1;
wire [23:0] pc_gr_paddle_2_Z_p1;

physics_core core(
    .aclk(CLK100MHZ),
    .aresetn(CPU_RESETN),
    .start(BTNC),
    
    .ip_pc_paddle_1_X(SW[7:0]),
    .ip_pc_paddle_1_Y({24'b0}),
    .ip_pc_paddle_1_Z({24'b0}),
    .ip_pc_paddle_1_valid(1'b0),
    
    .ip_pc_paddle_2_X(SW[15:8]),
    .ip_pc_paddle_2_Y({24'b0}),
    .ip_pc_paddle_2_Z({24'b0}),
    .ip_pc_paddle_2_valid(1'b0),
    
    .pc_gr_ball_X_p1(pc_gr_ball_X_p1),
    .pc_gr_ball_Y_p1(pc_gr_ball_Y_p1),
    .pc_gr_ball_Z_p1(pc_gr_ball_Z_p1),
    
    .pc_gr_paddle_1_X_p1(pc_gr_paddle_1_X_p1),
    .pc_gr_paddle_1_Y_p1(pc_gr_paddle_1_Y_p1),
    .pc_gr_paddle_1_Z_p1(pc_gr_paddle_1_Z_p1),
    
    .pc_gr_paddle_2_X_p1(pc_gr_paddle_2_X_p1),
    .pc_gr_paddle_2_Y_p1(pc_gr_paddle_2_Y_p1),
    .pc_gr_paddle_2_Z_p1(pc_gr_paddle_2_Z_p1)
);
endmodule


module seven_seg_decoder (
    input [31:0] bin1, // 16-bit binary input
    input clk, // Clock input
    output reg [7:0] an, // Anode selection
    output reg [6:0] seg // 7 segment output
);

reg [2:0] digit;
reg [31:0] value;
reg [26:0] clk_divider; // Clock divider for 100MHz to 1Hz

always @(posedge clk)
begin
    clk_divider <= clk_divider + 1;
    if(clk_divider == 200000) // Half period of 1Hz at 100MHz
    begin
        clk_divider <= 0;
        digit <= digit + 1;
        value <= bin1;
    end
end

always @(digit)
begin
    case(digit)
        4'h0: an = 8'b11111110; // Selecting first digit
        4'h1: an = 8'b11111101; // Selecting second digit
        4'h2: an = 8'b11111011; // Selecting third digit
        4'h3: an = 8'b11110111; // Selecting fourth digit
        4'h4: an = 8'b11101111; // Selecting first digit
        4'h5: an = 8'b11011111; // Selecting second digit
        4'h6: an = 8'b10111111; // Selecting third digit
        4'h7: an = 8'b01111111; // Selecting fourth digit
        default: an = 8'b11111111; // Default case
    endcase
end

always @(value[digit*4 +: 4])
begin
    case(value[digit*4 +: 4])
        4'h0: seg = 7'b0000001; // For displaying 0
        4'h1: seg = 7'b1001111; // For displaying 1
        4'h2: seg = 7'b0010010; // For displaying 2
        4'h3: seg = 7'b0000110; // For displaying 3
        4'h4: seg = 7'b1001100; // For displaying 4
        4'h5: seg = 7'b0100100; // For displaying 5
        4'h6: seg = 7'b0100000; // For displaying 6
        4'h7: seg = 7'b0001111; // For displaying 7
        4'h8: seg = 7'b0000000; // For displaying 8
        4'h9: seg = 7'b0000100; // For displaying 9
        4'hA: seg = 7'b0001000; // For displaying A
        4'hB: seg = 7'b1100000; // For displaying B
        4'hC: seg = 7'b0110001; // For displaying C
        4'hD: seg = 7'b1000010; // For displaying D
        4'hE: seg = 7'b0110000; // For displaying E
        4'hF: seg = 7'b0111000; // For displaying F
        default: seg = 7'b1111111; // Default case
    endcase
end

endmodule
