`timescale 1ns / 1ps

module physics_core_tb (

);
    reg aclk;
    reg aresetn;

    reg [23:0] ip_pc_paddle_1_X;
    reg [23:0] ip_pc_paddle_1_Y;
    reg [23:0] ip_pc_paddle_1_Z;
    reg ip_pc_paddle_1_valid;

    reg [23:0] ip_pc_paddle_2_X;
    reg [23:0] ip_pc_paddle_2_Y;
    reg [23:0] ip_pc_paddle_2_Z;
    reg ip_pc_paddle_2_valid;

    wire [23:0] pc_gr_ball_X_p1;
    wire [23:0] pc_gr_ball_Y_p1;
    wire [23:0] pc_gr_ball_Z_p1;

    wire [15:0] BALL_X;
    wire [15:0] BALL_Y;
    wire [15:0] BALL_Z;
    assign BALL_X = pc_gr_ball_X_p1[23:8];
    assign BALL_Y = pc_gr_ball_Y_p1[23:8];
    assign BALL_Z = pc_gr_ball_Z_p1[23:8];
    
    wire [23:0] pc_gr_paddle_1_X_p1;
    wire [23:0] pc_gr_paddle_1_Y_p1;
    wire [23:0] pc_gr_paddle_1_Z_p1;

    wire [23:0] pc_gr_paddle_2_X_p1;
    wire [23:0] pc_gr_paddle_2_Y_p1;
    wire [23:0] pc_gr_paddle_2_Z_p1;
    
    physics_core core(
        .aclk(aclk),
        .aresetn(aresetn),
        .ip_pc_paddle_1_X(ip_pc_paddle_1_X),
        .ip_pc_paddle_1_Y(ip_pc_paddle_1_Y),
        .ip_pc_paddle_1_Z(ip_pc_paddle_1_Z),
        .ip_pc_paddle_1_valid(ip_pc_paddle_1_valid),
        .ip_pc_paddle_2_X(ip_pc_paddle_2_X),
        .ip_pc_paddle_2_Y(ip_pc_paddle_2_Y),
        .ip_pc_paddle_2_Z(ip_pc_paddle_2_Z),
        .ip_pc_paddle_2_valid(ip_pc_paddle_2_valid),
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

    initial begin
        aclk <= 0;
        ip_pc_paddle_1_X <= 0;
        ip_pc_paddle_1_Y <= 0;
        ip_pc_paddle_1_Z <= 0;
        ip_pc_paddle_1_valid <= 0;
        ip_pc_paddle_2_X <= 0;
        ip_pc_paddle_2_Y <= 0;
        ip_pc_paddle_2_Z <= 0;
        ip_pc_paddle_2_valid <= 0;

        forever begin
            #20 aclk <= ~aclk;
        end
    end
    
    initial begin
        aresetn <= 0;
        #80 aresetn <= 1;
    end
endmodule