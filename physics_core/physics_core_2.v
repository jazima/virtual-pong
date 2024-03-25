`timescale 1ns / 1ps

// ip => Image Processing
// pc => Physics Core
// gr => Graphics Rendering

// Z is up, Y is forward(into the screen), X is right

// Table:
// Position: (x, y, z) = (0, 1370, 740)
// Dimensions: (width, depth, height) = (762, 1370, 15)

// Paddle:
// Dimensions: (width, depth, height) = (60, 10, 70)

// Ball:
// Dimensions: (radius) = (20)

// Gravity = (0, 0, -1)

module physics_core (
   input wire		    aclk,	    // 25 MHz
   input wire		    aresetn,
   input wire           start,
   
   // Incoming Raw Coordinates from Image Processing
   input wire [23:0]	ip_pc_paddle_1_X,
   input wire [23:0]	ip_pc_paddle_1_Y,
   input wire [23:0]    ip_pc_paddle_1_Z,
   input wire           ip_pc_paddle_1_valid,

   input wire [23:0]	ip_pc_paddle_2_X,
   input wire [23:0]	ip_pc_paddle_2_Y,
   input wire [23:0]    ip_pc_paddle_2_Z,
   input wire           ip_pc_paddle_2_valid,

   // To be used by Rendering Logic
   output wire [23:0]	pc_gr_ball_X_p1,
   output wire [23:0]	pc_gr_ball_Y_p1,
   output wire [23:0]	pc_gr_ball_Z_p1,
   
   output wire [23:0]  pc_gr_paddle_1_X_p1,  
   output wire [23:0]  pc_gr_paddle_1_Y_p1, 
   output wire [23:0]  pc_gr_paddle_1_Z_p1, 
    
   output wire [23:0]  pc_gr_paddle_2_X_p1, 
   output wire [23:0]  pc_gr_paddle_2_Y_p1,  
   output wire [23:0]  pc_gr_paddle_2_Z_p1
);

// Constants and Reset Values
localparam signed SCALE = 256;

localparam signed TABLE_dX = 24'd762 * SCALE;
localparam signed TABLE_dY = 24'd1370 * SCALE;
localparam signed TABLE_dZ = 24'd15 * SCALE;

localparam signed NET_dX = 24'd762 * SCALE;
localparam signed NET_dY = 24'd10 * SCALE;
localparam signed NET_dZ = 24'd30 * SCALE;

localparam signed PADDLE_dX = 24'd60 * SCALE;
localparam signed PADDLE_dY = 24'd10 * SCALE;
localparam signed PADDLE_dZ = 24'd70 * SCALE;

localparam signed [23:0] BALL_RADIUS = 24'd20 * SCALE;

localparam signed TABLE_X = 24'd0 * SCALE;
localparam signed TABLE_Y = 24'd1370 * SCALE;
localparam signed TABLE_Z = 24'd740 * SCALE;

localparam signed NET_X = 24'd762 * SCALE;
localparam signed NET_Y = 24'd1370 * SCALE;
localparam signed NET_Z = 24'd15 * SCALE;

localparam signed PADDLE1_X = 24'd0 * SCALE;
localparam signed PADDLE1_Y = 24'd0 * SCALE;
localparam signed PADDLE1_Z = 24'd790 * SCALE;

localparam signed PADDLE2_X = 24'd0 * SCALE;
localparam signed PADDLE2_Y = 24'd2740 * SCALE;
localparam signed PADDLE2_Z = 24'd790 * SCALE;

localparam signed BALL_X = 24'd0 * SCALE;
localparam signed BALL_Y = 24'd1370 * SCALE;
localparam signed BALL_Z = 24'd840 * SCALE;

localparam signed BALL_VEL_X = 24'd4;
localparam signed BALL_VEL_Y = -24'd205;
localparam signed BALL_VEL_Z = 24'd0;

localparam signed Gravity = -1;

localparam signed Loss = 1; // Loss of energy in each collision, Need to figure out how to implement using integer math

localparam signed BOUNDS_X1 = -24'd1000 * SCALE;
localparam signed BOUNDS_X2 = 24'd1000 * SCALE;
localparam signed BOUNDS_Y1 = -24'd1000 * SCALE;
localparam signed BOUNDS_Y2 = 24'd4000 * SCALE;
localparam signed BOUNDS_Z1 = 24'd0 * SCALE;
localparam signed BOUNDS_Z2 = 24'd1200 * SCALE;

localparam UPDATE_COUNT_PER_FRAME = 'd5;
localparam CYCLES_PER_FRAME = 100_000_000/60;
localparam DELAY_TIME = CYCLES_PER_FRAME / UPDATE_COUNT_PER_FRAME;

// Control Registers
// Video Processing can write to these registers to control the physics engine
reg ip_pc_paddle_1_valid_r;
reg ip_pc_paddle_2_valid_r;

// Control Registers
// Microblaze can write to these registers to control the physics engine
reg game_run; // Start/Stop the physics engine
reg game_reset; // Reset the physics engine

reg signed [23:0] ball_X_reset; // Set the balls reset position
reg signed [23:0] ball_Y_reset;
reg signed [23:0] ball_Z_reset;

reg signed [23:0] ball_velocity_X_reset; // Set the balls reset velocity
reg signed [23:0] ball_velocity_Y_reset;
reg signed [23:0] ball_velocity_Z_reset;

// Physics state registers
// Microblaze can read these registers to get the current state of the physics engine
reg signed [23:0]  paddle_1_X_p1;
reg signed [23:0]  paddle_1_Y_p1;
reg signed [23:0]  paddle_1_Z_p1;

reg signed [23:0]  paddle_2_X_p1;
reg signed [23:0]  paddle_2_Y_p1;
reg signed [23:0]  paddle_2_Z_p1;
   
reg signed [23:0] paddle_1_velocity_X;
reg signed [23:0] paddle_1_velocity_Y;
reg signed [23:0] paddle_1_velocity_Z;

reg signed [23:0] paddle_2_velocity_X;
reg signed [23:0] paddle_2_velocity_Y;
reg signed [23:0] paddle_2_velocity_Z;

reg signed [23:0] ball_X_p1;
reg signed [23:0] ball_Y_p1;
reg signed [23:0] ball_Z_p1;

reg signed [23:0] ball_velocity_X;
reg signed [23:0] ball_velocity_Y;
reg signed [23:0] ball_velocity_Z;

reg signed [23:0] ball_X_p1_new;
reg signed [23:0] ball_Y_p1_new;
reg signed [23:0] ball_Z_p1_new;

reg ball_impact_side; // Indicates which player side last impacted the ball
reg within_bounds; // Indicates if the ball is within the bounds of the table

// Collision result registers
reg collide_paddle_1; // Indicates if the ball has collided
reg collide_paddle_2;
reg collide_table;
reg collide_net;
reg collide_bounds;

// Delay Counter
reg [19:0] delay_counter;

// State machine
reg [3:0] state;

parameter [3:0] 
                IDLE                = 4'd0,
                START               = 4'd1,
                NEW_BALL_POSITION   = 4'd2,
                BALL_COLLISION      = 4'd3,

                PADDLE_1_COLLISION  = 4'd4,
                PADDLE_2_COLLISION  = 4'd5,
                TABLE_COLLISION     = 4'd6,
                NET_COLLISION       = 4'd7,

                UPDATE_BALL_POS     = 4'd8;

/////////////////////////////////////////////////////////////////////////////
///////////////////////////// Control Registers /////////////////////////////
/////////////////////////////////////////////////////////////////////////////

// TODO: How to handle set from video processing unit to only set velocity once
always @(posedge aclk) begin
    if (~aresetn) begin
        ip_pc_paddle_1_valid_r <= 0;
        ip_pc_paddle_2_valid_r <= 0;
        
        game_run <= 1;
        game_reset <= 0;

        ball_X_reset <= 0;
        ball_Y_reset <= 0;
        ball_Z_reset <= 0;

        ball_velocity_X_reset <= 0;
        ball_velocity_Y_reset <= 0;
        ball_velocity_Z_reset <= 0;
    end else begin
        // IP input
        if(ip_pc_paddle_1_valid) begin
            ip_pc_paddle_1_valid_r <= 1;
        end
        if(ip_pc_paddle_2_valid) begin
            ip_pc_paddle_2_valid_r <= 1;
        end

        // Microblaze input
        if(~within_bounds) begin
            game_run <= 0;
        end

        // TODO: Implement AXI-Lite interface
    end
end

/////////////////////////////////////////////////////////////////////////////
//////////////// Ball's Collision detection & its Position //////////////////
/////////////////////////////////////////////////////////////////////////////

wire debug_table_collision_x1;
wire debug_table_collision_x2;
wire debug_table_collision_y1;
wire debug_table_collision_y2;
wire debug_table_collision_z1;
wire debug_table_collision_z2;

wire signed [23:0] debug_table_1;
wire signed [23:0] debug_table_2;
assign debug_table_1 = TABLE_X - TABLE_dX - BALL_RADIUS;
assign debug_table_2 = TABLE_Y - TABLE_dY - BALL_RADIUS;

assign debug_table_collision_x1 = ball_X_p1_new > (TABLE_X - TABLE_dX - BALL_RADIUS);
assign debug_table_collision_x2 = ball_X_p1_new < (TABLE_X + TABLE_dX + BALL_RADIUS);
assign debug_table_collision_y1 = ball_Y_p1_new > (TABLE_Y - TABLE_dY - BALL_RADIUS);
assign debug_table_collision_y2 = ball_Y_p1_new < (TABLE_Y + TABLE_dY + BALL_RADIUS);
assign debug_table_collision_z1 = ball_Z_p1_new > (TABLE_Z - TABLE_dZ - BALL_RADIUS);
assign debug_table_collision_z2 = ball_Z_p1_new < (TABLE_Z + TABLE_dZ + BALL_RADIUS);

always @(posedge aclk) begin
   if (~aresetn) begin
        // Physics state registers
        paddle_1_X_p1 <= PADDLE1_X;
        paddle_1_Y_p1 <= PADDLE1_Y;
        paddle_1_Z_p1 <= PADDLE1_Z;

        paddle_2_X_p1 <= PADDLE2_X;
        paddle_2_Y_p1 <= PADDLE2_Y;
        paddle_2_Z_p1 <= PADDLE2_Z;
        
        paddle_1_velocity_X <= 0;
        paddle_1_velocity_Y <= 0;
        paddle_1_velocity_Z <= 0;

        paddle_2_velocity_X <= 0;
        paddle_2_velocity_Y <= 0;
        paddle_2_velocity_Z <= 0;

        ball_X_p1 <= BALL_X;
        ball_Y_p1 <= BALL_Y;
        ball_Z_p1 <= BALL_Z;

        ball_velocity_X <= BALL_VEL_X;
        ball_velocity_Y <= BALL_VEL_Y;
        ball_velocity_Z <= BALL_VEL_Z;

        ball_X_p1_new <= 0;
        ball_Y_p1_new <= 0;
        ball_Z_p1_new <= 0;

        ball_impact_side <= 0;
        within_bounds <= 1;

        // Collision result registers
        collide_paddle_1 <= 0;
        collide_paddle_2 <= 0;
        collide_table <= 0;
        collide_net <= 0;
        collide_bounds <= 0;
        
        // Delay counter
        delay_counter <= 0;

        // State machine
        state <= START;
    end 
    else begin
        case (state)
            IDLE : begin
                if(start) begin
                    state <= START;
                end
                else begin
                    state <= IDLE;
                end
            end
            START : begin
                // TODO: Don't set velocity every time, only when a new position is provided
                if (ip_pc_paddle_1_valid) begin
                        paddle_1_velocity_X     <= ip_pc_paddle_1_X - paddle_1_X_p1;
                        paddle_1_velocity_Y     <= ip_pc_paddle_1_Y - paddle_1_Y_p1;
                        paddle_1_velocity_Z     <= ip_pc_paddle_1_Z - paddle_1_Z_p1;
                        paddle_1_X_p1	    <= ip_pc_paddle_1_X;   
                        paddle_1_Y_p1	    <= ip_pc_paddle_1_Y;
                        paddle_1_Z_p1	    <= ip_pc_paddle_1_Z;   
                end
                if (ip_pc_paddle_2_valid) begin
                        paddle_2_velocity_X     <= ip_pc_paddle_2_X - paddle_2_X_p1;
                        paddle_2_velocity_Y     <= ip_pc_paddle_2_Y - paddle_2_Y_p1;
                        paddle_2_velocity_Z     <= ip_pc_paddle_2_Z - paddle_2_Z_p1;
                        paddle_2_X_p1	    <= ip_pc_paddle_2_X;   
                        paddle_2_Y_p1	    <= ip_pc_paddle_2_Y;
                        paddle_2_Z_p1	    <= ip_pc_paddle_2_Z;
                end

                if(game_reset) begin
                    ball_X_p1 <= ball_X_reset;
                    ball_Y_p1 <= ball_Y_reset;
                    ball_Z_p1 <= ball_Z_reset;
                    ball_velocity_X <= ball_velocity_X_reset;
                    ball_velocity_Y <= ball_velocity_Y_reset;
                    ball_velocity_Z <= ball_velocity_Z_reset;
                    within_bounds <= 1;
                end

                if(game_run && (delay_counter > DELAY_TIME)) begin
                    state <= NEW_BALL_POSITION;
                    delay_counter <= 0;
                end
                else begin
                    state <= START;
                    delay_counter <= delay_counter + 1;
                end
            end
            NEW_BALL_POSITION: begin
                ball_X_p1_new <= ball_X_p1 + ball_velocity_X;
                ball_Y_p1_new <= ball_Y_p1 + ball_velocity_Y;
                ball_Z_p1_new <= ball_Z_p1 + ball_velocity_Z;
                state <= BALL_COLLISION;
                
                delay_counter <= delay_counter + 1;
            end
            BALL_COLLISION: begin
                // Detect Ball Collision. Priority order is Table, Paddle 1, Paddle 2, Net
                if( (ball_X_p1_new > (TABLE_X - TABLE_dX - BALL_RADIUS)) &&
                    (ball_X_p1_new < (TABLE_X + TABLE_dX + BALL_RADIUS)) &&
                    (ball_Y_p1_new > (TABLE_Y - TABLE_dY - BALL_RADIUS)) &&
                    (ball_Y_p1_new < (TABLE_Y + TABLE_dY + BALL_RADIUS)) &&
                    (ball_Z_p1_new > (TABLE_Z - TABLE_dZ - BALL_RADIUS)) &&
                    (ball_Z_p1_new < (TABLE_Z + TABLE_dZ + BALL_RADIUS)))begin
                    
                    collide_table <= 1;
                    if(ball_Y_p1_new > TABLE_Y) begin
                        ball_impact_side <= 1;
                    end
                    else begin
                        ball_impact_side <= 0;
                    end
                    state <= TABLE_COLLISION;
                end
                else if(ball_X_p1_new > (paddle_1_X_p1 - PADDLE_dX - BALL_RADIUS) &&
                        ball_X_p1_new < (paddle_1_X_p1 + PADDLE_dX + BALL_RADIUS) &&
                        ball_Y_p1_new > (paddle_1_Y_p1 - PADDLE_dY - BALL_RADIUS) &&
                        ball_Y_p1_new < (paddle_1_Y_p1 + PADDLE_dY + BALL_RADIUS) &&
                        ball_Z_p1_new > (paddle_1_Z_p1 - PADDLE_dZ - BALL_RADIUS) &&
                        ball_Z_p1_new < (paddle_1_Z_p1 + PADDLE_dZ + BALL_RADIUS)) begin
                    collide_paddle_1 <= 1;
                    state <= PADDLE_1_COLLISION;
                end
                else if(ball_X_p1_new > (paddle_2_X_p1 - PADDLE_dX - BALL_RADIUS) &&
                        ball_X_p1_new < (paddle_2_X_p1 + PADDLE_dX + BALL_RADIUS) &&
                        ball_Y_p1_new > (paddle_2_Y_p1 - PADDLE_dY - BALL_RADIUS) &&
                        ball_Y_p1_new < (paddle_2_Y_p1 + PADDLE_dY + BALL_RADIUS) &&
                        ball_Z_p1_new > (paddle_2_Z_p1 - PADDLE_dZ - BALL_RADIUS) &&
                        ball_Z_p1_new < (paddle_2_Z_p1 + PADDLE_dZ + BALL_RADIUS)) begin
                    collide_paddle_2 <= 1;
                    state <= PADDLE_2_COLLISION;
                end
                else if(ball_X_p1_new > (NET_X - NET_dX - BALL_RADIUS) &&
                        ball_X_p1_new < (NET_X + NET_dX + BALL_RADIUS) &&
                        ball_Y_p1_new > (NET_Y - NET_dY - BALL_RADIUS) &&
                        ball_Y_p1_new < (NET_Y + NET_dY + BALL_RADIUS) &&
                        ball_Z_p1_new > (NET_Z - NET_dZ - BALL_RADIUS) &&
                        ball_Z_p1_new < (NET_Z + NET_dZ + BALL_RADIUS)) begin
                    collide_net <= 1;
                    state <= NET_COLLISION;
                end
                else if(ball_X_p1_new < (BOUNDS_X1) ||
                        ball_X_p1_new > (BOUNDS_X2) ||
                        ball_Y_p1_new < (BOUNDS_Y1) ||
                        ball_Y_p1_new > (BOUNDS_Y2) ||
                        ball_Z_p1_new < (BOUNDS_Z1) ||
                        ball_Z_p1_new > (BOUNDS_Z2)) begin
                    collide_bounds <= 1;
                    within_bounds <= 0;
                    state <= IDLE;
                end
                else begin
                    collide_paddle_1 <= 0;
                    collide_paddle_2 <= 0;
                    collide_table <= 0;
                    collide_net <= 0;
                    collide_bounds <= 0;
                    state <= UPDATE_BALL_POS;
                end
                
                delay_counter <= delay_counter + 1;
            end

            // TODO: Implement loss of energy in each collision
            // TODO: Check if the ball bounces on the same side twice
            TABLE_COLLISION: begin
                ball_velocity_X <= ball_velocity_X;
                ball_velocity_Y <= ball_velocity_Y;
                ball_velocity_Z <= -ball_velocity_Z;

                state <= UPDATE_BALL_POS;
                
                delay_counter <= delay_counter + 1;
            end

            // TODO: Update to change the way the game feels
            PADDLE_1_COLLISION: begin
                ball_velocity_X <= ball_velocity_X;
                ball_velocity_Y <= -ball_velocity_Y;
                ball_velocity_Z <= ball_velocity_Z;

                state <= UPDATE_BALL_POS;
                
                delay_counter <= delay_counter + 1;
            end

            // TODO: Update to change the way the game feels
            PADDLE_2_COLLISION: begin
                ball_velocity_X <= ball_velocity_X;
                ball_velocity_Y <= -ball_velocity_Y;
                ball_velocity_Z <= ball_velocity_Z;

                state <= UPDATE_BALL_POS;
                
                delay_counter <= delay_counter + 1;
            end

            NET_COLLISION: begin
                ball_velocity_X <= ball_velocity_X;
                ball_velocity_Y <= ball_velocity_Y;
                ball_velocity_Z <= -ball_velocity_Z;

                state <= UPDATE_BALL_POS;
                
                delay_counter <= delay_counter + 1;
            end

	        UPDATE_BALL_POS : begin
                ball_X_p1 <= ball_X_p1 + ball_velocity_X;
                ball_Y_p1 <= ball_Y_p1 + ball_velocity_Y;
                ball_Z_p1 <= ball_Z_p1 + ball_velocity_Z;

                ball_velocity_Z <= ball_velocity_Z + Gravity;

                state <= START;
                
                delay_counter <= delay_counter + 1;
            end
            default: begin
                state <= START;
                
                delay_counter <= delay_counter + 1;
            end
        endcase
    end
end

// Pipeline Stage 1 Outputs
assign pc_gr_ball_X_p1  = ball_X_p1;
assign pc_gr_ball_Y_p1  = ball_Y_p1;
assign pc_gr_ball_Z_p1  = ball_Z_p1;

assign pc_gr_paddle_1_X_p1  = paddle_1_X_p1;
assign pc_gr_paddle_1_Y_p1  = paddle_1_Y_p1;
assign pc_gr_paddle_1_Z_p1  = paddle_1_Z_p1;

assign pc_gr_paddle_2_X_p1  = paddle_2_X_p1;
assign pc_gr_paddle_2_Y_p1  = paddle_2_Y_p1;
assign pc_gr_paddle_2_Z_p1  = paddle_2_Z_p1;

endmodule
