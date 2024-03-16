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
localparam SCALE = 256;
localparam [23:0] TABLE_DIMS [2:0] = '{24'd762 * SCALE, 24'd1370 * SCALE, 24'd15 * SCALE};
localparam [23:0] NET_DIMS [2:0] = '{24'd762 * SCALE, 24'd10 * SCALE, 24'd30 * SCALE};
localparam [23:0] PADDLE_DIMS [2:0] = '{24'd60 * SCALE, 24'd10 * SCALE, 24'd70 * SCALE};
localparam [23:0] BALL_RADIUS = 24'd20 * SCALE;

localparam signed [23:0] TABLE_POS [2:0] = '{24'd0 * SCALE, 24'd1370 * SCALE, 24'd740 * SCALE};
localparam signed [23:0] NET_POS [2:0] = '{24'd762 * SCALE, 24'd1370 * SCALE, 24'd15 * SCALE};
localparam signed [23:0] PADDLE1_POS [2:0] = '{24'd0 * SCALE, 24'd0 * SCALE, 24'd790 * SCALE};
localparam signed [23:0] PADDLE2_POS [2:0] = '{24'd0 * SCALE, 24'd2740 * SCALE, 24'd790 * SCALE};
localparam signed [23:0] BALL_POS [2:0] = '{24'd0 * SCALE, 24'd1370 * SCALE, 24'd870 * SCALE};

localparam signed [23:0] BALL_VEL [2:0] = '{24'd0.03 * SCALE, 24'd-0.8 * SCALE, 24'd0 * SCALE};

localparam Gravity = -1;

localparam Loss = 0.9; // Loss of energy in each collision, Need to figure out how to implement using integer math

localparam signed BOUNDS_X [1:0] = '{24'd0 * SCALE, 24'd640 * SCALE};
localparam signed BOUNDS_Y [1:0] = '{24'd0 * SCALE, 24'd480 * SCALE};
localparam signed BOUNDS_Z [1:0] = '{24'd0 * SCALE, 24'd480 * SCALE};

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

reg signed [23:0] ball_velocity_X_new;
reg signed [23:0] ball_velocity_Y_new;
reg signed [23:0] ball_velocity_Z_new;

reg ball_impact_side; // Indicates which plaer side last impacted the ball
reg within_bounds; // Indicates if the ball is within the bounds of the table

// Collision result registers
reg collide_paddle_1; // Indicates if the ball has collided
reg collide_paddle_2;
reg collide_table;
reg collide_net;

// State machine
reg [3:0] state;

parameter [3:0] START               = 4'd0,
                NEW_BALL_POSITION   = 4'd1,
                BALL_COLLISION      = 4'd2,

                PADDLE_1_COLLISION  = 4'd3,
                PADDLE_2_COLLISION  = 4'd4,
                TABLE_COLLISION     = 4'd5,
                NET_COLLISION       = 4'd6,

                UPDATE_BALL_POS     = 4'd3;

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
        if(ip_pc_paddle_1_valid) begin
            ip_pc_paddle_1_valid_r <= 1;
        end
        if(ip_pc_paddle_2_valid) begin
            ip_pc_paddle_2_valid_r <= 1;
        end
    end
end

/////////////////////////////////////////////////////////////////////////////
//////////////// Ball's Collision detection & its Position //////////////////
/////////////////////////////////////////////////////////////////////////////

always @(posedge aclk) begin
   if (~aresetn) begin
        // Physics state registers
        paddle_1_X_p1 <= PADDLE1_POS[0];
        paddle_1_Y_p1 <= PADDLE1_POS[1];
        paddle_1_Z_p1 <= PADDLE1_POS[2];

        paddle_2_X_p1 <= PADDLE2_POS[0];
        paddle_2_Y_p1 <= PADDLE2_POS[1];
        paddle_2_Z_p1 <= PADDLE2_POS[2];
        
        paddle_1_velocity_X <= 0;
        paddle_1_velocity_Y <= 0;
        paddle_1_velocity_Z <= 0;

        paddle_2_velocity_X <= 0;
        paddle_2_velocity_Y <= 0;
        paddle_2_velocity_Z <= 0;

        ball_X_p1 <= BALL_POS[0];
        ball_Y_p1 <= BALL_POS[1];
        ball_Z_p1 <= BALL_POS[2];

        ball_velocity_X <= BALL_VEL[0];
        ball_velocity_Y <= BALL_VEL[1];
        ball_velocity_Z <= BALL_VEL[2];

        ball_X_p1_new <= 0;
        ball_Y_p1_new <= 0;
        ball_Z_p1_new <= 0;

        ball_velocity_X_new <= 0;
        ball_velocity_Y_new <= 0;
        ball_velocity_Z_new <= 0;

        reg ball_impact_side <= 0;
        reg within_bounds <= 1;

        // Collision result registers
        reg collide_paddle_1 <= 0;
        reg collide_paddle_2 <= 0;
        reg collide_table <= 0;
        reg collide_net <= 0;

        // State machine
        reg [3:0] state <= START;
    end else begin
        case (state)
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

                if(reset) begin
                    ball_X_p1 <= ball_X_reset;
                    ball_Y_p1 <= ball_Y_reset;
                    ball_Z_p1 <= ball_Z_reset;
                    ball_velocity_X <= ball_velocity_X_reset;
                    ball_velocity_Y <= ball_velocity_Y_reset;
                    ball_velocity_Z <= ball_velocity_Z_reset;
                end
                if(!game_run) begin
                    state <= START;
                end
                else begin
                    state <= NEW_BALL_POSITION;
                end
            end
            NEW_BALL_POSITION: begin
                ball_X_p1_new <= ball_X_p1 + ball_velocity_X;
                ball_Y_p1_new <= ball_Y_p1 + ball_velocity_Y;
                ball_Z_p1_new <= ball_Z_p1 + ball_velocity_Z;
                state <= BALL_COLLISION;
            end
            BALL_COLLISION: begin
                // Detect Ball Collision. Priority order is Table, Paddle 1, Paddle 2, Net
                if( ball_X_p1_new > ((TABLE_POS[0] - TABLE_DIMS[0] - BALL_RADIUS)) &&
                    ball_X_p1_new < (TABLE_POS[0] + TABLE_DIMS[0] + BALL_RADIUS) &&
                    ball_Y_p1_new > ((TABLE_POS[1] - TABLE_DIMS[1] - BALL_RADIUS)) &&
                    ball_Y_p1_new < (TABLE_POS[1] + TABLE_DIMS[1] + BALL_RADIUS) &&
                    ball_Z_p1_new > ((TABLE_POS[2] - TABLE_DIMS[2] - BALL_RADIUS)) &&
                    ball_Z_p1_new < (TABLE_POS[2] + TABLE_DIMS[2] + BALL_RADIUS)) begin
                    
                    collide_table <= 1;
                    if(ball_Y_p1_new > TABLE_POS[1]) begin
                        ball_impact_side <= 1;
                    end
                    else begin
                        ball_impact_side <= 0;
                    end
                    state <= TABLE_COLLISION;
                end
                else if(ball_X_p1_new > (paddle_1_X_p1 - PADDLE_DIMS[0] - BALL_RADIUS) &&
                        ball_X_p1_new < (paddle_1_X_p1 + PADDLE_DIMS[0] + BALL_RADIUS) &&
                        ball_Y_p1_new > (paddle_1_Y_p1 - PADDLE_DIMS[1] - BALL_RADIUS) &&
                        ball_Y_p1_new < (paddle_1_Y_p1 + PADDLE_DIMS[1] + BALL_RADIUS) &&
                        ball_Z_p1_new > (paddle_1_Z_p1 - BALL_RADIUS) &&
                        ball_Z_p1_new < (paddle_1_Z_p1 + PADDLE_DIMS[2] + BALL_RADIUS)) begin
                    
                    collide_paddle_1 <= 1;
                    state <= PADDLE_1_COLLISION;
                end
                else if(ball_X_p1_new > (paddle_2_X_p1 - PADDLE_DIMS[0] - BALL_RADIUS) &&
                        ball_X_p1_new < (paddle_2_X_p1 + PADDLE_DIMS[0] + BALL_RADIUS) &&
                        ball_Y_p1_new > (paddle_2_Y_p1 - PADDLE_DIMS[1] - BALL_RADIUS) &&
                        ball_Y_p1_new < (paddle_2_Y_p1 + PADDLE_DIMS[1] + BALL_RADIUS) &&
                        ball_Z_p1_new > (paddle_2_Z_p1 - BALL_RADIUS) &&
                        ball_Z_p1_new < (paddle_2_Z_p1 + PADDLE_DIMS[2] + BALL_RADIUS)) begin
                    
                    collide_paddle_2 <= 1;
                    state <= PADDLE_2_COLLISION;
                end
                else if(ball_X_p1_new > (NET_POS[0] - NET_DIMS[0] - BALL_RADIUS) &&
                        ball_X_p1_new < (NET_POS[0] + NET_DIMS[0] + BALL_RADIUS) &&
                        ball_Y_p1_new > (NET_POS[1] - NET_DIMS[1] - BALL_RADIUS) &&
                        ball_Y_p1_new < (NET_POS[1] + NET_DIMS[1] + BALL_RADIUS) &&
                        ball_Z_p1_new > (NET_POS[2] - BALL_RADIUS) &&
                        ball_Z_p1_new < (NET_POS[2] + NET_DIMS[2] + BALL_RADIUS)) begin
                    
                    collide_net <= 1;
                    state <= NET_COLLISION;
                end
                else begin
                    collide_paddle_1 <= 0;
                    collide_paddle_2 <= 0;
                    collide_table <= 0;
                    collide_net <= 0;
                    state <= UPDATE_BALL_POS;
                end
            end

            // TODO: Implement loss of energy in each collision
            TABLE_COLLISION: begin
                ball_velocity_X_new <= ball_velocity_X;
                ball_velocity_Y_new <= ball_velocity_Y;
                ball_velocity_Z_new <= -ball_velocity_Z;

                state <= UPDATE_BALL_POS;
            end

            // TODO: Update to change the way the game feels
            PADDLE_1_COLLISION: begin
                ball_velocity_X_new <= ball_velocity_X;
                ball_velocity_Y_new <= -ball_velocity_Y;
                ball_velocity_Z_new <= ball_velocity_Z;

                state <= UPDATE_BALL_POS;
            end

            // TODO: Update to change the way the game feels
            PADDLE_2_COLLISION: begin
                ball_velocity_X_new <= ball_velocity_X;
                ball_velocity_Y_new <= -ball_velocity_Y;
                ball_velocity_Z_new <= ball_velocity_Z;

                state <= UPDATE_BALL_POS;
            end

            NET_COLLISION: begin
                ball_velocity_X_new <= ball_velocity_X;
                ball_velocity_Y_new <= ball_velocity_Y;
                ball_velocity_Z_new <= -ball_velocity_Z;

                state <= UPDATE_BALL_POS;
            end

	        BALL_COLLISION : begin
                if(collide_paddle_1 || collide_paddle_2 || collide_table || collide_net) begin
                    ball_velocity_X <= ball_velocity_X_new;
                    ball_velocity_Y <= ball_velocity_Y_new;
                    ball_velocity_Z <= ball_velocity_Z_new;
                end
                ball_X_p1 <= ball_X_p1 + ball_velocity_X;
                ball_Y_p1 <= ball_Y_p1 + ball_velocity_Y;
                ball_Z_p1 <= ball_Z_p1 + ball_velocity_Z;

                ball_velocity_Z <= ball_velocity_Z + Gravity;

                state <= START;
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
