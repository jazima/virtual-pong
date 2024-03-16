// ip => Image Processing
// pc => Physics Core
// gr => Graphics Rendering



module physics_core # (
      parameter	  WIDTH     = 75,      
      parameter	  HEIGHT    = 50      
   )    (
   input wire		    aclk,	    // 25 MHz
   input wire		    aresetn,
   
   // Incoming Raw Coordinates from Image Processing
   input wire [15:0]	ip_pc_paddle_1_X,
   input wire [15:0]	ip_pc_paddle_1_Y,
   input wire [15:0]    ip_pc_paddle_1_Z,
   input wire           ip_pc_paddle_1_valid,

   input wire [15:0]	ip_pc_paddle_2_X,
   input wire [15:0]	ip_pc_paddle_2_Y,
   input wire [15:0]    ip_pc_paddle_2_Z,
   input wire           ip_pc_paddle_2_valid,
   
   // Comes from CPU
   input wire [15:0]	iBall_X,
   input wire [15:0]	iBall_Y,
   input wire [15:0]	iBall_Z,

   // To be used by Rendering Logic
   output wire [15:0]	pc_gr_ball_X_p1,
   output wire [15:0]	pc_gr_ball_Y_p1,
   output wire [15:0]	pc_gr_ball_Z_p1,
   
   output wire [15:0]  pc_gr_paddle_1_X_p1,  
   output wire [15:0]  pc_gr_paddle_1_Y_p1, 
   output wire [15:0]  pc_gr_paddle_1_Z_p1, 
    
   output wire [15:0]  pc_gr_paddle_2_X_p1, 
   output wire [15:0]  pc_gr_paddle_2_Y_p1,  
   output wire [15:0]  pc_gr_paddle_2_Z_p1
   

);


parameter GRAVITY        = 10;   // 9.8 m/s2
parameter SCREEN_HEIGHT  = 480;
parameter SCREEN_WIDTH   = 640;
parameter BALL_RADIUS    = 16;


parameter [2:0] BALL_IDLE               = 3'd0,
                PADDLE_1_BALL_COLLISION = 3'd1,
                PADDLE_2_BALL_COLLISION = 3'd2,
                BALL_COLLISION          = 3'd3; 


// Pipeline Stage 1
reg [15:0]  ip_pc_paddle_1_X_p1; 
reg [15:0]  ip_pc_paddle_1_Y_p1; 
reg [15:0]  ip_pc_paddle_1_Z_p1;  
reg [15:0]  ip_pc_paddle_2_X_p1; 
reg [15:0]  ip_pc_paddle_2_Y_p1;  
reg [15:0]  ip_pc_paddle_2_Z_p1;
   
   
reg [15:0] paddle_1_velocity_X;
reg [15:0] paddle_1_velocity_Y;
reg [15:0] paddle_1_velocity_Z;

reg [15:0] paddle_2_velocity_X;
reg [15:0] paddle_2_velocity_Y;
reg [15:0] paddle_2_velocity_Z;


reg [15:0] ball_X_p1;
reg [15:0] ball_Y_p1;
reg [15:0] ball_Z_p1; 

reg [2:0] ball_state;

           
             
/////////////////////////////////////////////////////////////////////////////
//////////////// Ball's Collision detection & its Position //////////////////
/////////////////////////////////////////////////////////////////////////////


always @(posedge aclk) begin
   if (~aresetn) begin
      ball_X_p1	                <= 16'd0;
	  ball_Y_p1	                <= 16'd0;
	  ball_Z_p1	                <= 16'd0;
      ip_pc_paddle_1_X_p1	    <= 16'd0;
      ip_pc_paddle_1_Y_p1		<= 16'd0;
      ip_pc_paddle_1_Z_p1		<= 16'd0;
      ip_pc_paddle_2_X_p1	    <= 16'd0;
      ip_pc_paddle_2_Y_p1		<= 16'd0;
      ip_pc_paddle_2_Z_p1		<= 16'd0;
      paddle_1_velocity_X       <= 16'd0;
      paddle_1_velocity_Y       <= 16'd0;
      paddle_1_velocity_Z       <= 16'd0;
      paddle_2_velocity_X       <= 16'd0;
      paddle_2_velocity_Y       <= 16'd0;
      paddle_2_velocity_Z       <= 16'd0;
      ball_state                <= BALL_IDLE;
    end else begin
        case (ball_state)
            BALL_IDLE : begin
               if (ip_pc_paddle_1_valid) begin
	               ip_pc_paddle_1_X_p1	   <= ip_pc_paddle_1_X;   
	               ip_pc_paddle_1_Y_p1	   <= ip_pc_paddle_1_Y;
	               ip_pc_paddle_1_Z_p1	   <= ip_pc_paddle_1_Z;   
	           end
	           if (ip_pc_paddle_2_valid) begin
	               ip_pc_paddle_2_X_p1	   <= ip_pc_paddle_1_X;   
	               ip_pc_paddle_2_Y_p1	   <= ip_pc_paddle_1_Y;
	               ip_pc_paddle_2_Z_p1	   <= ip_pc_paddle_1_Z;  
	           end
	           paddle_1_velocity_X       <= 16'd0;
               paddle_1_velocity_Y       <= 16'd0;
               paddle_1_velocity_Z       <= 16'd0;
               paddle_2_velocity_X       <= 16'd0;
               paddle_2_velocity_Y       <= 16'd0;
               paddle_2_velocity_Z       <= 16'd0;
               ball_X_p1                 <= iBall_X;
	           ball_Y_p1                 <= iBall_Y;
	           ball_Z_p1                 <= iBall_Z;
	           
	           
	           // Detect Ball and Paddle Collision.
	           if ( (ball_X_p1 < (ip_pc_paddle_1_X_p1 + WIDTH) ) && (ball_X_p1 > ip_pc_paddle_1_X_p1 ) && 
	                (ball_Y_p1 < (ip_pc_paddle_1_Y_p1 + HEIGHT)) && (ball_Y_p1 > ip_pc_paddle_1_Y_p1 ) && 
	                ((ball_Z_p1 - ip_pc_paddle_1_Z_p1) == BALL_RADIUS) ) begin
                   ball_state  <= PADDLE_1_BALL_COLLISION;
	           end else if ( (ball_X_p1 < (ip_pc_paddle_2_X_p1 + WIDTH) ) && (iBall_X > ip_pc_paddle_2_X_p1 ) && 
	                         (ball_Y_p1 < (ip_pc_paddle_2_Y_p1 + HEIGHT)) && (iBall_Y > ip_pc_paddle_2_Y_p1 ) && 
	                         ((ball_Z_p1 + BALL_RADIUS) == ip_pc_paddle_2_Z_p1) ) begin
                   ball_state  <= PADDLE_2_BALL_COLLISION;
               end else begin
                   ball_state  <= BALL_IDLE; 
               end
            end
            

            PADDLE_1_BALL_COLLISION : begin
                if (ip_pc_paddle_1_X > ip_pc_paddle_1_X_p1) begin
	               paddle_1_velocity_X   <= (ip_pc_paddle_1_X - ip_pc_paddle_1_X_p1);
	            end
	            if (ip_pc_paddle_1_Y > ip_pc_paddle_1_Y_p1) begin
	               paddle_1_velocity_Y   <= (ip_pc_paddle_1_Y - ip_pc_paddle_1_Y_p1);
	            end
	            if (ip_pc_paddle_1_Z > ip_pc_paddle_1_Z) begin
	               paddle_1_velocity_Z   <= (ip_pc_paddle_1_Z - ip_pc_paddle_1_Z_p1);
	            end
	            ball_state  <= BALL_COLLISION;
	       end
	        
	       PADDLE_2_BALL_COLLISION : begin
                if (ip_pc_paddle_2_X > ip_pc_paddle_2_X_p1) begin
	               paddle_2_velocity_X   <= (ip_pc_paddle_2_X - ip_pc_paddle_2_X_p1);
	            end
	            if (ip_pc_paddle_2_Y > ip_pc_paddle_2_Y_p1) begin
	               paddle_2_velocity_Y   <= (ip_pc_paddle_2_Y - ip_pc_paddle_2_Y_p1);
	            end
	            if (ip_pc_paddle_2_Z > ip_pc_paddle_2_Z) begin
	               paddle_2_velocity_Z   <= (ip_pc_paddle_2_Z - ip_pc_paddle_2_Z_p1);
	            end
	            ball_state  <= BALL_COLLISION;
	        end
	        

	        BALL_COLLISION : begin

            // Only Ball & Paddle 1 is done here. 
            // Similarly, Ball & Paddle 2 need to be done here.

	           if (ball_X_p1 >= 16'd640) begin	     
	               ball_X_p1       <= SCREEN_WIDTH >> 1;
	               ball_Y_p1       <= ball_Y_p1 + paddle_1_velocity_Y - GRAVITY;
	               ball_Z_p1       <= ball_Z_p1 + paddle_1_velocity_Z - GRAVITY;	         
                end else if (ball_Y_p1 >= 16'd480) begin	     //Y > 480, Ball hits Top Wall.
	               ball_X_p1       <= ball_X_p1 + paddle_1_velocity_X - GRAVITY;
	               ball_Y_p1       <= SCREEN_HEIGHT >> 1;
	               ball_Z_p1       <= ball_Z_p1 + paddle_1_velocity_Z - GRAVITY;
                end else if (ball_Y_p1 == 0) begin	          
	               ball_X_p1       <= ball_X_p1 + paddle_1_velocity_X - GRAVITY;
	               ball_Y_p1       <= SCREEN_HEIGHT >> 1;
	               ball_Z_p1       <= ball_Z_p1 + paddle_1_velocity_Z - GRAVITY;
                end else begin	// No Collisions
	               ball_X_p1	   <= ball_X_p1 + paddle_1_velocity_X - GRAVITY;
	               ball_Y_p1	   <= ball_Y_p1 + paddle_1_velocity_Y - GRAVITY;
	               ball_Z_p1	   <= ball_Z_p1 + paddle_1_velocity_Z - GRAVITY;
	            end 
	         
             
	               
	           // Detect Ball and Paddle Collision.
	           if ( (ball_X_p1 < (ip_pc_paddle_1_X_p1 + WIDTH) ) && (ball_X_p1 > ip_pc_paddle_1_X_p1 ) && 
	                (ball_Y_p1 < (ip_pc_paddle_1_Y_p1 + HEIGHT)) && (ball_Y_p1 > ip_pc_paddle_1_Y_p1 ) && 
	                ((ball_Z_p1 - ip_pc_paddle_1_Z_p1) == BALL_RADIUS) ) begin
                   ball_state  <= PADDLE_1_BALL_COLLISION;
	           end else if ( (ball_X_p1 < (ip_pc_paddle_2_X_p1 + WIDTH) ) && (iBall_X > ip_pc_paddle_2_X_p1 ) && 
	                         (ball_Y_p1 < (ip_pc_paddle_2_Y_p1 + HEIGHT)) && (iBall_Y > ip_pc_paddle_2_Y_p1 ) && 
	                         ((ball_Z_p1 + BALL_RADIUS) == ip_pc_paddle_2_Z_p1) ) begin
                   ball_state  <= PADDLE_2_BALL_COLLISION;
               end else begin
                   ball_state  <= BALL_IDLE; 
               end
	         end
        endcase
    end
end



// Pipeline Stage 1 Outputs
assign pc_gr_ball_X_p1  = ball_X_p1;
assign pc_gr_ball_Y_p1  = ball_Y_p1;
assign pc_gr_ball_Z_p1  = ball_Z_p1;

assign pc_gr_paddle_1_X_p1  = ip_pc_paddle_1_X_p1;
assign pc_gr_paddle_1_Y_p1  = ip_pc_paddle_1_Y_p1;
assign pc_gr_paddle_1_Z_p1  = ip_pc_paddle_1_Z_p1;

assign pc_gr_paddle_2_X_p1  = ip_pc_paddle_2_X_p1;
assign pc_gr_paddle_2_Y_p1  = ip_pc_paddle_2_Y_p1;
assign pc_gr_paddle_2_Z_p1  = ip_pc_paddle_2_Z_p1;


endmodule
