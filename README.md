PMBUS1.3_SLAVE
==============

Verilog implementation of Slave
//Name: Venkatesh karra
//Organization:University of Limerick
//Supervisor: Dr.Karl Rinne
//MASTER THESIS PM BUS SPECIFICATION 1.3
//AVS MASTER 
//REFERENCES:[1]PM.BUS 1.3 spcification
//				 [2] Advanced digital design by michale D cileleti

module AVS_SLAVE_STATEMACHINE#(
   // the following parameters can be overridden at module instantiation (see p. 180 of [1])
  parameter		NOC_HIGH=5,		// number of clock cycles during clk high period
  parameter		NOC_LOW=5
)(
/////////////////////////////////////////////////////////////////////////////////
//INPUTS AND OUTPUTS//
/////////////////////////////////////////////////////////////////////////////////
	
    input clk,
	 input rst,
	 input avs_mdata_sample_async,//data that comes from the master
	 input avs_clk_in,
	 input [15:0] avs_cmd_data,
	 output reg avs_sdata ,
	 output reg avs_mdata_sample_display,
	 output reg avs_sdata_transmit,
    output wire avs_mdata_sample_sync,
	 output wire avs_clk_slave,
	  output wire vgood,
	  output wire pwm,
	  output reg sledperfect,//glows when the computed crc for the recieved bits is zero(SLAVE)
	 output reg slederror   //glows when the computed crc for the recieved bits is not zero(SLAVE)
    );
	
	reg [15:0] discrete_filter_in;
//syncronises clock and data coming from the master external input	 
syncroniserclk N3(.clk(clk),.rst(rst),.sync_out_data(avs_mdata_sample_sync),.sync_out_clk(avs_clk_slave),.async_in_clk(avs_clk_in),
.async_in_data(avs_mdata_sample_async));



// ************************************************************************************************
// management of outputs avs_clk and avs_mdata
// ************************************************************************************************
reg				avs_sdata_clr;
reg				avs_sdata_set;
reg 				avs_sdata_set_transmit;
reg				avs_sdata_clr_transmit;
reg				avs_mdata_set_sample;
reg				avs_mdata_clr_sample;
always @(posedge clk) begin
  if (rst) begin avs_sdata<=1; end
  else if (avs_sdata_set) avs_sdata<=1;
  else if (avs_sdata_clr) avs_sdata<=0;
  end
  always @(posedge clk) begin
  if (rst) begin  avs_sdata_transmit <=1; end
  
  else if (avs_sdata_set_transmit) avs_sdata_transmit<=1;
  else if (avs_sdata_clr_transmit) avs_sdata_transmit<=0;
  end
always @(posedge clk) begin
	if(rst) avs_mdata_sample_display<=1;
  else if (avs_mdata_set_sample) avs_mdata_sample_display<=1;
  else if (avs_mdata_clr_sample) avs_mdata_sample_display<=0;	
end

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ************************************************************************************************
// management of counter_bits (bit counter)
// ************************************************************************************************
reg [5:0]			counter_bits;
reg [5:0]			counter_bits_value;
reg				counter_bits_load;
reg				counter_bits_dec;	// dec command
wire				counter_bits_z;		// flag that counter_bits has reached a value of zero
always @(posedge clk) begin
  if (rst) counter_bits<=0;
  else if (counter_bits_load) counter_bits<=counter_bits_value;
  else if (counter_bits_dec && (~counter_bits_z)) counter_bits<=counter_bits-1;
end
assign counter_bits_z=(counter_bits==0); 
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ************************************************************************************************
// management of counter_clks (clk cycle counter)
// ************************************************************************************************
reg [3:0]			counter_clks;
reg [3:0]			counter_clks_value;
reg				counter_clks_load;
wire				counter_clks_z;		// flag that counter_clks has reached a value of zero
always @(posedge clk) begin
  if (rst) counter_clks<=0;
  else if (counter_clks_load) counter_clks<=counter_clks_value;
  else if (~counter_clks_z) counter_clks<=counter_clks-1;	// if counter is not already zero, decrement it
end
assign counter_clks_z=(counter_clks==0); 	
//***************************************************************************
//data revieve from the slave
//***************************************************************************
reg [31:0] rx_slave; //stores 32 bits that comes from the slave
reg recieve_slave;	//used as the control signal every assertion shifts in data

always@(posedge clk )
	if(rst == 1) begin
		rx_slave<=0;
		end

	else if(recieve_slave)
		rx_slave<={rx_slave[30:0],avs_mdata_sample_sync};
	assign rx_slave_din = avs_mdata_sample_sync;	//used for the display of the data
//*********************************************************************************************************
//MANAGEMENT OF CRC
//*********************************************************************************************************
reg [2:0]crc_slave;
reg [2:0] crctransmit_slave;
reg compute_slave;
reg shiftcrc_slave;
reg clearcrc_slave;
wire [2:0] crcout_slave;
reg check_slave;

always@(posedge clk)
if(rst == 1) begin
	crc_slave<=0;
	crctransmit_slave<=0;
end
else if(clearcrc_slave)
crc_slave<=0;

else if(compute_slave)
begin

	crctransmit_slave<=crcout_slave;
end
else if(check_slave)	begin	//useful to check the value of crc after the reception of 32 bits
	crctransmit_slave<=crcout_slave;
		end
else if(shiftcrc_slave)  begin
		
		crctransmit_slave<={crctransmit_slave[1:0],1'b0};
		end
	
	
else if(shift_transmit_reg_slave | (load_transmit_reg_slave == 1) )   
	begin	
		
			     crc_slave[0] <=transmit_reg_dout_slave ^ crc_slave[2];
                       crc_slave[1] <= crc_slave[0]^(crc_slave[2]^transmit_reg_dout_slave);
                       crc_slave[2] <= crc_slave[1];
				end			  
else if(recieve_slave) //switching the crc engine to calculate the crc of the recieved bits
		begin
			 crc_slave[0] <=rx_slave_din ^ crc_slave[2];
                       crc_slave[1] <= crc_slave[0]^(crc_slave[2]^rx_slave_din );
                       crc_slave[2] <= crc_slave[1];
				end
else
		crc_slave<=crc_slave;
assign crcout_slave = crc_slave;
assign crc_dout_slave = crctransmit_slave[2];	


	
//////////////////////////////////////////////////////////////////////////////////
//controlling slave response 
//////////////////////////////////////////////////////////////////////////////////
reg [1:0] slave_ack;

reg statusalert;
reg reserved_stts;
reg mfrspfc_stts1;
reg mfrspfc_stts2;
reg [20:0] reserved;
reg filter_start;
always @(posedge clk)
	if(rst == 1)
		begin
			
			slave_ack	<=2'bx;
			discrete_filter_in<=0;
			statusalert<=0;
			reserved_stts<=0;
			mfrspfc_stts1<=0;
			mfrspfc_stts2<=0;
		
end
		else if(check_slave) //check transfers thr crc into crc transmit after 32 bits 
							//of recieved data
			begin
				if(crctransmit_slave == 0  )//if the computed crc is zero then the led perfect
										  //glows else led error glows.
					begin
			slave_ack	<=2'b00;
		
			statusalert<=0;
			reserved_stts<=0;
			mfrspfc_stts1<=0;
			mfrspfc_stts2<=0;
			
					end

				else if(crctransmit_slave != 0)
					begin
						slave_ack	<=2'b10;
						filter_start <= 0;
						statusalert<=0;
						reserved_stts<=0;
						mfrspfc_stts1<=0;
						mfrspfc_stts2<=0;
					end
					
			
						
end



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CALCULATING DISCRETE FILTER COEFFICIENTS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
reg [15:0] discrete_in;
always @(posedge clk)
if(rst == 1)
 discrete_in<=0;
else 
if(crctransmit_slave == 0 && check_slave == 1)
	discrete_in<= rx_slave[18:3];
   
//DISCRETE FILTER
Q_POINT_ARITHMITIC R1(.clk(clk),.rst(rst),.data_in(discrete_in),.vgood(vgood));

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//PWM INPUT
////////////////////////////////////////////////////////////////////////////////////////////////////////////
reg [10:0] pwm_in;
always @(posedge clk)
if(rst == 1)
 pwm_in<=0;
else 
if(crctransmit_slave == 0 && check_slave == 1)
	pwm_in<= rx_slave[13:3];
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
always @(posedge clk)
	if(rst == 1)
		begin
			sledperfect<=0;
			slederror<=0;
		end
		else if(check_slave == 1) //check transfers thr crc into crc transmit after 32 bits 
							//of recieved data
			begin
				if(crctransmit_slave == 0)//if the computed crc is zero then the led perfect
										  //glows else led error glows.
					begin
							sledperfect<=1;
							slederror<=0;
					end
				else
					begin
						sledperfect<=0;
							slederror<=1;
					end
end


/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
//INTERNAL REGISTERS//
//////////////////////////////////////////////////////////////////////////////////////
reg [28:0] transmit_reg_slave;				
wire [28:0] transmit_subframe_slave;
reg load_transmit_reg_slave;
reg shift_transmit_reg_slave;


assign transmit_subframe_slave = {slave_ack,vgood,1'b0,statusalert,reserved_stts,mfrspfc_stts1,mfrspfc_stts2,21'b0};
//{2'b11,1'b0,vgood,statusalert,reserved_stts,mfrspfc_stts1,mfrspfc_stts2,21'b0};
//////////////////////////////////////////////////////////////////////////////////////////
//****************************************************************************************
//transmit_reg//
//****************************************************************************************
always @(posedge clk) begin
  if (rst) transmit_reg_slave<=0;					// synchronous reset, according to [2]
  else if (load_transmit_reg_slave)transmit_reg_slave<=transmit_subframe_slave;
  else if (shift_transmit_reg_slave)begin 
  transmit_reg_slave<={transmit_reg_slave[27:0],1'b0}; end 	// shift left
end
assign transmit_reg_dout_slave=transmit_reg_slave[28];				// pointing at current bit to be applied to 







// ************************************************************************************************
// state machine (Moore, type 3, see [4]
// ************************************************************************************************
  // states
  localparam		STATE_COUNT=5;		// number of bits to encode states
  localparam		IDLE=0;
  localparam		RX_CLKH=1;
  localparam		RX_CLKL=2;
  localparam		TX_CLKH_SLAVE = 3;
  localparam		TX_CLKL_SLAVE = 4;
  localparam      TX_CRC_CLKH_SLAVE =5;
  localparam		TX_CRC_CLKL_SLAVE = 6;
  
  
  reg [STATE_COUNT-1:0]	currentstate, next_state;	// pcm master state machine
/////////////////////////////////////////////////////////////////////////////////
//state register//
/////////////////////////////////////////////////////////////////////////////////
always @(posedge clk) begin
	if(rst == 1)
		currentstate<=IDLE;
	else
		currentstate<=next_state;
end
////////////////////////////////////////////////////////////////////////////////
// SM next state logic combined with output logic
always @(*) begin
  // default outputs
  next_state=currentstate;
  counter_bits_value=0; counter_bits_load=0; counter_bits_dec=0;
  counter_clks_value=0; counter_clks_load=0;
	avs_sdata_set_transmit = 0;
	avs_sdata_clr_transmit = 0;
  avs_sdata_clr=0; avs_sdata_set=0;
  load_transmit_reg_slave=0; shift_transmit_reg_slave=0;
  compute_slave = 0; shiftcrc_slave = 0; clearcrc_slave = 0;
  recieve_slave = 0;  check_slave = 0;
  avs_mdata_set_sample = 0; avs_mdata_clr_sample = 0;

  case(currentstate)
	
		IDLE: begin
					
					clearcrc_slave = 1;
//					avs_clk_clr = 1;
					avs_sdata_set = 1;
					avs_sdata_set_transmit = 1;
					clearcrc_slave = 1;
  avs_mdata_set_sample = 1 ;
				if(!rx_slave_din  && avs_clk_slave )
					begin
						
					counter_bits_load=1; counter_bits_value=32;//loasding with 32 bits
								counter_clks_load=1; counter_clks_value=(NOC_HIGH-2);	
								
								
								
								
								next_state= RX_CLKH;
				end		
				end
RX_CLKH:  begin

 
										if (counter_clks_z) begin
										
										if(rx_slave_din)begin 
//										avs_sdata_set = 1; 
										avs_mdata_set_sample = 1;
										end
										else  begin 
//										avs_sdata_clr=1;  
										avs_mdata_clr_sample = 1;
										end
										
									counter_bits_dec=1;
										counter_clks_load=1; 
										
													counter_clks_value=(NOC_LOW-1);
														recieve_slave = 1;	
															next_state= RX_CLKL;
									end
    end
	RX_CLKL: begin
					if (counter_bits_z) begin check_slave = 1; 	end
					if (counter_clks_z) begin
							if (counter_bits_z) begin
			counter_bits_load=1; counter_bits_value=29;							
				counter_clks_load=1; counter_clks_value=(NOC_HIGH-1);		
				load_transmit_reg_slave  = 1;						
if(transmit_reg_dout_slave) 
								begin avs_sdata_set=1; 
								avs_sdata_set_transmit=1;
								end
								else begin avs_sdata_clr = 1 ;
											 avs_sdata_clr_transmit = 1 ;
							end
								next_state = TX_CLKH_SLAVE;
								end 
								
							else begin
							
									
								counter_clks_load=1; counter_clks_value=(NOC_HIGH-1);
								next_state=RX_CLKH;
							end
					end
				end
				


TX_CLKH_SLAVE: begin 						

					if (counter_clks_z) begin
								
							shift_transmit_reg_slave=1;
							 counter_bits_dec=1;
							counter_clks_load=1; counter_clks_value=(NOC_LOW-1);
							next_state=TX_CLKL_SLAVE;
							end
							
							
					
			    end
	TX_CLKL_SLAVE: begin
					if (counter_bits_z) compute_slave = 1;
					if (counter_clks_z) begin
							if (counter_bits_z) begin
									
							
							
								counter_bits_load=1; counter_bits_value=3;
								counter_clks_load=1; counter_clks_value=(NOC_HIGH-1);
								if(crc_dout_slave) begin avs_sdata_set=1; 
								avs_sdata_set_transmit=1;
								end
								else begin avs_sdata_clr = 1 ;
											 avs_sdata_clr_transmit = 1 ;
							end

								
								next_state=TX_CRC_CLKH_SLAVE;
								end 
								
							else begin
								if (transmit_reg_dout_slave) begin 
									avs_sdata_set=1; 
									avs_sdata_set_transmit=1;
								end
							else 
									begin avs_sdata_clr = 1 ;
											avs_sdata_clr_transmit = 1 ;
							end

								
								
								counter_clks_load=1; counter_clks_value=(NOC_HIGH-1);
								next_state=TX_CLKH_SLAVE;
							end
					end
				end
	TX_CRC_CLKH_SLAVE: 
						begin
							if (counter_clks_z) begin
								
								counter_bits_dec=1;
								counter_clks_load=1; 
								counter_clks_value=(NOC_LOW-1);
								shiftcrc_slave = 1;	
								next_state=TX_CRC_CLKL_SLAVE;
      end
    end
 TX_CRC_CLKL_SLAVE : begin

						if (counter_clks_z) begin
							if (counter_bits_z) begin
								avs_sdata_set = 1;
								next_state= IDLE;
								
							end 
							else begin
										
																	
								if(crc_dout_slave) begin avs_sdata_set=1; 
								avs_sdata_set_transmit=1;
								end
								else begin avs_sdata_clr = 1 ;
											 avs_sdata_clr_transmit = 1 ;
							end
	
								counter_clks_load=1; counter_clks_value=(NOC_HIGH-1);
								next_state=TX_CRC_CLKH_SLAVE;
							end
						end
					 end				
		
							
 default: begin
	next_state=IDLE;
    end
  endcase
end
/////////////////////////////////////////////////////////////////////////////////////////////////
//CALLING PWM MODULE
////////////////////////////////////////////////////////////////////////////////////////////////
PWMDACSRLN KV1 (.clk(clk),.rstp(rst),.inputpwm(pwm_in),.outputpwm(pwm));

endmodule
