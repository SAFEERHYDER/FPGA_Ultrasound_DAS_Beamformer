`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Sukkur IBA Universty
// Engineer: Supervision Of Dr: Safeer Hyder Laghari
// 
// Create Date: 14.12.2020 13:12:33
// Design Name: FPGA_Based_Beamformed_Ultrasound_Signal_Processing_Platfrom
// Module Name: FPGA_Based_Ultrasound_Beamforming
// Project Name: M.E THESIS 2020 
// Target Devices: PYNQ_Z2
// Description:  
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//% Delay and Sum Beamformer 
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%
//%       
//%       Performes DAS beamforming for the RF data of a plane wave steered
//%       with (theta_d) degree.
//%       RF_data is the received RF data.
//%       theta_d is the steering angle of the plane wave in degree, can be
//%       a positive or a negative value.
//%       z_start is the imaging depth starting point in meter.
//%       z_stop is the imaging depth ending point in meter.
//%       image_width is the required width of the produced image.
//%       lateral_step is the lateral step size between each lines of the
//%       beamformed image. (pitch/2 is a good choice)
//%       N_elements is the total number of elements in the transducer. 
//%       pitch is the distance between the centres of two adjacent elements.
//%       c is the sound speed im m/s.
//%       fs is the sampling frequency.
//%
//%
//%   Method:
//%       In DAS beamforming, for each point of (x,z), the RF data from each receiving element is
//%       delayed and the selected samples from these elements are summed. 
//%       for each point (x,z), the delays applied to the RF data are
//%       calculated by the time required for the signal to travel from the
//%       transmiter to the field point and back to the receiving element as follows:
//%       t = t_transmit + t_receive
//%       t_transmit= [ z.cos(theta_d)+x.sin(theta_d)+0.5.N_elements.pitch.sin(|theta_d|) ]/c
//%       t_receive(for element j)= sqrt(z^2 + (xj-x)^2)/c
//%
//%
//%   Additions:
//%       Receive Apodization - Hann window (twice the transducer) 
//%       Element Directivity Check - discard contributions from elements that are >45 degrees to the beamforming point
//%                                 - corrected for non-zero starting depth
//%
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//%
//%   Example:
//%       RF_data = UARP_TEMP_DATA;
//%       theta_d = 0;
//%       N_elements = 128;
//%       pitch = 0.3048e-3;
//%       image_width = N_elements*pitch;
//%       lateral_step = pitch/2;
//%       z_start = 5e-3;
//%       z_stop = 60e-3;
//%       c = 1540;
//%       fs = 80e6;
//%      
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//////////////////////////////////////////////////////////////////////////////////


module FPGA_Based_Ultrasound_Beamforming#
	(
		// Users to add parameters here

		// User parameters ends
		// Do not modify the parameters beyond this line



		// Parameters of Axi Slave Bus Interface S00_AXIS
		parameter integer C_S00_AXIS_TDATA_WIDTH	= 32,
		parameter integer C_S01_AXIS_TDATA_WIDTH    = 24,

		// Parameters of Axi Master Bus Interface M00_AXIS
		parameter integer C_M00_AXIS_TDATA_WIDTH	= 32,
		parameter integer C_M00_AXIS_START_COUNT	= 32
	)
	(
		// Users to add ports here

		// User ports ends
		// Do not modify the ports beyond this line
		
       // Ports of Axi Slave Bus Interface S00_AXIS
		input wire  s00_axis_aclk,
		input wire  s00_axis_aresetn,
		output wire  s00_axis_tready,
		input wire [C_S00_AXIS_TDATA_WIDTH-1 : 0] s00_axis_tdata,
		input wire [(C_S00_AXIS_TDATA_WIDTH/8)-1 : 0] s00_axis_tstrb,
		input wire  s00_axis_tlast,
		input wire  s00_axis_tvalid,
		
		input wire  m00_axis_aclk,
		input wire  m00_axis_aresetn,
		output wire  m00_axis_tvalid,
		output wire [C_M00_AXIS_TDATA_WIDTH-1: 0] m00_axis_tdata,
		output wire [(C_M00_AXIS_TDATA_WIDTH/8)-1 : 0] m00_axis_tstrb,
		output wire  m00_axis_tlast,
		input wire  m00_axis_tready
		);
		
wire [7:0] N_elements;
wire [6:0]theta_d;
wire [7:0] pitch;
wire [10:0] c;
wire [26:0] fs;
wire [2:0]  lambda_fs;
wire [31:0] M;

assign theta_d = 7'd0; /// 0
assign N_elements = 8'b10000000; // 128
assign pitch = 8'b1010000; // 80
assign c = 11'b11000000100; // 1540
assign fs = 27'b100110001001011010000000000; // 800000000
assign lambda_fs = 3'b101; // 8

// Control Signals
  reg signed [31:0] m00_axis_tdata_reg;
  reg m00_axis_tvalid_reg;
  reg m00_axis_tlast_reg;
  reg s00_axis_tready_reg;
  reg signed [31:0] i,j,k,m,count,addr2,addr3;  // int32#
  
//  reg signed [11:0] RF_data [0:771199]; // Define Input Data
  reg signed [11:0] RF_data [0:24099]; // Define Input Data
  reg signed [13:0] X [0:65535];  // Define image x-axis (lateral) (-19.5E-3: 1.5240e-04: 19.5E-3)
  reg signed [14:0] Z [0:65535];  // Define image z-axis (axial)   (0e-3: 9.6250e-06*16: 39.4e-3)
  reg signed [27:0] Zsq[0:65535];  // Define Z.^2 
  reg signed [11:0] RF_data1 [0:6024];
  reg signed [31:0] Beamformed_Data[0:65535];  // Define Output Data
  reg signed [13:0] RF_Adresess;  // ufix9_En1
  reg signed [28:0] RF_Adresess1 [0:65535];  // Define Adressess
  reg signed [14:0] RF_Adresess2 [0:65535];  // Define Adressess
  reg signed [15:0] RF_Adresess3[0:65535];  //  Define Adressess
  reg signed [31:0] sqrt_reg;
  
 // Suare_Root Function 
reg [15:0] sqr;
  
  //Verilog function to find square root of a 32 bit number.
  //The output is 16 bit.
  function [15:0] sqrt;
      input [31:0] num;  //declare input
      //intermediate signals.
      reg [31:0] a;
      reg [15:0] q;
      reg [17:0] left,right,r;    
      integer i;
  begin
      //initialize all the variables.
      a = num;
      q = 0;
      i = 0;
      left = 0;   //input to adder/sub
      right = 0;  //input to adder/sub
      r = 0;  //remainder
      //run the calculations for 16 iterations.
      for(i=0;i<16;i=i+1) begin 
          right = {q,r[17],1'b1};
          left = {r[15:0],a[31:30]};
          a = {a[29:0],2'b00};    //left shift by 2 bits.
          if (r[17] == 1) //add if r is negative
              r = left + right;
          else    //subtract if r is positive
              r = left - right;
          q = {q[14:0],!r[17]};       
      end
      sqrt = q;   //final assignment of output.
  end
  endfunction 
// Suare_Root of Function
  
    initial begin
      $readmemh("D:/ME_Thesis_Work/ME Thesis/coding/FPGA_Implementation_of_DAS/X_65536.txt", X);
      $readmemh("D:/ME_Thesis_Work/ME Thesis/coding/FPGA_Implementation_of_DAS/Z1.txt", Z);
      $readmemh("D:/ME_Thesis_Work/ME Thesis/coding/FPGA_Implementation_of_DAS/Zsq_65536.txt", Zsq);
     end
           
    assign m00_axis_tvalid = m00_axis_tvalid_reg;
    assign m00_axis_tlast = m00_axis_tlast_reg;
    assign m00_axis_tdata = m00_axis_tdata_reg;
    assign s00_axis_tready = s00_axis_tready_reg; 
    assign M = 24097; //12047 TWO COLUMN DATA
    
   
 // Block To Generate Ready Signal             
 always @(posedge s00_axis_aclk)begin
        if (s00_axis_aresetn==0)begin
      s00_axis_tready_reg <= 0;
        end else begin
      s00_axis_tready_reg <= m00_axis_tready;
        end
       end 
       
 // Block To Generate For Input Data Transfer  
    always @(posedge m00_axis_aclk)begin
            if (m00_axis_aresetn==0) begin
              m00_axis_tlast_reg <= 0; m00_axis_tdata_reg<=0;  count <= 0; addr2<= 0; addr3<= 0; m00_axis_tvalid_reg<= 0;
            end else begin
                 m00_axis_tlast_reg <= s00_axis_tlast;  m00_axis_tvalid_reg <= s00_axis_tvalid;
          if(s00_axis_tready && s00_axis_tvalid )
          count  <= count+1;
          if(count<=M)
                RF_data[count]<= s00_axis_tdata;
            else begin 

    if(m00_axis_tready && m00_axis_tvalid )
        if (addr2 >= 65536 ) 
        addr2 <= 0;  
        else 
        addr2 <= addr2+1;

        Beamformed_Data[addr2] = 0;   // Allocate memeory for the Beamformed Data
        
 // Calculate the image data for each receiving element   
 // d1 = Z Because of using Zero Angle  % Distance from the transmitter to the points
   for(i = 32'sd0; i <= 32'sd3; i = i + 32'sd1) 
      begin
            for(j = 32'sd0; j <= 32'sd6024; j = j + 32'sd1) // Column wise RF_Data Transfer
                begin
                    RF_data1[j] = RF_data[j+(i*32'sd6025)]; // First 0 TO 6024 Then 6025 to 12049 end else
                end
       RF_Adresess               =   (((i+1)- N_elements /2) * pitch)+6;
       RF_Adresess1[addr2]       =   ((X[addr2] - RF_Adresess)* (X[addr2] - RF_Adresess))+ Zsq[addr2]; 
       sqrt_reg                  =   RF_Adresess1[addr2];
       RF_Adresess2[addr2]       =   sqrt(sqrt_reg);      
       RF_Adresess3[addr2]       =   ((RF_Adresess2[addr2] + Z[addr2])/lambda_fs);
       Beamformed_Data[addr2]    =   Beamformed_Data[addr2] + RF_data1[RF_Adresess3[addr2]];
       m00_axis_tdata_reg        =   Beamformed_Data[addr2];
             
    end
  end
 end
end

endmodule