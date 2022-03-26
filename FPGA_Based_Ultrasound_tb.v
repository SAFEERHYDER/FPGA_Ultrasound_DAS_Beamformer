`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.03.2020 12:00:26
// Design Name: 
// Module Name: FPGA_Based_Ultrasound_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module FPGA_Based_Ultrasound_tb();
parameter 
          
        N  = 24100;  // system input RAM depth
        reg  s00_axis_aclk;
		reg  s00_axis_aresetn;
		wire  s00_axis_tready;
		reg [31 : 0] s00_axis_tdata;
		reg  s00_axis_tlast;
		reg  s00_axis_tvalid;
        
		// Ports of Axi Master Bus Interface M00_AXIS
		reg  m00_axis_aclk;
		reg  m00_axis_aresetn;
		wire  m00_axis_tvalid;
		wire [31 : 0] m00_axis_tdata;
		wire  m00_axis_tlast;
		reg  m00_axis_tready;
		
        reg [11:0] ram_xin [0:24100];
  
    //Generate a clock with 10 ns clock period.
        initial s00_axis_aclk = 0;
        always 
        #5 s00_axis_aclk =~s00_axis_aclk;
   // Reset signal
        initial begin
        #00 s00_axis_aresetn = 0; 
        #10 s00_axis_aresetn = 1; 
        end
    
         //Generate a clock with 10 ns clock period.
        initial m00_axis_aclk = 0;
        always 
        #5 m00_axis_aclk =~m00_axis_aclk;
   // Reset signal
        initial begin
        #00 m00_axis_aresetn = 0; 
        #10 m00_axis_aresetn = 1; 
        end
  // Reset signal
        initial begin
        #00 s00_axis_tvalid = 0; 
        #10 s00_axis_tvalid = 1; 
       end
       // Reset signal
        initial begin
        #00 s00_axis_tlast = 0; 
        #10 s00_axis_tlast = 64790; 
       end
       
     
         // Reset signal
        initial begin
        #00 m00_axis_tready = 0; 
        #10 m00_axis_tready = 1; 
       end
    
    // Generate RAM Addresses for system input
        integer addr1;
        initial addr1=0;
        always
        if (addr1 == N)
        addr1 = 0;
        else
        #10 addr1 = addr1+1;
        

        
    

          
 //    Read data file and assign data to memory
        initial begin
      $readmemh("D:/ME_Thesis_Work/ME Thesis/coding/FPGA_Implementation_of_DAS/RF_DATA_24100.txt", ram_xin);
       end
         
    // Assign memory data to the Input of the filter
        always @(posedge s00_axis_aclk)
        
        begin
            s00_axis_tdata <= ram_xin[addr1];
        end     
    
  FPGA_Based_Ultrasound_Beamforming inst0
//  ram1 inst0
  (     .s00_axis_aclk(s00_axis_aclk),
        .s00_axis_aresetn(s00_axis_aresetn),
        .m00_axis_aclk(m00_axis_aclk),
        .m00_axis_aresetn(m00_axis_aresetn),
        .s00_axis_tdata(s00_axis_tdata),
        .s00_axis_tready(s00_axis_tready),
        .s00_axis_tlast(s00_axis_tlast),
        .s00_axis_tvalid(s00_axis_tvalid),
        .m00_axis_tdata(m00_axis_tdata),
        .m00_axis_tvalid(m00_axis_tvalid),
        .m00_axis_tlast(m00_axis_tlast),
        .m00_axis_tready(m00_axis_tready)
  );  
endmodule
