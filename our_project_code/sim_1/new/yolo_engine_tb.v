`timescale 1ns / 1ns

module yolo_engine_tb;

parameter IFM_WIDTH         = 256;
parameter IFM_HEIGHT        = 256;
parameter IFM_FILE          = "C:/yolohw/sim/inout_data_sw/log_feamap/CONV00_input_32b.hex"; 

parameter CONV_INPUT_IMG00  = "C:/yolohw/sim/inout_data_hw/CONV00_input_ch00.bmp"; 
parameter CONV_INPUT_IMG01  = "C:/yolohw/sim/inout_data_hw/CONV00_input_ch01.bmp"; 
parameter CONV_INPUT_IMG02  = "C:/yolohw/sim/inout_data_hw/CONV00_input_ch02.bmp"; 
parameter CONV_INPUT_IMG03  = "C:/yolohw/sim/inout_data_hw/CONV00_input_ch03.bmp"; 

parameter CONV_OUTPUT_IMG00 = "C:/yolohw/sim/inout_data_hw/CONV00_output_ch00.bmp"; 
parameter CONV_OUTPUT_IMG01 = "C:/yolohw/sim/inout_data_hw/CONV00_output_ch01.bmp"; 
parameter CONV_OUTPUT_IMG02 = "C:/yolohw/sim/inout_data_hw/CONV00_output_ch02.bmp"; 
parameter CONV_OUTPUT_IMG03 = "C:/yolohw/sim/inout_data_hw/CONV00_output_ch03.bmp"; 

localparam MEM_ADDRW = 22;
localparam MEM_DW    = 16; 
localparam A = 32; localparam D = 32; localparam I = 4; localparam L = 8; localparam M = D/8;

parameter CLK_PERIOD = 10;
reg clk; reg rstn;
initial begin clk = 1'b1; forever #(CLK_PERIOD/2) clk = ~clk; end

wire [I-1:0] M_AWID; wire [A-1:0] M_AWADDR; wire [L-1:0] M_AWLEN; wire [2:0] M_AWSIZE;     
wire [1:0] M_AWBURST; wire [1:0] M_AWLOCK; wire [3:0] M_AWCACHE; wire [2:0] M_AWPROT;     
wire M_AWVALID; wire M_AWREADY; wire [I-1:0] M_WID; wire [D-1:0] M_WDATA;      
wire [M-1:0] M_WSTRB; wire M_WLAST; wire M_WVALID; wire M_WREADY;     
wire [I-1:0] M_BID; wire [1:0] M_BRESP; wire M_BVALID; wire M_BREADY;     
wire [I-1:0] M_ARID; wire [A-1:0] M_ARADDR; wire [L-1:0] M_ARLEN; wire [2:0] M_ARSIZE;     
wire [1:0] M_ARBURST; wire [1:0] M_ARLOCK; wire [3:0] M_ARCACHE; wire [2:0] M_ARPROT;     
wire M_ARVALID; wire M_ARREADY; wire [I-1:0] M_RID; wire [D-1:0] M_RDATA;      
wire [1:0] M_RRESP; wire M_RLAST; wire M_RVALID; wire M_RREADY;     

// 💡 AXI BUSER, WUSER 추가 (VRFC 에러 방지용)
wire [3:0] M_ARUSER;
wire [3:0] M_AWUSER;
wire [3:0] M_WUSER;
wire M_BUSER;

wire [MEM_ADDRW-1:0] mem_addr; wire mem_we; wire [MEM_DW-1:0] mem_di; wire [MEM_DW-1:0] mem_do;

// 💡 AXI SRAM IF 인스턴스 (원본 컨트롤러 유지)
axi_sram_if #(.MEM_ADDRW(MEM_ADDRW), .MEM_DW(MEM_DW), .A(A), .I(I), .L(L), .D(D), .M(M))
u_axi_ext_mem_if_input(
   .ACLK(clk), .ARESETn(rstn),
   .AWID(M_AWID), .AWADDR(M_AWADDR), .AWLEN(M_AWLEN), .AWSIZE(M_AWSIZE), .AWBURST(M_AWBURST), .AWLOCK(M_AWLOCK), .AWCACHE(M_AWCACHE), .AWPROT(M_AWPROT), .AWVALID(M_AWVALID), .AWREADY(M_AWREADY),
   .WID(M_WID), .WDATA(M_WDATA), .WSTRB(M_WSTRB), .WLAST(M_WLAST), .WVALID(M_WVALID), .WREADY(M_WREADY),
   .BID(M_BID), .BRESP(M_BRESP), .BVALID(M_BVALID), .BREADY(M_BREADY),
   .ARID(M_ARID), .ARADDR(M_ARADDR), .ARLEN(M_ARLEN), .ARSIZE(M_ARSIZE), .ARBURST(M_ARBURST), .ARLOCK(M_ARLOCK), .ARCACHE(M_ARCACHE), .ARPROT(M_ARPROT), .ARVALID(M_ARVALID), .ARREADY(M_ARREADY),
   .RID(M_RID), .RDATA(M_RDATA), .RRESP(M_RRESP), .RLAST(M_RLAST), .RVALID(M_RVALID), .RREADY(M_RREADY),
   .mem_addr(mem_addr), .mem_we(mem_we), .mem_di(mem_di), .mem_do(mem_do)
);

// 💡 SRAM 인스턴스
sram #(.FILE_NAME(IFM_FILE), .SIZE(2**MEM_ADDRW), .WL_ADDR(MEM_ADDRW), .WL_DATA(MEM_DW))
u_ext_mem_input (.clk(clk), .rst(rstn), .addr(mem_addr), .wdata(mem_di), .rdata(mem_do), .ena(mem_we));

wire i_WVALID = M_WVALID; wire [31:0] i_WDATA = M_WDATA; 

// 💡 [수정] 6개의 레지스터 설정용 변수
reg [31:0] i_0 = 0; reg [31:0] i_1 = 0; reg [31:0] i_2 = 0; 
reg [31:0] i_3 = 0; reg [31:0] i_4 = 0; reg [31:0] i_5 = 0;
wire network_done; wire network_done_led;

yolo_engine #(.AXI_WIDTH_AD(A), .AXI_WIDTH_ID(4), .AXI_WIDTH_DA(D), .AXI_WIDTH_DS(M))
u_yolo_engine (
    .clk(clk), .rstn(rstn),
    .i_ctrl_reg0(i_0), .i_ctrl_reg1(i_1), .i_ctrl_reg2(i_2), 
    .i_ctrl_reg3(i_3), .i_ctrl_reg4(i_4), .i_ctrl_reg5(i_5),
    
    .M_ARVALID(M_ARVALID), .M_ARREADY(M_ARREADY), .M_ARADDR(M_ARADDR), .M_ARID(M_ARID), .M_ARLEN(M_ARLEN), .M_ARSIZE(M_ARSIZE), .M_ARBURST(M_ARBURST), .M_ARLOCK(M_ARLOCK), .M_ARCACHE(M_ARCACHE), .M_ARPROT(M_ARPROT), .M_ARQOS(), .M_ARREGION(), .M_ARUSER(M_ARUSER),
    .M_RVALID(M_RVALID), .M_RREADY(M_RREADY), .M_RDATA(M_RDATA), .M_RLAST(M_RLAST), .M_RID(M_RID), .M_RUSER(4'd0), .M_RRESP(M_RRESP),
    
    .M_AWVALID(M_AWVALID), .M_AWREADY(M_AWREADY), .M_AWADDR(M_AWADDR), .M_AWID(M_AWID), .M_AWLEN(M_AWLEN), .M_AWSIZE(M_AWSIZE), .M_AWBURST(M_AWBURST), .M_AWLOCK(M_AWLOCK), .M_AWCACHE(M_AWCACHE), .M_AWPROT(M_AWPROT), .M_AWQOS(), .M_AWREGION(), .M_AWUSER(M_AWUSER),
    .M_WVALID(M_WVALID), .M_WREADY(M_WREADY), .M_WDATA(M_WDATA), .M_WSTRB(M_WSTRB), .M_WLAST(M_WLAST), .M_WID(M_WID), .M_WUSER(M_WUSER),
    .M_BVALID(M_BVALID), .M_BREADY(M_BREADY), .M_BRESP(M_BRESP), .M_BID(M_BID), .M_BUSER(1'b0),
    
    .network_done(network_done), .network_done_led(network_done_led)
);

// 💡 TB에서 내부 신호를 직접 참조(Hierarchical Reference)하여 Input BMP 출력 복구
wire read_data_vld = u_yolo_engine.read_data_vld; 
wire [31:0] read_data = u_yolo_engine.read_data;

// 가중치를 읽을 때는 이미지 라이터에 섞여 들어가지 않도록 차단
wire is_ifm_load = (u_yolo_engine.top_state == 3'd2); // S_LOAD_IFM 상태일 때만
wire input_bmp_vld = read_data_vld & is_ifm_load;

initial begin
   rstn = 1'b0; i_0 = 0; 
   // 💡 SW-driven 파라미터 동적 세팅 (CONV00 기준)
   i_1 = 32'd2048;            // IFM_ADDR (DDR 메모리 상의 이미지 시작 주소)
   i_2 = (4*256*256)*4;       // OFM_ADDR (출력 저장 주소)
   i_3 = 32'd0;               // WGT_ADDR (파라미터 주소 - 현재는 파일에서 읽지만 세팅)
   i_4 = {16'd256, 16'd256};  // Height=256, Width=256
   i_5 = {16'd16,  16'd3};    // Out_Ch=16, In_Ch=3
   
   #(4*CLK_PERIOD) rstn = 1'b1; 
   #(100*CLK_PERIOD) @(posedge clk) i_0 = 32'd1; // Start 펄스 ON
   #(100*CLK_PERIOD) @(posedge clk) i_0 = 32'd0; // Start 펄스 OFF
   
   while(!network_done) begin #(1000*CLK_PERIOD) @(posedge clk); end 
   
   $display("==== 다차원 타일링 연산 & AXI 전송 완벽 종료! ====");
   #(500*CLK_PERIOD) @(posedge clk) $stop;
end

bmp_image_writer #(.OUTFILE(CONV_INPUT_IMG00),.WIDTH(IFM_WIDTH),.HEIGHT(IFM_HEIGHT)) u_bmp_in_00(.clk(clk), .rstn(rstn), .din(read_data[7:0]), .vld(input_bmp_vld), .frame_done());
bmp_image_writer #(.OUTFILE(CONV_INPUT_IMG01),.WIDTH(IFM_WIDTH),.HEIGHT(IFM_HEIGHT)) u_bmp_in_01(.clk(clk), .rstn(rstn), .din(read_data[15:8]), .vld(input_bmp_vld), .frame_done());
bmp_image_writer #(.OUTFILE(CONV_INPUT_IMG02),.WIDTH(IFM_WIDTH),.HEIGHT(IFM_HEIGHT)) u_bmp_in_02(.clk(clk), .rstn(rstn), .din(read_data[23:16]), .vld(input_bmp_vld), .frame_done());
bmp_image_writer #(.OUTFILE(CONV_INPUT_IMG03),.WIDTH(IFM_WIDTH),.HEIGHT(IFM_HEIGHT)) u_bmp_in_03(.clk(clk), .rstn(rstn), .din(read_data[31:24]), .vld(input_bmp_vld), .frame_done());

bmp_image_writer #(.OUTFILE(CONV_OUTPUT_IMG00),.WIDTH(IFM_WIDTH),.HEIGHT(IFM_HEIGHT)) u_bmp_out_00(.clk(clk), .rstn(rstn), .din(i_WDATA[7:0]), .vld(i_WVALID), .frame_done());
bmp_image_writer #(.OUTFILE(CONV_OUTPUT_IMG01),.WIDTH(IFM_WIDTH),.HEIGHT(IFM_HEIGHT)) u_bmp_out_01(.clk(clk), .rstn(rstn), .din(i_WDATA[15:8]), .vld(i_WVALID), .frame_done());
bmp_image_writer #(.OUTFILE(CONV_OUTPUT_IMG02),.WIDTH(IFM_WIDTH),.HEIGHT(IFM_HEIGHT)) u_bmp_out_02(.clk(clk), .rstn(rstn), .din(i_WDATA[23:16]), .vld(i_WVALID), .frame_done());
bmp_image_writer #(.OUTFILE(CONV_OUTPUT_IMG03),.WIDTH(IFM_WIDTH),.HEIGHT(IFM_HEIGHT)) u_bmp_out_03(.clk(clk), .rstn(rstn), .din(i_WDATA[31:24]), .vld(i_WVALID), .frame_done());

endmodule
