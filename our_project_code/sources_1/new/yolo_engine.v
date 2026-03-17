`timescale 1ns / 1ps

module yolo_engine #(
    parameter AXI_WIDTH_AD = 32, parameter AXI_WIDTH_ID = 4, parameter AXI_WIDTH_DA = 32,
    parameter AXI_WIDTH_DS = AXI_WIDTH_DA/8, parameter OUT_BITS_TRANS = 18
)(
    input clk, input rstn,
    
    input [31:0] i_ctrl_reg0, input [31:0] i_ctrl_reg1, input [31:0] i_ctrl_reg2, input [31:0] i_ctrl_reg3,
    input [31:0] i_ctrl_reg4, input [31:0] i_ctrl_reg5,
    
    // AXI Master Ports
    output M_ARVALID, input M_ARREADY, output [AXI_WIDTH_AD-1:0] M_ARADDR, output [AXI_WIDTH_ID-1:0] M_ARID,
    output [7:0] M_ARLEN, output [2:0] M_ARSIZE, output [1:0] M_ARBURST, output [1:0] M_ARLOCK,
    output [3:0] M_ARCACHE, output [2:0] M_ARPROT, output [3:0] M_ARQOS, output [3:0] M_ARREGION, output [3:0] M_ARUSER,
    input M_RVALID, output M_RREADY, input [AXI_WIDTH_DA-1:0] M_RDATA, input M_RLAST,
    input [AXI_WIDTH_ID-1:0] M_RID, input [3:0] M_RUSER, input [1:0] M_RRESP,
       
    output M_AWVALID, input M_AWREADY, output [AXI_WIDTH_AD-1:0] M_AWADDR, output [AXI_WIDTH_ID-1:0] M_AWID,
    output [7:0] M_AWLEN, output [2:0] M_AWSIZE, output [1:0] M_AWBURST, output [1:0] M_AWLOCK,
    output [3:0] M_AWCACHE, output [2:0] M_AWPROT, output [3:0] M_AWQOS, output [3:0] M_AWREGION, output [3:0] M_AWUSER,
    output M_WVALID, input M_WREADY, output [AXI_WIDTH_DA-1:0] M_WDATA, output [AXI_WIDTH_DS-1:0] M_WSTRB,
    output M_WLAST, output [AXI_WIDTH_ID-1:0] M_WID, output [3:0] M_WUSER,
    input M_BVALID, output M_BREADY, input [1:0] M_BRESP, input [AXI_WIDTH_ID-1:0] M_BID, input M_BUSER,
    
    output reg network_done, output network_done_led   
);

    assign network_done_led = network_done;

    // ----------------------------------------------------------------
    // 1. Parameter Extraction & Main FSM (다차원 타일링 제어)
    // ----------------------------------------------------------------
    wire start_pulse       = i_ctrl_reg0[0]; 
    wire [31:0] ifm_addr   = i_ctrl_reg1;
    wire [31:0] ofm_addr   = i_ctrl_reg2;
    wire [31:0] wgt_addr   = i_ctrl_reg3;
    
    wire [15:0] q_height   = i_ctrl_reg4[31:16];
    wire [15:0] q_width    = i_ctrl_reg4[15:0];
    wire [15:0] q_out_ch   = i_ctrl_reg5[31:16];
    wire [15:0] q_in_ch    = i_ctrl_reg5[15:0];
    
    wire [31:0] q_frame_size = q_height * q_width;

    localparam S_IDLE       = 3'd0;
    localparam S_LOAD_PARAM = 3'd1;
    localparam S_LOAD_IFM   = 3'd2;
    localparam S_COMPUTE    = 3'd3;
    localparam S_WRITE_OFM  = 3'd4;
    localparam S_DONE       = 3'd5;
    
    reg [2:0] top_state;
    reg [15:0] out_ch_idx; // 💡 타일링을 위한 채널 카운터
    
    wire param_load_done;
    wire ifm_load_done;
    wire layer_done;
    reg [15:0] wr_blk_idx;
    wire write_ofm_done = (top_state == S_WRITE_OFM && wr_blk_idx == (q_frame_size / 16));

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            top_state <= S_IDLE;
            out_ch_idx <= 0;
        end else begin
            case (top_state)
                S_IDLE: begin
                    if (start_pulse) begin
                        top_state <= S_LOAD_PARAM;
                        out_ch_idx <= 0;
                    end
                end
                S_LOAD_PARAM: begin
                    if (param_load_done) begin
                        // 💡 [IFM Reuse] 첫 타일에서만 IFM을 불러오고, 나머지는 기존 BRAM 데이터 재활용!
                        if (out_ch_idx == 0) top_state <= S_LOAD_IFM; 
                        else                 top_state <= S_COMPUTE;
                    end
                end
                S_LOAD_IFM: begin
                    if (ifm_load_done) top_state <= S_COMPUTE;
                end
                S_COMPUTE: begin
                    if (layer_done) top_state <= S_WRITE_OFM;
                end
                S_WRITE_OFM: begin
                    if (write_ofm_done) begin
                        if (out_ch_idx + 4 < q_out_ch) begin
                            out_ch_idx <= out_ch_idx + 4; // 다음 4개 채널 타일로 이동
                            top_state <= S_LOAD_PARAM;    // 루프백
                        end else begin
                            top_state <= S_DONE;          // 전체 레이어 완료
                        end
                    end
                end
                S_DONE: top_state <= S_IDLE;
            endcase
        end
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn) network_done <= 0;
        else network_done <= (top_state == S_DONE);
    end

    // ----------------------------------------------------------------
    // 2. DMA Read Multiplexer (시분할 및 주소 오프셋 연동)
    // ----------------------------------------------------------------
    wire dma_rd_start;
    wire [15:0] dma_num_trans = 16;
    wire [15:0] dma_max_blk;
    wire dma_rd_done;

    // 타일 인덱스에 따라 가중치를 읽어올 주소를 192 bytes (48 words)씩 증가시킴
    wire [31:0] wgt_addr_current = wgt_addr + ((out_ch_idx >> 2) * 192);

    assign dma_max_blk = (top_state == S_LOAD_PARAM) ? 16'd3 : (q_frame_size / 16);
    wire [31:0] dma_rd_addr = (top_state == S_LOAD_PARAM) ? wgt_addr_current : ifm_addr;

    reg [2:0] top_state_d;
    always @(posedge clk) top_state_d <= top_state;
    assign dma_rd_start = ((top_state == S_LOAD_PARAM && top_state_d != S_LOAD_PARAM) || 
                           (top_state == S_LOAD_IFM   && top_state_d != S_LOAD_IFM));

    wire ctrl_read; wire [31:0] read_addr;
    axi_dma_ctrl #(.BIT_TRANS(16)) u_dma_ctrl_rd(
        .clk(clk), .rstn(rstn), .i_start(dma_rd_start), 
        .i_base_address_rd(dma_rd_addr), .i_base_address_wr(32'd0),
        .i_num_trans(dma_num_trans), .i_max_req_blk_idx(dma_max_blk),
        .i_read_done(dma_rd_done), .o_ctrl_read(ctrl_read), .o_read_addr(read_addr),
        .i_write_done(1'b1), .i_indata_req_wr(1'b0), .o_ctrl_write(), .o_write_addr(), .o_write_data_cnt(), .o_ctrl_write_done()
    );

    wire [31:0] read_data; wire read_data_vld;
    axi_dma_rd #(.BITS_TRANS(16), .OUT_BITS_TRANS(16), .AXI_WIDTH_USER(1), .AXI_WIDTH_ID(4), .AXI_WIDTH_AD(AXI_WIDTH_AD), .AXI_WIDTH_DA(AXI_WIDTH_DA), .AXI_WIDTH_DS(AXI_WIDTH_DS)) u_dma_read(
        .M_ARVALID(M_ARVALID), .M_ARREADY(M_ARREADY), .M_ARADDR(M_ARADDR), .M_ARID(M_ARID), .M_ARLEN(M_ARLEN), .M_ARSIZE(M_ARSIZE), .M_ARBURST(M_ARBURST), .M_ARLOCK(M_ARLOCK),
        .M_ARCACHE(M_ARCACHE), .M_ARPROT(M_ARPROT), .M_ARQOS(M_ARQOS), .M_ARREGION(M_ARREGION), .M_ARUSER(M_ARUSER), .M_RVALID(M_RVALID), .M_RREADY(M_RREADY), .M_RDATA(M_RDATA), .M_RLAST(M_RLAST),
        .M_RID(M_RID), .M_RUSER(M_RUSER), .M_RRESP(M_RRESP),
        .start_dma(ctrl_read), .num_trans(dma_num_trans), .start_addr(read_addr),
        .data_o(read_data), .data_vld_o(read_data_vld), .data_cnt_o(), .done_o(dma_rd_done), .clk(clk), .rstn(rstn)
    );

    // ----------------------------------------------------------------
    // 3. Tile Parameter Buffer (Register File)
    // ----------------------------------------------------------------
    reg [31:0] param_buf [0:47]; 
    reg [5:0] param_cnt;

    assign param_load_done = (top_state == S_LOAD_PARAM && dma_rd_done);

    always @(posedge clk or negedge rstn) begin
        if (!rstn) param_cnt <= 0;
        else if (top_state != S_LOAD_PARAM) param_cnt <= 0; // 상태 변경 시 리셋
        else if (top_state == S_LOAD_PARAM && read_data_vld) begin
            param_buf[param_cnt] <= read_data;
            param_cnt <= param_cnt + 1;
        end
    end

    // ----------------------------------------------------------------
    // 4. Hardware-Friendly Unpacking (MUX Slice)
    // ----------------------------------------------------------------
    wire [1:0] current_chn; 
    wire [4:0] ch_base = current_chn * 3; 

    wire [127:0] win_ch0 = {56'd0, param_buf[0  + ch_base + 2][7:0], param_buf[0  + ch_base + 1], param_buf[0  + ch_base]};
    wire [127:0] win_ch1 = {56'd0, param_buf[9  + ch_base + 2][7:0], param_buf[9  + ch_base + 1], param_buf[9  + ch_base]};
    wire [127:0] win_ch2 = {56'd0, param_buf[18 + ch_base + 2][7:0], param_buf[18 + ch_base + 1], param_buf[18 + ch_base]};
    wire [127:0] win_ch3 = {56'd0, param_buf[27 + ch_base + 2][7:0], param_buf[27 + ch_base + 1], param_buf[27 + ch_base]};

    wire signed [15:0] bias_ch0 = param_buf[36][15:0];  wire signed [15:0] bias_ch1 = param_buf[36][31:16];
    wire signed [15:0] bias_ch2 = param_buf[37][15:0];  wire signed [15:0] bias_ch3 = param_buf[37][31:16];

    wire [15:0] scale_ch0 = param_buf[38][15:0];  wire [15:0] scale_ch1 = param_buf[38][31:16];
    wire [15:0] scale_ch2 = param_buf[39][15:0];  wire [15:0] scale_ch3 = param_buf[39][31:16];

    function [3:0] get_shift;
        input [15:0] scale;
        begin
            case(scale)
                16'h0001: get_shift = 0; 16'h0002: get_shift = 1; 16'h0004: get_shift = 2;
                16'h0008: get_shift = 3; 16'h0010: get_shift = 4; 16'h0020: get_shift = 5;
                16'h0040: get_shift = 6; 16'h0080: get_shift = 7; 16'h0100: get_shift = 8;
                default:  get_shift = 7;
            endcase
        end
    endfunction

    // ----------------------------------------------------------------
    // 5. IFM Loading & Computation
    // ----------------------------------------------------------------
    assign ifm_load_done = (top_state == S_LOAD_IFM && dma_rd_done);
    reg write_64_en; reg [14:0] addra_64; reg [31:0] read_data_d; reg pack_state; reg [63:0] dia_64;

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            read_data_d <= 0; dia_64 <= 0; write_64_en <= 0; addra_64 <= 0; pack_state <= 0;
        end else if (top_state != S_LOAD_IFM) begin
            addra_64 <= 0; pack_state <= 0; write_64_en <= 0; // 상태 벗어나면 초기화
        end else if (top_state == S_LOAD_IFM && read_data_vld) begin
            if (pack_state == 0) begin read_data_d <= read_data; write_64_en <= 0; pack_state <= 1; end 
            else begin dia_64 <= {read_data, read_data_d}; write_64_en <= 1; pack_state <= 0; end
        end else write_64_en <= 0;
        
        if (write_64_en) addra_64 <= addra_64 + 1;
    end

    wire [63:0] in_ram_dob; wire [14:0] in_ram_addrb;
    dpram_wrapper #(.DEPTH(32768), .AW(15), .DW(64)) u_in_ram (
        .clk(clk), .ena(1'b1), .wea(write_64_en), .addra(addra_64), .dia(dia_64),
        .enb(1'b1), .addrb(in_ram_addrb), .dob(in_ram_dob)
    );

    wire engine_start = (top_state == S_COMPUTE && top_state_d != S_COMPUTE);
    wire [31:0] out_pixel0, out_pixel1; wire out_valid;

    dual_conv_layer u_conv_layer(
        .clk(clk), .rstn(rstn), .start_i(engine_start), .done_o(layer_done),
        .q_width(q_width), .q_height(q_height),
        .init_psum(1'b1), .apply_relu(1'b1), // (CONV00는 In_Ch=3 이므로 타일 내 1회 루프로 끝)
        .in_ram_addrb(in_ram_addrb), .in_ram_dob(in_ram_dob), .current_chn(current_chn),
        .win_ch0(win_ch0), .win_ch1(win_ch1), .win_ch2(win_ch2), .win_ch3(win_ch3),
        .bias_ch0(bias_ch0), .bias_ch1(bias_ch1), .bias_ch2(bias_ch2), .bias_ch3(bias_ch3),
        .shift_ch0(get_shift(scale_ch0)), .shift_ch1(get_shift(scale_ch1)), .shift_ch2(get_shift(scale_ch2)), .shift_ch3(get_shift(scale_ch3)),
        .out_pixel0_32b(out_pixel0), .out_pixel1_32b(out_pixel1), .out_valid(out_valid)
    );

    // ----------------------------------------------------------------
    // 6. Output Buffer & DMA Write (주소 오프셋 적용)
    // ----------------------------------------------------------------
    reg write_p1_pending; reg [15:0] out_ram_addra; reg [31:0] out_ram_dia; reg out_ram_wea;
    always @(posedge clk or negedge rstn) begin
        if(!rstn) begin 
            out_ram_addra <= 0; out_ram_wea <= 0; write_p1_pending <= 0; out_ram_dia <= 0; 
        end else begin
            if (top_state != S_COMPUTE && top_state != S_WRITE_OFM) begin
                out_ram_addra <= 0; out_ram_wea <= 0; write_p1_pending <= 0;
            end else begin
                if (out_valid) begin
                    out_ram_dia <= out_pixel0; out_ram_wea <= 1; write_p1_pending <= 1;
                end else if (write_p1_pending) begin
                    out_ram_dia <= out_pixel1; out_ram_wea <= 1; write_p1_pending <= 0;
                end else begin 
                    out_ram_wea <= 0; 
                end
                if (out_ram_wea) out_ram_addra <= out_ram_addra + 1; 
            end
        end
    end

    reg wr_start_dma; wire write_done; wire indata_req_wr; wire [31:0] write_data;
    reg dma_writing; 

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin 
            wr_blk_idx <= 0; dma_writing <= 0; wr_start_dma <= 0;
        end else if (top_state == S_WRITE_OFM) begin 
            if (wr_blk_idx < (q_frame_size / 16)) begin
                if (!dma_writing) begin 
                    wr_start_dma <= 1; dma_writing <= 1;
                end else begin
                    wr_start_dma <= 0;
                    if (write_done) begin 
                        wr_blk_idx <= wr_blk_idx + 1; dma_writing <= 0;
                    end
                end
            end
        end else begin
            wr_blk_idx <= 0; dma_writing <= 0; wr_start_dma <= 0;
        end
    end

    // 💡 [핵심] 타일 인덱스에 따라 메모리 출력 주소를 격리 (Planar Channel-Interleaved)
    wire [31:0] safe_write_addr = ofm_addr + ((out_ch_idx >> 2) * q_frame_size * 4) + (wr_blk_idx * 16 * 4);
    reg [16:0] my_write_data_cnt; 
    always @(posedge clk or negedge rstn) begin
        if (!rstn) my_write_data_cnt <= 0;
        else if (top_state != S_WRITE_OFM) my_write_data_cnt <= 0;
        else begin
            if (wr_start_dma) my_write_data_cnt <= wr_blk_idx * 16;
            else if (indata_req_wr) my_write_data_cnt <= my_write_data_cnt + 1;
        end
    end

    dpram_wrapper #(.DEPTH(65536), .AW(16), .DW(32)) u_out_ram (
        .clk(clk), .ena(1'b1), .wea(out_ram_wea), .addra(out_ram_addra), .dia(out_ram_dia),
        .enb(1'b1), .addrb(my_write_data_cnt[15:0]), .dob(write_data) 
    );

    axi_dma_wr #(.BITS_TRANS(16), .OUT_BITS_TRANS(16), .AXI_WIDTH_USER(1), .AXI_WIDTH_ID(4), .AXI_WIDTH_AD(AXI_WIDTH_AD), .AXI_WIDTH_DA(AXI_WIDTH_DA), .AXI_WIDTH_DS(AXI_WIDTH_DS)) u_dma_write(
        .M_AWID(M_AWID), .M_AWADDR(M_AWADDR), .M_AWLEN(M_AWLEN), .M_AWSIZE(M_AWSIZE), .M_AWBURST(M_AWBURST), .M_AWLOCK(M_AWLOCK), .M_AWCACHE(M_AWCACHE), .M_AWPROT(M_AWPROT), .M_AWQOS(M_AWQOS), .M_AWREGION(M_AWREGION), .M_AWUSER(M_AWUSER), .M_AWVALID(M_AWVALID), .M_AWREADY(M_AWREADY),
        .M_WID(M_WID), .M_WDATA(M_WDATA), .M_WSTRB(M_WSTRB), .M_WLAST(M_WLAST), .M_WUSER(M_WUSER), .M_WVALID(M_WVALID), .M_WREADY(M_WREADY),
        .M_BID(M_BID), .M_BRESP(M_BRESP), .M_BUSER(M_BUSER), .M_BVALID(M_BVALID), .M_BREADY(M_BREADY),
        .start_dma(wr_start_dma), .num_trans(dma_num_trans), .start_addr(safe_write_addr),
        .indata(write_data), .indata_req_o(indata_req_wr), .done_o(write_done), .clk(clk), .rstn(rstn)
    );

endmodule
