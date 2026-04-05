module RF_main(
    input wire sys_clk,           // 27 MHz System Clock (H11)
    input sys_rst_n, 

    // --- ADC Interface (Bank 0) ---
    input wire [11:0] adc_data,   // ADC Parallel Data
    input wire adc_otr,           // ADC Over-Range
    output wire adc_clk_out,      // Clock to ADC

    // --- ADF4351 SPI & Control (Bank 1) ---
    output wire adf_clk,          // SPI Clock (P6)
    output wire adf_data,         // SPI MOSI (T6)
    output wire adf_le,           // Load Enable (R8)
    output wire adf_ce,           // Chip Enable (T7)
    input wire adf_mux,           // Mux Out / Ready (T8)
    input wire adf_ld,            // Lock Detect (P8)

    // --- System Control (Bank 1) ---
    output wire ref_10m_out,        // 10 MHz Reference (P9)
    output wire relay_filter,       // 1 = 5MHz bandwidth, 0 = 1MHz bandwidth
    output reg relay_gain = 1'b0,   // 1 = x33 voltage gain, 0 = x100 voltage gain

    // --- LO Output ---
    output wire LO_OUT,            // Output (T11)
    
    // --- UART ---
    output TX,
    
    //--- Audio Controller ---
    output wire BCK,
    output wire DOUT,
    output wire WS, 
    output wire PA_EN,

    //--- Control Inputs ---
    input S1,   // T3 Toggle UART Mode (Fast Radar vs Scope)
    input S2,   // T2 Increase setting
    input S3,   // D7 Decrease setting
    input S4    // C7 Change analog bandwidth 
);

// Default Control States
assign adf_clk = 1'b0;
assign adf_data = 1'b0;
assign adf_le = 1'b0;
assign adf_ce = 1'b0;
assign ref_10m_out = 1'b0; 

assign BCK = 1;
assign DOUT = 1;
assign WS = 1;
assign PA_EN = 0;

// --- 1. LO PLL Instantiation ---
wire lo_lock;
Gowin_rPLL LO_364_5(
    .clkout(LO_OUT),   
    .clkin(sys_clk),
    .lock(lo_lock)    
);

// --- 2. Sampling PLL Instantiation ---
wire sampling_lock;
wire clk_20mhz;
Gowin_rPLL_sampling SAMPLING_20MHZ(
    .clkout(clk_20mhz),
    .clkin(sys_clk),          
    .lock(sampling_lock)   
);
assign adc_clk_out = clk_20mhz;

// ==============================================================================
// 1. UART CONTROLLER
// ==============================================================================

wire uart_write_done;
reg [7:0] uart_data = 8'b0;
reg uart_wr_en = 1'b0;

UART_Controller #(
    .BAUD_RATE(921_600),
    .CLOCK_FREQ(20_250_000) 
) debug_uart (
    .sys_clk(clk_20mhz),    
    .sys_rst_n(sys_rst_n),
    .write_enable(uart_wr_en), 
    .data_to_send(uart_data),  
    .RX(1'b1), 
    .TX(TX),
    .write_done(uart_write_done),
    .read_done(),
    .data_readed()
);

// ==============================================================================
// 2. DEBOUNCING & S1 TOGGLE LOGIC
// ==============================================================================

reg [19:0] debounce_counter = 0;
reg S1_stable = 1, S2_stable = 1, S3_stable = 1, S4_stable = 1;
reg S1_prev = 1, S2_prev = 1, S3_prev = 1, S4_prev = 1;

always @(posedge clk_20mhz or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        debounce_counter <= 0;
        S1_stable <= 1; S2_stable <= 1; S3_stable <= 1; S4_stable <= 1;
    end else begin
        if (S1 == S1_stable && S2 == S2_stable && S3 == S3_stable && S4 == S4_stable) begin
            debounce_counter <= 0;
        end else begin
            debounce_counter <= debounce_counter + 1;
            if (debounce_counter >= 20'd1_000_000) begin 
                S1_stable <= S1; S2_stable <= S2; S3_stable <= S3; S4_stable <= S4;
                debounce_counter <= 0;
            end
        end
    end
end

wire S1_pressed = (!S1_stable && S1_prev); 
wire S2_pressed = (!S2_stable && S2_prev); 
wire S3_pressed = (!S3_stable && S3_prev); 
wire S4_pressed = (!S4_stable && S4_prev); 

always @(posedge clk_20mhz or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        S1_prev <= 1; S2_prev <= 1; S3_prev <= 1; S4_prev <= 1;
    end else begin
        S1_prev <= S1_stable; S2_prev <= S2_stable; S3_prev <= S3_stable; S4_prev <= S4_stable;
    end
end

// --- S1: FAST RADAR / SCOPE MODE TOGGLE ---
reg fast_radar_mode = 1'b1; // Default to Fast Radar mode
always @(posedge clk_20mhz or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        fast_radar_mode <= 1'b1;
    end else if (S1_pressed) begin
        fast_radar_mode <= ~fast_radar_mode;
    end
end

reg [25:0] hold_duration = 0; 
reg [20:0] repeat_timer = 0;  
reg auto_step_pulse = 0;

reg [31:0] current_step = 32'd10_000;
reg [11:0] margin_step  = 12'd1; 

always @(posedge clk_20mhz or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        hold_duration <= 0;
        repeat_timer <= 0;
        auto_step_pulse <= 0;
        current_step <= 32'd10_000;
        margin_step <= 12'd1;
    end else if (!S2_stable || !S3_stable) begin
        if (hold_duration < 26'd40_000_000) hold_duration <= hold_duration + 1;

        if (hold_duration > 26'd30_000_000) begin
            current_step <= 32'd100_000;       
            margin_step <= 12'd50; 
        end else if (hold_duration > 26'd15_000_000) begin
            current_step <= 32'd50_000;        
            margin_step <= 12'd10; 
        end else begin
            current_step <= 32'd10_000;        
            margin_step <= 12'd1;  
        end

        if (hold_duration > 26'd10_000_000) begin
            if (repeat_timer >= 21'd2_000_000) begin 
                repeat_timer <= 0;
                auto_step_pulse <= 1; 
            end else begin
                repeat_timer <= repeat_timer + 1;
                auto_step_pulse <= 0;
            end
        end else begin
            repeat_timer <= 0;
            auto_step_pulse <= 0;
        end
    end else begin
        hold_duration <= 0;
        repeat_timer <= 0;
        auto_step_pulse <= 0;
        current_step <= 32'd10_000;
        margin_step <= 12'd1;
    end
end

wire S2_action = S2_pressed || (!S2_stable && auto_step_pulse);
wire S3_action = S3_pressed || (!S3_stable && auto_step_pulse);

// ==============================================================================
// 3. SYSTEM STATE REGISTERS
// ==============================================================================

reg mod_type = 1'b0;                     
reg [31:0] center_freq = 32'd3_500_000;     
reg analog_bandwidth_extended = 1'b1;    
reg [11:0] threshold_margin = 12'd1300; // UPDATED INITIAL MARGIN        

assign relay_filter = analog_bandwidth_extended;

always @(posedge clk_20mhz or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        threshold_margin <= 12'd1300; 
    end else begin
        if (S2_action) begin
            if (threshold_margin <= (12'd4095 - margin_step)) begin
                threshold_margin <= threshold_margin + margin_step;
            end
        end

        if (S3_action) begin
            if (threshold_margin >= margin_step) begin
                threshold_margin <= threshold_margin - margin_step;
            end
        end
    end
end

// ==============================================================================
// 4. SAMPLING
// ==============================================================================

reg [11:0] adc_data_internal = 0;

always @(negedge clk_20mhz or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        adc_data_internal <= 12'b0;
        relay_gain <= 1'b0; 
    end else if (sampling_lock && lo_lock) begin
        adc_data_internal <= adc_data; 
        relay_gain <= 1'b0;
    end else begin
        adc_data_internal <= 12'b0;
    end
end

// ==============================================================================
// 5. BANDPASS FILTER & CLEAN LINEAR GAIN
// ==============================================================================
wire signed [12:0] adc_signed = $signed({1'b0, adc_data_internal}) - 13'sd2048;

wire signed [12:0] filtered_if;
FIR_BPF custom_bpf (.clk(clk_20mhz), .rst_n(sys_rst_n), .din(adc_signed), .dout(filtered_if));

wire [12:0] abs_full = (filtered_if[12]) ? (~filtered_if + 1'b1) : filtered_if;
wire [11:0] raw_abs = (abs_full > 13'd4095) ? 12'd4095 : abs_full[11:0];

wire [11:0] boosted_abs = (raw_abs > 12'd2047) ? 12'd4095 : {raw_abs[10:0], 1'b0};

// ==============================================================================
// 6. CASCADED MATCHED FILTER (Perfect 0.5us Pulse Shaper)
// ==============================================================================

// STAGE 1: 8-Tap Moving Average
reg [11:0] abs_sr [0:7];

always @(posedge clk_20mhz or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        abs_sr[0]<=0; abs_sr[1]<=0; abs_sr[2]<=0; abs_sr[3]<=0;
        abs_sr[4]<=0; abs_sr[5]<=0; abs_sr[6]<=0; abs_sr[7]<=0;
    end else begin
        abs_sr[0] <= boosted_abs; 
        abs_sr[1] <= abs_sr[0];
        abs_sr[2] <= abs_sr[1];
        abs_sr[3] <= abs_sr[2];
        abs_sr[4] <= abs_sr[3];
        abs_sr[5] <= abs_sr[4];
        abs_sr[6] <= abs_sr[5];
        abs_sr[7] <= abs_sr[6];
    end
end

wire [14:0] sum_8 = abs_sr[0] + abs_sr[1] + abs_sr[2] + abs_sr[3] +
                    abs_sr[4] + abs_sr[5] + abs_sr[6] + abs_sr[7];

wire [11:0] stage1_env = sum_8[14:3]; 

// STAGE 2: 4-Tap Moving Average
reg [11:0] env_sr [0:3];

always @(posedge clk_20mhz or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        env_sr[0]<=0; env_sr[1]<=0; env_sr[2]<=0; env_sr[3]<=0;
    end else begin
        env_sr[0] <= stage1_env;
        env_sr[1] <= env_sr[0];
        env_sr[2] <= env_sr[1];
        env_sr[3] <= env_sr[2];
    end
end

wire [13:0] sum_4 = env_sr[0] + env_sr[1] + env_sr[2] + env_sr[3];
wire [11:0] base_env = sum_4[13:2];

// --- EXTREME NON-LINEAR SENSITIVITY BOOST (8x Zoom) ---
wire [14:0] extreme_boost;

assign extreme_boost = 
    (base_env < 12'd256)  ? {base_env, 3'b000} :                     // 8x gain
    (base_env < 12'd512)  ? {base_env, 2'b00}  + 15'd1024 :          // 4x gain
    (base_env < 12'd1024) ? {base_env, 1'b0}   + 15'd2048 :          // 2x gain
                            {3'b000, base_env} + 15'd3072;           // 1x gain

wire [11:0] adsb_envelope = (extreme_boost > 15'd4095) ? 12'd4095 : extreme_boost[11:0];

// ==============================================================================
// 7. SLOW DYNAMIC THRESHOLD (404us Time Constant)
// ==============================================================================
reg [24:0] noise_accumulator = 25'd0; 
wire [11:0] noise_floor = noise_accumulator[24:13]; 

wire [12:0] raw_thresh = noise_floor + threshold_margin;
wire [11:0] dynamic_threshold = (raw_thresh > 13'd4095) ? 12'd4095 : raw_thresh[11:0];

reg adsb_bit = 1'b0;

always @(posedge clk_20mhz or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        noise_accumulator <= 25'd0;
        adsb_bit <= 1'b0;
    end else begin
        if (adsb_envelope > dynamic_threshold) begin
            adsb_bit <= 1'b1; 
        end else begin
            adsb_bit <= 1'b0; 
            noise_accumulator <= noise_accumulator - noise_floor + adsb_envelope;
        end
    end
end

// ==============================================================================
// 8. STRICT PREAMBLE & ERROR-TOLERANT PPM DECODER
// ==============================================================================

reg [9:0] bit_history = 10'b0;
wire [3:0] pulse_score = bit_history[0] + bit_history[1] + bit_history[2] + bit_history[3] + 
                         bit_history[4] + bit_history[5] + bit_history[6] + bit_history[7] + 
                         bit_history[8] + bit_history[9];

localparam WAIT_SYNC      = 3'd0;
localparam CHECK_PREAMBLE = 3'd1;
localparam DECODE_DATA    = 3'd2;
localparam DONE           = 3'd3;

reg [2:0] rx_state = WAIT_SYNC;
reg [11:0] timer_cnt = 12'd0;    
reg [6:0] bit_idx = 7'd0;        

reg [3:0] first_half_score = 4'd0;
reg [3:0] error_cnt = 4'd0; 

reg [111:0] adsb_payload = 112'd0;
reg packet_ready = 1'b0; 

wire [4:0] cycles_per_bit = (bit_idx[1:0] == 2'b11) ? 5'd20 : 5'd19;
wire [4:0] mid_point      = 5'd9;

always @(posedge clk_20mhz or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        bit_history <= 10'b0;
        rx_state <= WAIT_SYNC;
        timer_cnt <= 12'd0;
        packet_ready <= 1'b0;
    end else begin
        bit_history <= {bit_history[8:0], adsb_bit};
        packet_ready <= 1'b0; 

        case (rx_state)
            WAIT_SYNC: begin
                if (pulse_score >= 4'd8) begin 
                    rx_state <= CHECK_PREAMBLE;
                    timer_cnt <= 12'd10; 
                    error_cnt <= 4'd0; 
                end
            end

            CHECK_PREAMBLE: begin
                timer_cnt <= timer_cnt + 1;
                
                if (timer_cnt == 12'd20  && pulse_score > 4'd3) rx_state <= WAIT_SYNC; 
                if (timer_cnt == 12'd30  && pulse_score < 4'd7) rx_state <= WAIT_SYNC; 
                if (timer_cnt == 12'd50  && pulse_score > 4'd3) rx_state <= WAIT_SYNC; 
                if (timer_cnt == 12'd71  && pulse_score > 4'd3) rx_state <= WAIT_SYNC; 
                if (timer_cnt == 12'd81  && pulse_score < 4'd7) rx_state <= WAIT_SYNC; 
                if (timer_cnt == 12'd91  && pulse_score > 4'd3) rx_state <= WAIT_SYNC; 
                if (timer_cnt == 12'd101 && pulse_score < 4'd7) rx_state <= WAIT_SYNC; 
                if (timer_cnt == 12'd121 && pulse_score > 4'd3) rx_state <= WAIT_SYNC; 
                if (timer_cnt == 12'd141 && pulse_score > 4'd3) rx_state <= WAIT_SYNC; 

                if (timer_cnt == 12'd161) begin 
                    rx_state <= DECODE_DATA;
                    timer_cnt <= 12'd0;
                    bit_idx <= 7'd0;
                    error_cnt <= 4'd0; 
                end
            end

            DECODE_DATA: begin
                timer_cnt <= timer_cnt + 1;

                if (timer_cnt == mid_point) begin
                    first_half_score <= pulse_score;
                end

                if (timer_cnt == cycles_per_bit) begin
                    timer_cnt <= 12'd0; 
                    
                    if (first_half_score >= pulse_score + 1) begin
                        adsb_payload <= {adsb_payload[110:0], 1'b1}; 
                    end else if (pulse_score >= first_half_score + 1) begin
                        adsb_payload <= {adsb_payload[110:0], 1'b0}; 
                    end else begin
                        error_cnt <= error_cnt + 1;
                        if (first_half_score >= pulse_score) begin
                            adsb_payload <= {adsb_payload[110:0], 1'b1}; 
                        end else begin
                            adsb_payload <= {adsb_payload[110:0], 1'b0}; 
                        end
                    end

                    if (error_cnt >= 4'd10) begin
                        rx_state <= WAIT_SYNC;
                    end else if (bit_idx == 7'd111) begin
                        rx_state <= DONE;
                    end else begin
                        bit_idx <= bit_idx + 1;
                    end
                end
            end

            DONE: begin
                packet_ready <= 1'b1; 
                rx_state <= WAIT_SYNC;
            end
            
            default: rx_state <= WAIT_SYNC;
        endcase
    end
end

// ==============================================================================
// 9. DUAL-MODE UART PACKETIZER (Fast Radar 20B OR Scope 16KB)
// ==============================================================================

reg [111:0] tx_buffer = 112'd0;
reg [3:0] byte_cnt = 4'd0;

localparam ST_DELAY      = 4'd0;
localparam ST_SYNC1      = 4'd1;
localparam ST_SYNC2      = 4'd2;
localparam ST_FLAGS      = 4'd3;
localparam ST_PAYLOAD    = 4'd4;
localparam ST_BIN_FETCH  = 4'd5;
localparam ST_BIN_WAIT   = 4'd6;
localparam ST_BIN_B3     = 4'd7;
localparam ST_BIN_B2     = 4'd8;
localparam ST_BIN_B1     = 4'd9;
localparam ST_BIN_B0     = 4'd10;
localparam ST_TX_CLEAR   = 4'd11;

reg [3:0] tx_state = ST_DELAY;
reg [3:0] next_tx_state = ST_DELAY;

reg [11:0] bin_cnt = 12'd0; 

reg [7:0] current_payload_byte;
always @(*) begin
    case(byte_cnt)
        4'd0:  current_payload_byte = tx_buffer[111:104];
        4'd1:  current_payload_byte = tx_buffer[103:96];
        4'd2:  current_payload_byte = tx_buffer[95:88];
        4'd3:  current_payload_byte = tx_buffer[87:80];
        4'd4:  current_payload_byte = tx_buffer[79:72];
        4'd5:  current_payload_byte = tx_buffer[71:64];
        4'd6:  current_payload_byte = tx_buffer[63:56];
        4'd7:  current_payload_byte = tx_buffer[55:48];
        4'd8:  current_payload_byte = tx_buffer[47:40];
        4'd9:  current_payload_byte = tx_buffer[39:32];
        4'd10: current_payload_byte = tx_buffer[31:24];
        4'd11: current_payload_byte = tx_buffer[23:16];
        4'd12: current_payload_byte = tx_buffer[15:8];
        4'd13: current_payload_byte = tx_buffer[7:0];
        default: current_payload_byte = 8'h00;
    endcase
end

wire [31:0] scope_word = {7'b0, adsb_bit, dynamic_threshold[11:0], adsb_envelope[11:0]};

reg [11:0] scope_write_idx = 12'b0; 
reg is_capturing = 1'b1; 

reg [19:0] scope_trigger_timer = 20'd0;
reg start_tx = 1'b0;

always @(posedge clk_20mhz or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        scope_trigger_timer <= 20'd0;
        start_tx <= 1'b0;
        is_capturing <= 1'b1;
        tx_buffer <= 112'd0;
        scope_write_idx <= 12'b0;
    end else begin
        
        if (is_capturing) begin
            scope_write_idx <= scope_write_idx + 1;
        end

        if (tx_state == ST_DELAY) begin
            is_capturing <= 1'b1; 
            
            if (packet_ready) begin
                is_capturing <= 1'b0; 
                tx_buffer <= adsb_payload; 
                start_tx <= 1'b1; 
                scope_trigger_timer <= 20'd0; 
                
            end else if (scope_trigger_timer >= 20'd500_000) begin
                is_capturing <= 1'b0; 
                tx_buffer <= 112'd0;  
                start_tx <= 1'b1; 
                scope_trigger_timer <= 20'd0; 
                
            end else begin
                scope_trigger_timer <= scope_trigger_timer + 1;
                start_tx <= 1'b0;
            end
        end else begin
            start_tx <= 1'b0;
            is_capturing <= 1'b0; 
        end
    end
end

always @(posedge clk_20mhz or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        tx_state <= ST_DELAY;
        uart_wr_en <= 1'b0;
        bin_cnt <= 12'd0;
    end else begin
        case (tx_state)
            ST_DELAY: begin
                uart_wr_en <= 1'b0;
                if (start_tx) tx_state <= ST_SYNC1;
            end

            ST_SYNC1: begin
                uart_data <= 8'hAA; 
                uart_wr_en <= 1'b1;
                if (uart_write_done) begin
                    uart_wr_en <= 1'b0; next_tx_state <= ST_SYNC2; tx_state <= ST_TX_CLEAR;   
                end
            end

            ST_SYNC2: begin
                uart_data <= 8'h55; 
                uart_wr_en <= 1'b1;
                if (uart_write_done) begin
                    uart_wr_en <= 1'b0; next_tx_state <= ST_FLAGS; byte_cnt <= 4'd0; tx_state <= ST_TX_CLEAR;
                end
            end

            ST_FLAGS: begin
                if (byte_cnt == 0) uart_data <= threshold_margin[11:8];
                if (byte_cnt == 1) uart_data <= threshold_margin[7:0];
                if (byte_cnt == 2) uart_data <= {4'b0, noise_floor[11:8]};
                if (byte_cnt == 3) uart_data <= noise_floor[7:0];
                
                uart_wr_en <= 1'b1;
                if (uart_write_done) begin
                    uart_wr_en <= 1'b0;
                    if (byte_cnt == 3) begin
                        byte_cnt <= 4'd0; next_tx_state <= ST_PAYLOAD;
                    end else begin
                        byte_cnt <= byte_cnt + 1; next_tx_state <= ST_FLAGS;
                    end
                    tx_state <= ST_TX_CLEAR;
                end
            end

            ST_PAYLOAD: begin
                uart_data <= current_payload_byte;
                uart_wr_en <= 1'b1;
                if (uart_write_done) begin
                    uart_wr_en <= 1'b0;
                    if (byte_cnt == 4'd13) begin
                        // --- DUAL MODE BRANCH ---
                        if (fast_radar_mode) begin
                            next_tx_state <= ST_DELAY; // Fast Radar: End packet here!
                        end else begin
                            bin_cnt <= 12'd0; 
                            next_tx_state <= ST_BIN_FETCH; // Scope Mode: Dump VRAM!
                        end
                    end else begin
                        byte_cnt <= byte_cnt + 1; next_tx_state <= ST_PAYLOAD;
                    end
                    tx_state <= ST_TX_CLEAR;
                end
            end

            ST_BIN_FETCH: begin
                vram_rd_addr_reg <= bin_cnt;
                tx_state <= ST_BIN_WAIT;
            end
            
            ST_BIN_WAIT: tx_state <= ST_BIN_B3;

            ST_BIN_B3: begin
                uart_data <= vram_read_data[31:24]; 
                uart_wr_en <= 1'b1;
                if (uart_write_done) begin
                    uart_wr_en <= 1'b0; next_tx_state <= ST_BIN_B2; tx_state <= ST_TX_CLEAR;
                end
            end

            ST_BIN_B2: begin
                uart_data <= vram_read_data[23:16]; 
                uart_wr_en <= 1'b1;
                if (uart_write_done) begin
                    uart_wr_en <= 1'b0; next_tx_state <= ST_BIN_B1; tx_state <= ST_TX_CLEAR;
                end
            end

            ST_BIN_B1: begin
                uart_data <= vram_read_data[15:8]; 
                uart_wr_en <= 1'b1;
                if (uart_write_done) begin
                    uart_wr_en <= 1'b0; next_tx_state <= ST_BIN_B0; tx_state <= ST_TX_CLEAR;
                end
            end

            ST_BIN_B0: begin
                uart_data <= vram_read_data[7:0];  
                uart_wr_en <= 1'b1;
                if (uart_write_done) begin
                    uart_wr_en <= 1'b0;
                    if (bin_cnt == 12'd4095) begin
                        next_tx_state <= ST_DELAY; 
                    end else begin
                        bin_cnt <= bin_cnt + 1; next_tx_state <= ST_BIN_FETCH;
                    end
                    tx_state <= ST_TX_CLEAR;
                end
            end

            ST_TX_CLEAR: begin
                uart_wr_en <= 1'b0; 
                if (!uart_write_done) tx_state <= next_tx_state; 
            end
            
            default: tx_state <= ST_DELAY;
        endcase
    end
end

wire [11:0] vram_read_addr; 
wire [31:0] vram_read_data; 
reg [11:0] vram_rd_addr_reg = 12'd0;
assign vram_read_addr = vram_rd_addr_reg;

Gowin_SDPB_vram_2 custom_vram (
    .clka(clk_20mhz),            
    .cea(is_capturing),         
    .reseta(~sys_rst_n),        
    .ada(scope_write_idx),      
    .din(scope_word),            
    
    .clkb(clk_20mhz),            
    .ceb(1'b1),                 
    .resetb(~sys_rst_n),        
    .oce(1'b1),                 
    .adb(vram_read_addr),        
    .dout(vram_read_data)        
);

endmodule