module FIR_BPF (
    input wire clk,
    input wire rst_n,
    input wire signed [12:0] din,     // Signed ADC data input
    output reg signed [12:0] dout     // Filtered output
);

    // 15-Tap Shift Register to hold the history of incoming samples
    reg signed [12:0] shift_reg [0:14];
    
    
    wire signed [15:0] coeff [0:14];
    assign coeff[0]  =  16'sd16;
    assign coeff[1]  = -16'sd7;
    assign coeff[2]  =  16'sd82;
    assign coeff[3]  = -16'sd77;
    assign coeff[4]  = -16'sd229;
    assign coeff[5]  = -16'sd154;
    assign coeff[6]  =  16'sd130;
    assign coeff[7]  =  16'sd297;
    assign coeff[8]  =  16'sd130;
    assign coeff[9]  = -16'sd154;
    assign coeff[10] = -16'sd229;
    assign coeff[11] = -16'sd77;
    assign coeff[12] =  16'sd82;
    assign coeff[13] = -16'sd7;
    assign coeff[14] =  16'sd16;

    // Multiplier Results Pipeline
    reg signed [28:0] mult_res [0:14];
    
    // Accumulator
    reg signed [31:0] sum;
    
    integer i;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dout <= 13'sd0;
            sum <= 32'sd0;
            for (i = 0; i < 15; i = i + 1) begin
                shift_reg[i] <= 13'sd0;
                mult_res[i] <= 29'sd0;
            end
        end else begin
            // 1. Shift new data into the pipeline
            shift_reg[0] <= din;
            for (i = 1; i < 15; i = i + 1) begin
                shift_reg[i] <= shift_reg[i-1];
            end
            
            // 2. Multiply (These map directly to FPGA DSP blocks)
            for (i = 0; i < 15; i = i + 1) begin
                mult_res[i] <= shift_reg[i] * coeff[i];
            end
            
            // 3. Accumulate (Sum all multiplier results)
            sum <= mult_res[0] + mult_res[1] + mult_res[2] + mult_res[3] + 
                   mult_res[4] + mult_res[5] + mult_res[6] + mult_res[7] + 
                   mult_res[8] + mult_res[9] + mult_res[10] + mult_res[11] + 
                   mult_res[12] + mult_res[13] + mult_res[14];
                   
            // 4. Normalize (Divide by the coefficient scaling factor = 1024, shift right by 10)
            dout <= sum[22:10]; 
        end
    end
endmodule