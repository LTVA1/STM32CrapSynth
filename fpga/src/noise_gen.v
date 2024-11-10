/*module noise_gen (
    input sys_clk,
    input sys_rst_n,
    
    input spi_clock,
    input spi_data,
    input spi_cs,
    output reg noise_signal
);

reg [22:0] lfsr;
reg [16:0] freq_div;
reg [39:0] spi_receive_reg;

reg r_XNOR;
reg r_LFSR;

always @(posedge sys_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        lfsr <= 23'd1;
        freq_div <= 17'd0;
        spi_receive_reg <= 40'd0;
        noise_signal <= 1;
    end
    else begin
        if(lfsr === 23'd0) begin
            lfsr <= 23'd1;
        end
        
        r_XNOR <= lfsr[23-1] ^~ lfsr[18-1];
        lfsr <= {lfsr[22:1], r_XNOR};
        r_LFSR <= lfsr[0];
        
        //noise_signal <= r_LFSR;
        noise_signal <= 1;
    end
end

endmodule*/

module noise_gen (
    input sys_clk,
    input sys_rst_n,

    input spi_clock,
    input spi_data,
    input spi_cs,
    output reg noise_signal
);

reg [16:0] counter;

reg [22:0] lfsr;
reg [16:0] freq_div;
reg [39:0] spi_receive_reg;

reg r_XNOR;
reg r_LFSR;

reg block = 0;

always @(posedge sys_clk or negedge sys_rst_n) begin
    //if(!block) begin
        if (!sys_rst_n) begin
            counter <= 17'd0;

            lfsr <= 23'd111;
            freq_div <= 17'd13000;
            spi_receive_reg <= 40'd0;
        end
        else if (counter < freq_div)
            counter <= counter + 1;
        else begin
            counter <= 17'd0;

            if(lfsr === 23'd0) begin
                lfsr <= 23'd111;
            end
            
            r_XNOR <= lfsr[22] ^ lfsr[17];
            lfsr <= {lfsr[21:0], r_XNOR};
            r_LFSR <= lfsr[0];
            
            noise_signal <= r_LFSR;
        end
    //end
end

/*always @(negedge spi_clock) begin
    spi_receive_reg <= {spi_receive_reg[38:0], spi_data};
end

always @(negedge spi_cs) begin
    spi_receive_reg <= 40'd0;
    block = 1;
end

always @(posedge spi_cs) begin
    lfsr <= spi_receive_reg[22:0];
    freq_div <= spi_receive_reg[39:23];
    block = 0;
end*/

endmodule
