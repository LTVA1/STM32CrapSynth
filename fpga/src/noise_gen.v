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

// SPI part

// sync spi_clock to the FPGA clock using a 3-bit shift register
reg [2:0] SCKr;  always @(posedge sys_clk) SCKr <= {SCKr[1:0], spi_clock};
wire SCK_risingedge = (SCKr[2:1]==2'b01);  // now we can detect spi_clock rising edges
wire SCK_fallingedge = (SCKr[2:1]==2'b10);  // and falling edges

// same thing for spi_cs
reg [2:0] SSELr;  always @(posedge sys_clk) SSELr <= {SSELr[1:0], spi_cs};
wire SSEL_active = ~SSELr[1];  // spi_cs is active low
wire SSEL_startmessage = (SSELr[2:1]==2'b10);  // message starts at falling edge
wire SSEL_endmessage = (SSELr[2:1]==2'b01);  // message stops at rising edge

// and for spi_data
reg [1:0] MOSIr;  always @(posedge sys_clk) MOSIr <= {MOSIr[0], spi_data};
wire MOSI_data = MOSIr[1];

// we handle SPI in 40-bit format
reg [5:0] bitcnt;

reg data_received;  // high when a byte has been received

always @(posedge sys_clk)
begin
    if(~SSEL_active)
        bitcnt <= 6'd0;
    else
    if(SCK_risingedge)
    begin
        bitcnt <= bitcnt + 6'd1;

        // implement a shift-left register (since we receive the data MSB first)
        spi_receive_reg <= {spi_receive_reg[38:0], MOSI_data};
    end
end

always @(posedge sys_clk) data_received <= SSEL_active && SCK_risingedge && (bitcnt == 6'd39);

always @(posedge data_received)
begin
    if(SSEL_active)
    begin
        lfsr <= spi_receive_reg[22:0];
        freq_div <= spi_receive_reg[39:23];
    end
end

always @(posedge sys_clk or negedge sys_rst_n or posedge SSEL_active or posedge data_received)
begin
    if(data_received)
    begin
        if(SSEL_active)
        begin
            lfsr <= spi_receive_reg[22:0];
            freq_div <= spi_receive_reg[39:23];
        end
    end
    else
    begin
        if(SSEL_active) begin
            
        end
        else begin
            if (!sys_rst_n) begin
                counter <= 17'd0;

                lfsr <= 23'd111;
                freq_div <= 17'd13000;
                //spi_receive_reg <= 40'd0;
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
        end
    end
end

endmodule
