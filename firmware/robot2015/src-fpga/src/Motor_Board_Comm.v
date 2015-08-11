   
module Motor_Board_Comm (
    output [4:0] drv_ncs,
    output [1:0] adc_ncs,

    input spi_slave_sck, spi_slave_mosi, spi_slave_ncs
    inout spi_slave_miso,

    input spi_master_miso
    output spi_master_sck, spi_master_mosi
);


// We will need 5 SPI master instances for the 5 drivers, and 2 SPI instances for the ADCs
genvar i;
generate for (i = 0; i < 7; i = i + 1) begin: SPI_Master_Inst

    spi_master
      #(
      .DATA_WIDTH(DATA_WIDTH),
      .CLK_DIVIDER_WIDTH(CLK_DIVIDER_WIDTH)
      )
      spi_master_i
      (
      .clk(clk),
      .resetb(resetb),
      .CPOL(1'b0),
      .CPHA(1'b0),
      .clk_divider(clk_divider),
      .go(go),
      .datai(datai),
      .datao(master_datao0),
      .busy(busy[0]),
      .done(done[0]),
      .sclk(sclk[0]),
      .csb(csb[0]),
      .din(din[0]),
      .dout(dout[0])
    );

  end 
endgenerate

endmodule
