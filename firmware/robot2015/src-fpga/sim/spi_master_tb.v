`timescale 1ns/10ps

module SPI_Master_tb;

localparam SPI_MASTER_DATA_WIDTH = 16;
localparam SPI_MASTER_CPOL = 0;
localparam SPI_MASTER_CPHA = 1;

reg clk;
reg spi_master_en;
reg spi_master_miso;
reg spi_master_start;
reg [SPI_MASTER_DATA_WIDTH-1:0] spi_master_data_to_send;

wire spi_master_sck;
wire spi_master_mosi;
wire spi_master_busy;
wire spi_master_valid;
wire [SPI_MASTER_DATA_WIDTH-1:0] spi_master_data_received;
wire drv_ncs;

assign drv_ncs = ~spi_master_sel;

SPI_Master #(
  .DATA_BIT_WIDTH ( SPI_MASTER_DATA_WIDTH )
  ) spi_master_module (
  .clk        ( clk                 ),
  .EN         ( spi_master_en       ),
  .SCK        ( spi_master_sck      ),
  .MOSI       ( spi_master_mosi     ),
  .MISO       ( spi_master_miso     ),
  .SEL        ( spi_master_sel      ),
  .START      ( spi_master_start    ),
  .BUSY       ( spi_master_busy     ),
  .VALID      ( spi_master_valid    ),
  .DATA_OUT   ( spi_master_data_received ),
  .DATA_IN    ( spi_master_data_to_send )
);

// module main clock
initial begin
  forever begin
    #0.5 clk = !clk;
  end
end

// main simulation entry
initial begin
  clk = 0;
  spi_master_en = 0;
  spi_master_start = 0;
  spi_master_miso = 1;

  $dumpfile("SPI_Master_tb-results.vcd");
  $dumpvars(0, SPI_Master_tb);

  #20 spi_master_en = 1;
  #5 spi('hd10d);
  #5;

  forever begin
    @(negedge spi_master_busy) begin
      #10 spi('hacac);
    end
  end

end

task spi;
  input [SPI_MASTER_DATA_WIDTH-1:0] data;
  begin
    spi_master_data_to_send = data;
    spi_master_start = 1;
    #1 spi_master_start = 0;
  end
endtask

endmodule
