
module Single_Port_RAM_Sync
#(
    parameter ADDR_WIDTH = 12,
    DATA_WIDTH = 8
)
(
    input                         clk,
    input                         we,
    input   [ ADDR_WIDTH - 1:0 ]  addr,
    input   [ DATA_WIDTH - 1:0 ]  din,
    output  [ DATA_WIDTH - 1:0 ]  dout
);

  // Declarations
  reg [ DATA_WIDTH - 1:0 ] ram [ 2**ADDR_WIDTH - 1:0 ];
  reg [ ADDR_WIDTH - 1:0 ] addr_reg;

  always @(posedge clk)
  begin : RAM_SYNC
      // Write
      if ( we ) begin
        ram[addr] <= din;
      end

      addr_reg <= addr;
  end

  // Read
  assign dout = ram[addr_reg];

endmodule
