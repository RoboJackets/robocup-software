`ifndef _CLK_DIVIDE_
`define _CLK_DIVIDE_

// ClkDivide module
module ClkDivide ( CLK_IN, EN, CLK_OUT );

parameter WIDTH = ( 3 );

input CLK_IN, EN;
output CLK_OUT;

reg [WIDTH-1:0] edge_cnt = 0;

// Posedge counter
always @(posedge CLK_IN)
begin
    if ( EN != 1 ) begin
        edge_cnt <= 0;
    end else if ( edge_cnt == (1 << WIDTH) - 1) begin
        edge_cnt <= 0;
    end else begin
        edge_cnt <= edge_cnt + 1;
    end
end

assign CLK_OUT = edge_cnt[WIDTH-1];

endmodule // ClkDivide

`endif
