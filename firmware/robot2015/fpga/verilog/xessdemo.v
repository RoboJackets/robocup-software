module xessdemo(
  // Outputs
  s,
  // Inputs
  clka,
  );
  
  output [7:0] s;
  input clka;

  reg [25:0] divider;
  wire [7:0] s = divider[25:18];
  always @(posedge clka)
  begin
    divider <= divider + 1;
  end
endmodule // top

