// Always include

`ifdef ICARUS_VERILOG
   `define  LOG2( x )   $clog2( x )
   
`else
function integer log2;
    input integer value;
        begin
            value = value - 1;
            for ( log2 = 0; value > 0; log2 = log2 + 1 )
                value = value >> 1;
        end
endfunction
    
    `define  LOG2( x )   log2( x )
`endif 
