from myhdl import *
#from myhdl.conversion import analyzes

def qencoder(clk, a, b, count):  

    a_d, b_d, valid, upcount = [intbv(0) for i in range(4)]

    #@always_comb
    #def updown():
    valid = a ^ a_d ^ b ^ b_d
    upcount = a ^ b_d
    a_d.next = a
    b_d.next = b


    @always(clk.posedge)
    def logic():
        if valid:
            if upcount:
                count[:].next += 1
            else:
                count[:].next -= 1

    return logic


def convert():
    clk, a, b = [Signal(bool(0)) for i in range(3)]
    count = Signal(intbv(0)[15:])

    toVerilog(qencoder, clk, a, b, count)
    toVHDL(qencoder, clk, a, b, count)
