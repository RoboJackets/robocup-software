from myhdl import *
from qencoder import qencoder


def tb_stimulus():
    nIt = 4
    W = 10

    clk, a, b = inputs = [Signal(bool(0)) for i in range(3)]
    count = [Signal(intbv(0)[W:]) for i in range(nIt)]

    #inputs = concat(a, b)

    inst = qencoder(clk, a, b, count)

    interval = 10
    clk = delay(interval)

    @always(clk)
    def clkgen():
        clk.next = not clk

    @instance
    def stimulus():
        data = [Signal(intbv(i)[1:]) for i in [0, 1, 3, 2]]

        print "Starting"

        for i in range(nIt):
            a.next = data[i % len(data)][1]
            b.next = data[i % len(data)][2]
            print "Count: %s" % str(count[i])
            yield delay(interval)


    return clkgen, stimulus, inst



def test_bench():
    sim = Simulation(tb_stimulus())
    sim.run(1000)



if __name__ == '__main__':
    test_bench()

