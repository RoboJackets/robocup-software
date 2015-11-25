import time
import datetime as dt
from pyOCD.board import MbedBoard
from pyOCD.gdbserver import GDBServer

port = 3333

if __name__ == "__main__":
    b = MbedBoard.chooseBoard()
    gdb = GDBServer(b,port)
    print("starting gdb server running on localhost:{}".format(port))
    while True:
        time.sleep(2)
        print("tick: {}".format(dt.datetime.now()))
	

