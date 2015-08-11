from pyOCD.board import MbedBoard
 
board = MbedBoard.chooseBoard()
 
target = board.target
flash = board.flash


def showRegs(t):

    print "pc\t\t0x%08X" % t.readCoreRegister("pc")
    print "lr\t\t0x%08X" % t.readCoreRegister("lr")
    print "sp\t\t0x%08X" % t.readCoreRegister("sp")
    print "faultmask\t0x%08X" % t.readCoreRegister("faultmask")
    print "control\t\t0x%08X" % t.readCoreRegister("control")

    for i in range(0, 15):
        buf = "r%d" % int(i)
        print buf, "\t\t0x%08X" % t.readCoreRegister(buf)

    for i in range(0, 14):
        buf = "s%d" % int(i)
        print buf, "\t\t0x%08X" % t.readCoreRegister(buf)

    print "\n"


print board.getInfo()
print board.getUniqueID()
print flash.getFlashInfo()
 
target.resetStopOnReset()
print "PC after reset: 0x%08X\n" % target.readCoreRegister("pc")

showRegs(target)

target.step()
showRegs(target)
 
board.uninit()
