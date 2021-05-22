from PyQt5.QtGui import QIcon

import rj_msgs
import PyQt5.QtWidgets as widget
from PyQt5.QtWidgets import QApplication, QLabel, QWidget


class QuickRestartsPanel(QWidget):
    def __init__(self):
        super(QuickRestartsPanel, self).__init__()
        layout = widget.QHBoxLayout()
        self.fast_halt = widget.QPushButton()
        self.fast_halt.setIcon(QIcon("../resource/halt.svg"))
        self.fast_halt.setFixedWidth(30)
        self.fast_halt.setFixedHeight(30)
        layout.addWidget(self.fast_halt)


def main():
    app = QApplication([])
    restarts = QuickRestartsPanel()
    restarts.show()
    app.exec_()


if __name__ == '__main__':
    main()
