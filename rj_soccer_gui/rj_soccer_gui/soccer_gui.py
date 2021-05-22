import rj_msgs
from PyQt5.QtWidgets import QApplication, QLabel


def main():
    app = QApplication([])
    label = QLabel('Hello world!')
    label.show()
    app.exec_()


if __name__ == '__main__':
    main()
