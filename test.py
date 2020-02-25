import sys
from PyQt5.QtWidgets import (QApplication,QWidget)
from PyQt5.Qt import Qt

class mainWindow(QWidget):
    def __init__(self):
        super().__init__()
    def keyPressEvent(self,event):
        print("asf")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    demo = mainWindow()
    demo.show()
    sys.exit(app.exec_())
