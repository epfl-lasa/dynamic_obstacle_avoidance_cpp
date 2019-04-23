#!/usr/bin/python3

# import Tkindr as tkinter
# import tkinter

# window = tkinter.Tk()

# window.title("Static Obstacle Avoidance")
# window.geometry("300x300")
#window.wm_icon-bitmap("") add icon

# window.mainloop()


import sys

from PySide2.QtWidgets import QApplication, QMessageBox
from PySide2 import QtGui
# from PySide2 import QWidget
# from PySide2 import QWidgets


# Create the application object
# if app is None:
    # app = QApplication(sys.argv)


# Create a simple dialog box

# msg_box = QMessageBox()
# msg_box.setText("Hello World2!")
# msg_box.show()

# sys.exit(msg_box.exec_())

class Sample(QtGui.QWidget):

    def __init__(self):
        super(Sample, self).__init__()

        self.initUI()

    def initUI(self):
        self.setGeometry(500, 500, 800, 600)
        self.setWindowTitle('Sample')

        btn1 = QtGui.QPushButton("Button 1")

        hbox = QtGui.QHBoxLayout()
        hbox.addStretch(1)
        hbox.addWidget(btn1)

        self.setLayout(hbox)
        self.show()

def main():
    app = QtGui.QApplication(sys.argv)
    ex = Sample()
    sys.exit(app.exec_())

print("start it")

if __name__ == '__main__':
    main()

print("end")
