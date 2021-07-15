
#!/home/loris/.pyenv/shims/python
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow
import sys
import numpy
numpy.random
def window():
    app = QApplication(sys.argv) # used to setup the application wrt the operating system
    win =QMainWindow() 
    xpos = 0 # up and rigth position
    ypos = 0
    width = 800
    heigth = 800
    win.setGeometry(xpos, ypos, width, heigth) 
    win.setWindowTitle("mia prova")
    label = QtWidgets.QLabel(win)
    label.setText("my first label")
    label.move(50,50)

    win.show()
    sys.exit(app.exec_())

window()
