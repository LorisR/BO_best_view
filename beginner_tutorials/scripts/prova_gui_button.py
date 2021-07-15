
#!/home/loris/.pyenv/shims/python
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow
import sys
import numpy
numpy.random

class Mywindow(QMainWindow):
    def __init__(self):
        super(Mywindow,self).__init__()
        xpos = 0 # up and rigth position
        ypos = 0
        width = 800
        heigth = 800
        self.setGeometry(xpos, ypos, width, heigth) 
        self.setWindowTitle("mia prova")
    
        self.initUI() 
    def initUI(self):
        self.label = QtWidgets.QLabel(self)
        self.label.setText("my first label")
        self.label.move(50,50)
        self.b1 = QtWidgets.QPushButton(self)
        self.b1.setText("button")  
        self.b1.move(20,20)
        self.b1.clicked.connect(self.clicked)
    def clicked(self):
        self.label.setText("you have pressed the button")
        self.update()
    
    def update(self):
        self.label.adjustSize()



def button1():
    print("button 1 clicked")

def window():
    app = QApplication(sys.argv) # used to setup the application wrt the operating system
    win = Mywindow() 



    win.show()
    sys.exit(app.exec_())

window()
