import PyQt5
import numpy as np
np.asmatrix
from PyQt5 import QtCore, QtGui, QtWidgets, uic
import sys

class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__() # Call the inherited classes __init__ method
        uic.loadUi('prova_2.ui', self) # Load the .ui file
        self.show() # Show the GUI

        
        #----------------------------------------------------------------
        #                        connection definition
        #----------------------------------------------------------------
        
        #la funzione cerca il bottone di nome "bottone_1"
        self.bottone_1 = self.findChild(QtWidgets.QPushButton, 'bottone_1')
        self.bottone_1.clicked.connect(self.printButtonPressed)
        self.bottone_2 = self.findChild(QtWidgets.QPushButton,"bottone_2")
        self.bottone_2.clicked.connect(lambda: self.clicked("button 2 clicked"))
        self.label = self.findChild(QtWidgets.QLabel,"label")

    #azione da intraprendere nelò caso il bottone 1 sia premuto
    def printButtonPressed(self):
        # This is executed when the button is pressed
        print('printButtonPressed')

    def clicked(self, text):
        #self.label = QtWidgets.QLabel da accendere per avere suggerimenti vscode
        self.label.setText(text)
        self.label.adjustSize()


app = QtWidgets.QApplication(sys.argv) # Create an instance of QtWidgets.QApplication
window = Ui() # Create an instance of our class
app.exec_() # Start the applicatio