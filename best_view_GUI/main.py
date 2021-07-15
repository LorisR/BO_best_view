# %% import delle librerie
#------------------------------------------------------
# ---------------------- main.py -----------------------
# ------------------------------------------------------
from PyQt5.QtWidgets import*
from PyQt5.uic import loadUi
from PyQt5.QtWidgets import QFileDialog, QMessageBox
from PyQt5 import QtCore, QtGui, QtWidgets
from matplotlib.backends.backend_qt5agg import (NavigationToolbar2QT as NavigationToolbar)
import numpy as np
import random
import open3d as o3d     
from stl import mesh
from mpl_toolkits import mplot3d
import functions
import re
global xyz_load
global your_mesh


#%% debug ui


#%% definisco la gui vera e propria
class customdialog(QDialog):
    def __init__(self, *args, **kwargs):
        super(customdialog, self).__init__(*args, **kwargs)
        # super(Ui, self).__init__()
        loadUi('Qt_dialog.ui', self)
        self.scene_file_name = self.findChild(QtWidgets.QLineEdit, 'scene_file_edit')
        self.model_file_name = self.findChild(QtWidgets.QLineEdit, 'model_file_edit')
        self.hpr_scene_file_name = self.findChild(QtWidgets.QLineEdit, 'hpr_scene_file_edit')
        self.planner_file_dir = self.findChild(QtWidgets.QLineEdit, 'planner_file_dir')
        self.bo_file_dir = self.findChild(QtWidgets.QLineEdit, 'bo_file_dir')
        self.planner_online_debug_dir = self.findChild(QtWidgets.QLineEdit, 'planner_online_debug_dir')
        self.real_scan_dir = self.findChild(QtWidgets.QLineEdit, 'real_scan_dir')
        self.real_scan_file = self.findChild(QtWidgets.QLineEdit, 'real_scan_file')
        self.real_scan_trans_file = self.findChild(QtWidgets.QLineEdit, 'real_scan_trans_file')
        self.calib_file = self.findChild(QtWidgets.QLineEdit, 'calib_file')

        file = open(window.file_param_name,"r")
        self.param = file.readlines()                
        # QtWidgets.QLineEdit.setText()
        self.scene_file_name.setText(self.taglia_stringa("{(.*?)}",self.param[0]))
        self.model_file_name.setText(self.taglia_stringa("{(.*?)}",self.param[27]))
        self.hpr_scene_file_name.setText(self.taglia_stringa("{(.*?)}",self.param[28]))
        self.planner_file_dir.setText(self.taglia_stringa("{(.*?)}",self.param[37]))
        self.bo_file_dir.setText(self.taglia_stringa("{(.*?)}",self.param[38]))
        self.planner_online_debug_dir.setText(self.taglia_stringa("{(.*?)}",self.param[46]))
        self.real_scan_dir.setText(self.taglia_stringa("{(.*?)}",self.param[47]))
        self.real_scan_file.setText(self.taglia_stringa("{(.*?)}",self.param[48]))
        self.real_scan_trans_file.setText(self.taglia_stringa("{(.*?)}",self.param[49]))
        self.calib_file.setText(self.taglia_stringa("{(.*?)}",self.param[50]))
        #carico i parametri presenti nel file


        self.show()

    def taglia_stringa(self,pattern,stringa):
        substring = re.search(pattern, stringa).group(1)
        return substring

class MatplotlibWidget(QMainWindow):
    
    def __init__(self):
        self.file_param_name = "/home/loris/ply_and_stl/param.txt"

        file = open(self.file_param_name,"r")
        self.param = file.readlines()   
        self.scene_stl_save = self.taglia_stringa_string("{(.*?)}",self.param[0])
        self.directory = "/home/loris/ply_and_stl"
        self.file_model_name = self.taglia_stringa_string("{(.*?)}",self.param[27])
        self.HPR_scene_name = self.taglia_stringa_string("{(.*?)}",self.param[28])
        self.planner_file_dir=self.taglia_stringa_string("{(.*?)}",self.param[37])
        self.bo_file_dir=self.taglia_stringa_string("{(.*?)}",self.param[38])
        self.planner_online_debug_dir=self.taglia_stringa_string("{(.*?)}",self.param[46])
        self.real_scan_dir=self.taglia_stringa_string("{(.*?)}",self.param[47])
        self.real_scan_file=self.taglia_stringa_string("{(.*?)}",self.param[48])
        self.real_scan_trans_file=self.taglia_stringa_string("{(.*?)}",self.param[49])
        self.calib_file=self.taglia_stringa_string("{(.*?)}",self.param[50])
        
        self.ply_to_save = 0
        self.scene_trans = np.array([0,0,0],dtype=np.float64)
        QMainWindow.__init__(self)

        loadUi("Qt_GUI.ui",self)

        self.setWindowTitle("View simulator v1.0")
        #self.pushButton.clicked.connect(self.update_graph)
        self.addToolBar(NavigationToolbar(self.MplWidget.canvas, self))

        self.actionOpen = self.findChild(QtWidgets.QAction,"actionOpen")
        self.actionOpen_robot_file = self.findChild(QtWidgets.QAction,"actionOpen_robot_file")
        self.actionSave_stl = self.findChild(QtWidgets.QAction,"actionSave_stl")
        self.actionFilemanager = self.findChild(QtWidgets.QAction,"actionFile_manager")

        # self.actionOpen = QtWidgets.QAction()
        self.actionOpen.triggered.connect(self.file_open)
        self.actionOpen_robot_file.triggered.connect(self.robot_open)
        self.actionSave_stl.triggered.connect(self.save_stl)
        self.actionFilemanager.triggered.connect(self.file_man_open)
        #bottoni load and save

        self.load_button  = self.findChild(QtWidgets.QPushButton,"load_button")
        self.save_button  = self.findChild(QtWidgets.QPushButton,"save_button")
        self.update_button = self.findChild(QtWidgets.QPushButton,"update_button")
        self.load_button.clicked.connect(self.load_parameter)
        self.save_button.setEnabled(False)
        self.save_button.clicked.connect(self.save_parameter)
        self.update_button.clicked.connect(self.update_graph)



        #schermata view

        self.x_pos_box = self.findChild(QtWidgets.QDoubleSpinBox, 'x_pos_box')
        self.y_pos_box = self.findChild(QtWidgets.QDoubleSpinBox, 'y_pos_box')
        self.z_pos_box = self.findChild(QtWidgets.QDoubleSpinBox, 'z_pos_box')
        self.radius_box = self.findChild(QtWidgets.QDoubleSpinBox, 'radius_box')
        self.latitude_min_box = self.findChild(QtWidgets.QDoubleSpinBox, 'latitude_min_box')
        self.latitude_max_box = self.findChild(QtWidgets.QDoubleSpinBox, 'latitude_max_box')
        self.longitude_min_box = self.findChild(QtWidgets.QDoubleSpinBox, 'longitude_min_box')
        self.longitude_max_box = self.findChild(QtWidgets.QDoubleSpinBox, 'longitude_max_box')
        self.latitude_num_box = self.findChild(QtWidgets.QDoubleSpinBox, 'latitude_num')
        self.longitude_num_box = self.findChild(QtWidgets.QDoubleSpinBox, 'longitude_num')

        # self.x_pos_box.valueChanged.connect(self.update_graph)
        # self.y_pos_box.valueChanged.connect(self.update_graph)
        # self.z_pos_box.valueChanged.connect(self.update_graph)
        # self.radius_box.valueChanged.connect(self.update_graph)
        # self.latitude_min_box.valueChanged.connect(self.update_graph)
        # self.latitude_max_box.valueChanged.connect(self.update_graph)
        # self.longitude_min_box.valueChanged.connect(self.update_graph)
        # self.longitude_max_box.valueChanged.connect(self.update_graph)
        # self.latitude_num_box.valueChanged.connect(self.update_graph)
        # self.longitude_num_box.valueChanged.connect(self.update_graph)
        
        #schermata scena
        
        self.x_pos_scene_box = self.findChild(QtWidgets.QDoubleSpinBox, 'x_pos_scene_box')
        self.y_pos_scene_box = self.findChild(QtWidgets.QDoubleSpinBox, 'y_pos_scene_box')
        self.z_pos_scene_box = self.findChild(QtWidgets.QDoubleSpinBox, 'z_pos_scene_box')

        #schermata robot
        
        self.x_pos_robot_box = self.findChild(QtWidgets.QDoubleSpinBox, 'x_pos_robot_box')
        self.y_pos_robot_box = self.findChild(QtWidgets.QDoubleSpinBox, 'y_pos_robot_box')
        self.z_pos_robot_box = self.findChild(QtWidgets.QDoubleSpinBox, 'z_pos_robot_box')
        self.rz_robot_box = self.findChild(QtWidgets.QDoubleSpinBox, 'rz_robot_box')
        self.ry_robot_box = self.findChild(QtWidgets.QDoubleSpinBox, 'ry_robot_box')
        self.rx_robot_box = self.findChild(QtWidgets.QDoubleSpinBox, 'rx_robot_box')
        
        #schermata ppf
        
        
        self.model_sampling_box = self.findChild(QtWidgets.QDoubleSpinBox, 'model_sampling_box')
        self.scene_sampling_box = self.findChild(QtWidgets.QDoubleSpinBox, 'scene_sampling_box')
        self.ppf_normal_radius_box = self.findChild(QtWidgets.QDoubleSpinBox, 'ppf_normal_radius_box')
        self.ppf_normal_neighbors_box = self.findChild(QtWidgets.QDoubleSpinBox, 'ppf_normal_neighbours_box')
        self.training_completeness_box = self.findChild(QtWidgets.QDoubleSpinBox, 'training_completeness_box')
        self.training_keypoints_box = self.findChild(QtWidgets.QDoubleSpinBox, 'training_keypoints_box')
        self.matching_completeness_box = self.findChild(QtWidgets.QDoubleSpinBox, 'matching_completeness_box')
        self.matching_keypoints_box = self.findChild(QtWidgets.QDoubleSpinBox, 'matching_keypoints_box')

        #schermata halcon

        self.hpr_radius_box = self.findChild(QtWidgets.QDoubleSpinBox, 'hpr_radius_box')
        self.scan_sampling_distance_box = self.findChild(QtWidgets.QDoubleSpinBox, 'scan_sampling_distance_box')
        self.model_rough_sampling_box = self.findChild(QtWidgets.QDoubleSpinBox, 'model_rough_sampling_box')
        self.model_fine_sampling_box = self.findChild(QtWidgets.QDoubleSpinBox, 'model_fine_sampling_box')
        self.match_min_score_box = self.findChild(QtWidgets.QDoubleSpinBox, 'match_min_score_box')
        self.match_sample_distance_box = self.findChild(QtWidgets.QDoubleSpinBox, 'match_sample_distance_box')
        self.keypoints_fraction_box = self.findChild(QtWidgets.QDoubleSpinBox, 'keypoints_fraction_box')
        
        #schermata bayesian optimization
        self.bo_test = self.findChild(QtWidgets.QDoubleSpinBox, 'bo_test')
        self.bo_iteration = self.findChild(QtWidgets.QDoubleSpinBox, 'bo_iteration')
        self.bo_alpha = self.findChild(QtWidgets.QDoubleSpinBox, 'bo_alpha')
        self.bo_test_no_box = self.findChild(QtWidgets.QDoubleSpinBox, 'bo_test_no_box')
        self.bo_iteration_no_box = self.findChild(QtWidgets.QDoubleSpinBox, 'bo_iteration_no_box')
        self.bo_enable_check = self.findChild(QtWidgets.QCheckBox,"bo_enable")
        self.score_pen_box = self.findChild(QtWidgets.QDoubleSpinBox, 'score_pen_box')
        self.score_reward_box = self.findChild(QtWidgets.QDoubleSpinBox, 'score_reward_box')
        self.score_pen_box_v = self.findChild(QtWidgets.QDoubleSpinBox, 'score_pen_box_v')
        self.score_reward_box_v = self.findChild(QtWidgets.QDoubleSpinBox, 'score_reward_box_v')
        self.bo_enable_check.stateChanged.connect(self.enable_bo)
    
    def enable_bo(self):
        if self.bo_enable_check.isChecked() == True:
            self.score_pen_box.setEnabled(True)
            self.score_reward_box.setEnabled(True)
            self.score_pen_box_v.setEnabled(True)
            self.score_reward_box_v.setEnabled(True)
        else:
            self.score_pen_box.setEnabled(False)
            self.score_reward_box.setEnabled(False)
            self.score_pen_box_v.setEnabled(False)
            self.score_reward_box_v.setEnabled(False)

    
    
    def file_man_open(self):
        
        dlg = customdialog()

        returnValue = dlg.exec_()
        if returnValue == 1: 
            window.scene_stl_save = dlg.scene_file_name.text()
            window.file_model_name = dlg.model_file_name.text()
            window.HPR_scene_name = dlg.hpr_scene_file_name.text()
            window.planner_file_dir = dlg.planner_file_dir.text()
            window.bo_file_dir = dlg.bo_file_dir.text()
            window.planner_online_debug_dir = dlg.planner_online_debug_dir.text()
            window.real_scan_dir = dlg.real_scan_dir.text()
            window.real_scan_file = dlg.real_scan_file.text()
            window.real_scan_trans_file = dlg.real_scan_trans_file.text()
            window.calib_file = dlg.calib_file.text()

            


    def spin_changed(self):
        spinValue = self.x_pos_box.value()
        print(spinValue)
        self.update_graph()

    def save_parameter(self):

        x_centre = self.x_pos_box.value()
        y_centre = self.y_pos_box.value()
        z_centre = self.z_pos_box.value()
        radius = self.radius_box.value()
        lat_min = self.latitude_min_box.value()
        lat_max = self.latitude_max_box.value()
        long_min = self.longitude_min_box.value()
        long_max = self.longitude_max_box.value()
        n_lat = self.latitude_num_box.value()
        n_long = self.longitude_num_box.value()
        x_pos_scene = self.x_pos_scene_box.value()
        y_pos_scene = self.y_pos_scene_box.value()
        z_pos_scene = self.z_pos_scene_box.value()
        x_pos_robot = self.x_pos_robot_box.value()
        y_pos_robot = self.y_pos_robot_box.value()
        z_pos_robot = self.z_pos_robot_box.value()
        rx_robot = self.rx_robot_box.value()
        ry_robot = self.ry_robot_box.value()
        rz_robot = self.rz_robot_box.value() 
        hpr_radius = self.hpr_radius_box.value()
        ppf_mod_sampling = self.model_sampling_box.value()
        ppf_scene_sampling = self.scene_sampling_box.value()
        ppf_normal_radius = self.ppf_normal_radius_box.value()
        ppf_normal_neighbours = self.ppf_normal_neighbors_box.value()
        ppf_train_comp = self.training_completeness_box.value()
        ppf_train_keyp = self.training_keypoints_box.value()
        ppf_matching_comp = self.matching_completeness_box.value()
        ppf_matching_keyp = self.matching_keypoints_box.value()        
        scan_sampling_distance = self.scan_sampling_distance_box.value()
        model_rough_sampling = self.model_rough_sampling_box.value()
        model_fine_sampling = self.model_fine_sampling_box.value()
        match_min_score = self.match_min_score_box.value()
        match_sample_distance = self.match_sample_distance_box.value()
        keypoints_fraction = self.keypoints_fraction_box.value()
        
    #    name = QFileDialog.getSaveFileName(self,"Open File",self.directory,"Files (*.stl)")
    #    fileName = QFileDialog.getSaveFileName(self, "Save F:xile",self.directory,"text_file (*.txt)")
        file = open(self.file_param_name,"w")
        string_filename = "0. translated stl file: \t{" + self.scene_stl_save +"}"
        file.write(string_filename)
        file.write("\n1.sphere centre position x:\t{%.4f}" %x_centre)
        file.write("\n2.sphere centre position y:\t{%.4f}" %y_centre)
        file.write("\n3.sphere centre position z:\t{%.4f}" %z_centre)
        file.write("\n4.sphere radius:\t{%.4f}" %radius)
        file.write("\n5.minimum latitude:\t{%.4f}" %lat_min)
        file.write("\n6.maximum latitude:\t{%.4f}" %lat_max)
        file.write("\n7.latitude sample points:\t{%.4f}" %n_lat)
        file.write("\n8.minimum longitude:\t{%.4f}" %long_min)
        file.write("\n9.maximum longitude:\t{%.4f}" %long_max)
        file.write("\n10.longitude sample points:\t{%.4f}" %n_long)
        file.write("\n11.scene position x:\t{%.4f}" %x_pos_scene)
        file.write("\n12.scene position y:\t{%.4f}" %y_pos_scene)
        file.write("\n13.scene position z:\t{%.4f}" %z_pos_scene)
        file.write("\n14.robot position x:\t{%.4f}" %x_pos_robot)
        file.write("\n15.robot position y:\t{%.4f}" %y_pos_robot)
        file.write("\n16.robot position z:\t{%.4f}" %z_pos_robot)
        file.write("\n17.robot Rz rotation:\t{%.4f}" %rz_robot)
        file.write("\n18.robot Ry rotation:\t{%.4f}" %ry_robot)
        file.write("\n19.robot Rx rotation:\t{%.4f}" %rx_robot)
        file.write("\n20.HPR radius:\t{%.4f}" %hpr_radius)
        file.write("\n21.ppf model sampling:\t{%.4f}" %ppf_mod_sampling)
        file.write("\n22.ppf scene sampling:\t{%.4f}" %ppf_scene_sampling)
        file.write("\n23.ppf training completeness:\t{%.4f}" %ppf_train_comp)
        file.write("\n24.ppf training keypoints:\t{%.4f}" %ppf_train_keyp)
        file.write("\n25.ppf matching completeness:\t{%.4f}" %ppf_matching_comp)
        file.write("\n26.ppf matching keypoints:\t{%.4f}" %ppf_matching_keyp)
        file.write("\n27.model_file_name:\t{%s}" %self.file_model_name)
        file.write("\n28.HPR_scene_file_name:\t{%s}" %self.HPR_scene_name)
        file.write("\n29.scan sampling distance:\t{%s}" %scan_sampling_distance)
        file.write("\n30.model rough sampling:\t{%s}" %model_rough_sampling)
        file.write("\n31.model fine sampling:\t{%s}" %model_fine_sampling)
        file.write("\n32.matching minimum score:\t{%s}" %match_min_score)
        file.write("\n33.matching sample distance:\t{%s}" %match_sample_distance)
        file.write("\n34.keypoins fraction:\t{%s}" %keypoints_fraction)
        file.write("\n35.ppf scene normal radius:\t{%s}" %ppf_normal_radius)
        file.write("\n36.ppf_normal_neighbours:\t{%s}" %ppf_normal_neighbours)
        file.write("\n37.planner debug file directory:\t{%s}" %self.planner_file_dir)
        file.write("\n38.bayesian optimization debug file directory:\t{%s}" %self.bo_file_dir)
        file.write("\n39.bayesian optimization reachability weigth:\t{%d}" %self.bo_test_no_box.value())
        file.write("\n40.bayesian optimization score weigth:\t{%d}" %self.bo_iteration_no_box.value())
        file.write("\n41.bayesian optimization debug file directory:\t{%s}" %self.bo_enable_check.isChecked())
        file.write("\n42.bayesian optimization score penalty:\t{%.4f}" %self.score_pen_box.value())
        file.write("\n43.bayesian optimization score reward:\t{%.4f}" %self.score_reward_box.value())
        file.write("\n44.bayesian optimization score penalty value:\t{%d}" %self.score_pen_box_v.value())
        file.write("\n45.bayesian optimization score reward value:\t{%d}" %self.score_reward_box_v.value())
        file.write("\n46.planner online debug file directory:\t{%s}" %self.planner_online_debug_dir)
        file.write("\n47.real scan diirectory:\t{%s}" %self.real_scan_dir)
        file.write("\n48.real scan file:\t{%s}" %self.real_scan_file)
        file.write("\n49.real translated scan file:\t{%s}" %self.real_scan_trans_file)
        file.write("\n50.calibration matrix file:\t{%s}" %self.calib_file)
        file.write("\n51.bayesian optimization test number:\t{%d}" %self.bo_test.value())
        file.write("\n52.bayesian optimization iteration:\t{%d}" %self.bo_iteration.value())
        file.write("\n53.bayesian optimization alpha value:\t{%d}" %self.bo_alpha.value())


    def taglia_stringa(self,pattern,stringa):
        substring = re.search(pattern, stringa).group(1)
        return float(substring)

    def taglia_stringa_string(self,pattern,stringa):
        substring = re.search(pattern, stringa).group(1)
        return substring
    def load_parameter(self):
        
        file = open(self.file_param_name,"r")
        param = file.readlines()        
        self.x_pos_box.setValue(self.taglia_stringa("{(.*?)}",param[1]))
        self.y_pos_box.setValue(self.taglia_stringa("{(.*?)}",param[2]))      
        self.z_pos_box.setValue(self.taglia_stringa("{(.*?)}",param[3]))
        self.radius_box.setValue(self.taglia_stringa("{(.*?)}",param[4]))
        self.latitude_min_box.setValue(self.taglia_stringa("{(.*?)}",param[5]))
        self.latitude_max_box.setValue(self.taglia_stringa("{(.*?)}",param[6]))
        self.latitude_num_box.setValue(self.taglia_stringa("{(.*?)}",param[7]))
        self.longitude_min_box.setValue(self.taglia_stringa("{(.*?)}",param[8]))
        self.longitude_max_box.setValue(self.taglia_stringa("{(.*?)}",param[9]))
        self.longitude_num_box.setValue(self.taglia_stringa("{(.*?)}",param[10]))
        self.x_pos_scene_box.setValue(self.taglia_stringa("{(.*?)}",param[11]))
        self.y_pos_scene_box.setValue(self.taglia_stringa("{(.*?)}",param[12]))
        self.z_pos_scene_box.setValue(self.taglia_stringa("{(.*?)}",param[13]))
        self.x_pos_robot_box.setValue(self.taglia_stringa("{(.*?)}",param[14]))
        self.y_pos_robot_box.setValue(self.taglia_stringa("{(.*?)}",param[15]))
        self.z_pos_robot_box.setValue(self.taglia_stringa("{(.*?)}",param[16]))
        self.rz_robot_box.setValue(self.taglia_stringa("{(.*?)}",param[17]))
        self.ry_robot_box.setValue(self.taglia_stringa("{(.*?)}",param[18]))
        self.rx_robot_box.setValue(self.taglia_stringa("{(.*?)}",param[19]))
        self.hpr_radius_box.setValue(self.taglia_stringa("{(.*?)}",param[20]))
        self.model_sampling_box.setValue(self.taglia_stringa("{(.*?)}",param[21]))
        self.scene_sampling_box.setValue(self.taglia_stringa("{(.*?)}",param[22]))
        self.training_completeness_box.setValue(self.taglia_stringa("{(.*?)}",param[23]))
        self.training_keypoints_box.setValue(self.taglia_stringa("{(.*?)}",param[24]))
        self.matching_completeness_box.setValue(self.taglia_stringa("{(.*?)}",param[25]))
        self.matching_keypoints_box.setValue(self.taglia_stringa("{(.*?)}",param[26]))
        self.scan_sampling_distance_box.setValue(self.taglia_stringa("{(.*?)}",param[29]))
        self.model_rough_sampling_box.setValue(self.taglia_stringa("{(.*?)}",param[30]))
        self.model_fine_sampling_box.setValue(self.taglia_stringa("{(.*?)}",param[31]))
        self.match_min_score_box.setValue(self.taglia_stringa("{(.*?)}",param[32]))
        self.match_sample_distance_box.setValue(self.taglia_stringa("{(.*?)}",param[33]))
        self.keypoints_fraction_box.setValue(self.taglia_stringa("{(.*?)}",param[34]))
        self.ppf_normal_radius_box.setValue(self.taglia_stringa("{(.*?)}",param[35]))
        self.ppf_normal_neighbors_box.setValue(self.taglia_stringa("{(.*?)}",param[36]))
        self.bo_test_no_box.setValue(self.taglia_stringa("{(.*?)}",param[39]))
        self.bo_iteration_no_box.setValue(self.taglia_stringa("{(.*?)}",param[40]))
        self.bo_test.setValue(self.taglia_stringa("{(.*?)}",param[51]))
        self.bo_iteration.setValue(self.taglia_stringa("{(.*?)}",param[52]))
        self.bo_alpha.setValue(self.taglia_stringa("{(.*?)}",param[53]))  

        if self.taglia_stringa_string("{(.*?)}",param[41])=="True":
            self.bo_enable_check.setChecked(True)
            self.score_pen_box.setEnabled(True)
            self.score_reward_box.setEnabled(True)
        else:
            self.bo_enable_check.setChecked(False)
            self.score_pen_box.setEnabled(False)
            self.score_reward_box.setEnabled(False)
        self.score_pen_box.setValue(self.taglia_stringa("{(.*?)}",param[42]))
        self.score_reward_box.setValue(self.taglia_stringa("{(.*?)}",param[43]))
        self.score_pen_box_v.setValue(self.taglia_stringa("{(.*?)}",param[44]))
        self.score_reward_box_v.setValue(self.taglia_stringa("{(.*?)}",param[45]))

        self.save_button.setEnabled(True)


        








    def open_popup(self):
        msg = QMessageBox()
        msg.setWindowTitle("View simulator v1.0")
        msg.setText("Load a new robot file before start")
        msg.setIcon(QMessageBox.Information)
        msg.setStandardButtons(QMessageBox.Ignore|QMessageBox.Open)
        msg.setDefaultButton(QMessageBox.Open)
        
        msg.move(950/2,700/2)
        returnValue = msg.exec_() 
        if returnValue == QMessageBox.Open:
            self.robot_open()
    def info_popup(self,text):
        msg = QMessageBox()
        msg.setWindowTitle("View simulator v1.0")
        msg.setText(text)
        msg.setIcon(QMessageBox.Information)
        msg.setStandardButtons(QMessageBox.Ok )
        msg.setDefaultButton(QMessageBox.Ok)
        msg.move(950/2,700/2)
        returnValue = msg.exec()

   

    def file_open(self):
        global your_mesh    
        global downpcd 
        
 #       scale = your_mesh.points.flatten()
        name = QFileDialog.getOpenFileName(self,"Open File","/home/loris/ply_and_stl","Files (*.ply *.stl)")
        self.name_ply =name
        pcd = o3d.io.read_point_cloud(name[0]) 
        downpcd = pcd.voxel_down_sample(voxel_size=.015)

        xyz_load = np.asarray(downpcd.points)

        self.MplWidget.canvas.axes.clear()
        
        self.MplWidget.canvas.axes.scatter(xyz_load[:,0],xyz_load[:,1],xyz_load[:,2],s=1)
        self.MplWidget.canvas.axes.set_xlabel('X Label')
        self.MplWidget.canvas.axes.set_ylabel('Y Label')
        self.MplWidget.canvas.axes.set_zlabel('Z Label')
        self.MplWidget.canvas.axes.add_collection3d(mplot3d.art3d.Poly3DCollection(your_mesh.vectors))  
 #       self.MplWidget.canvas.axes.auto_scale_xyz(scale, scale, scale)
        #self.MplWidget.canvas.axes.legend(('point cloud', 'sphere centre'),loc='upper right')
        self.MplWidget.canvas.draw()
        print (name)
    def save_stl(self):
        translation = np.array([0,0,0],dtype=np.float64)
        translation[0] = self.x_pos_scene_box.value()
        translation[1] = self.y_pos_scene_box.value()
        translation[2] = self.z_pos_scene_box.value()

        pcd = o3d.io.read_point_cloud(self.name_ply[0])         
        fileName_ply = QFileDialog.getSaveFileName(self, "Save ply file",self.directory,"ply file (*.ply)")
        self.scene_stl_save = fileName_ply[0]
        print(translation)
        print(fileName_ply)
        pcd_1 = pcd.translate(translation)

        o3d.io.write_point_cloud(fileName_ply[0],pcd_1)    
        #o3d.io.write_point_cloud(fileName[0],pcd_to_save_t)
        self.info_popup("Point cloud correctly translated and saved")   
    def robot_open(self):        
        global your_mesh
        self.MplWidget.canvas.axes.clear()
        #self.MplWidget.canvas.axes.scatter(xyz_load[:,0],xyz_load[:,1],xyz_load[:,2],s=1)
        self.MplWidget.canvas.axes.set_xlabel('X Label')
        self.MplWidget.canvas.axes.set_ylabel('Y Label')
        self.MplWidget.canvas.axes.set_zlabel('Z Label')
        name = QFileDialog.getOpenFileName(self,"Open File","/home/loris/ply_and_stl","Files (*.stl)")
        your_mesh = mesh.Mesh.from_file(name[0])
        scale = your_mesh.points.flatten()
        self.MplWidget.canvas.axes.add_collection3d(mplot3d.art3d.Poly3DCollection(your_mesh.vectors))
        self.MplWidget.canvas.axes.autoscale_view(tight=None, scalex=True, scaley=True)
        self.MplWidget.canvas.draw()

    def update_graph(self):

        global downpcd
        global your_mesh
         
        x_centre = self.x_pos_box.value()
        y_centre = self.y_pos_box.value()
        z_centre = self.z_pos_box.value()
        print(z_centre)
        radius = self.radius_box.value()
        lat_min = self.latitude_min_box.value()
        lat_max = self.latitude_max_box.value()
        long_min = self.longitude_min_box.value()
        long_max = self.longitude_max_box.value()
        n_lat = self.latitude_num_box.value()
        n_long = self.longitude_num_box.value()
        x_pos_scene = self.x_pos_scene_box.value()
        y_pos_scene = self.y_pos_scene_box.value()
        z_pos_scene = self.z_pos_scene_box.value()
        x_pos_robot = self.x_pos_robot_box.value()
        y_pos_robot = self.y_pos_robot_box.value()
        z_pos_robot = self.z_pos_robot_box.value()
        rx_robot = self.rx_robot_box.value()
        ry_robot = self.ry_robot_box.value()
        rz_robot = self.rz_robot_box.value() 
        self.scene_trans[0] = x_pos_scene
        self.scene_trans[1] = y_pos_scene
        self.scene_trans[2] = z_pos_scene
        print(self.scene_trans)
        #traslazioni e rotazioni delle nuvole
        sphere_points=functions.sphere_generator(x_centre, y_centre, z_centre, radius, lat_min, lat_max, n_lat, long_min, long_max, n_long)
        mesh_points = functions.rotate_and_tranlate_stl(your_mesh.vectors,rz_robot,ry_robot,rx_robot,x_pos_robot,y_pos_robot,z_pos_robot)
        downpcd_trans = downpcd.translate(self.scene_trans,relative=False)
        xyz_load = np.asarray(downpcd_trans.points)
        print(mesh_points)
        self.MplWidget.canvas.axes.clear()
        self.MplWidget.canvas.axes.scatter(xyz_load[:,0],xyz_load[:,1],xyz_load[:,2],s=1)
        self.MplWidget.canvas.axes.scatter(sphere_points[:,0],sphere_points[:,1],sphere_points[:,2], c = 'r',)
        self.MplWidget.canvas.axes.scatter(x_centre,y_centre,z_centre, c = 'g', s = 20)
        self.MplWidget.canvas.axes.add_collection3d(mplot3d.art3d.Poly3DCollection(mesh_points))
        self.MplWidget.canvas.axes.set_xlabel('X Label')
        self.MplWidget.canvas.axes.set_ylabel('Y Label')
        self.MplWidget.canvas.axes.set_zlabel('Z Label')
        self.MplWidget.canvas.axes.autoscale_view(tight=None, scalex=True, scaley=True)
        #self.MplWidget.canvas.axes.legend(('cosinus', 'sinus'),loc='upper right')
        self.MplWidget.canvas.draw()


        

app = QApplication([])
window = MatplotlibWidget()
window.show()
window.open_popup()
window.enable_bo()
app.exec_()

