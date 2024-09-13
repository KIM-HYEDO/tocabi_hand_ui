#!/usr/bin/env python3
import sys
import os
from ruamel.yaml import YAML
import numpy as np
from datetime import datetime
import rospy
import time
import dynamixel_sdk as dxl

# msgs
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Bool

# QT
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtGui import QPixmap, QIcon
from PyQt5.QtCore import QThread, QObject, pyqtSignal, QTimer


# read yaml file
yaml = YAML()
yaml.preserve_quotes = True
yaml.indent(mapping=2, sequence=4, offset=2)
script_directory = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(script_directory, 'hand.yaml'), 'r') as file:
        config = yaml.load(file)
globals().update(config)

form_class = uic.loadUiType(os.path.join(script_directory, "TocabiHand.ui"))[0]

class Worker(QObject):
    finished = pyqtSignal()  # give worker class a finished signal

    def __init__(self, parent=None):
        QObject.__init__(self, parent=parent)
        self.continue_run = True  # provide a bool run condition for the class

    def run(self):
        i = 1
        while self.continue_run:  # give the loop a stoppable condition
            
            time.sleep(0.05)
            i = i + 0.05
        self.finished.emit()  # emit the finished signal when the loop is done

    def stop(self):
        self.continue_run = False  # set the run condition to false on stop

class HandCtrlGUI(QMainWindow, form_class) :

    stop_signal = pyqtSignal()

    def __init__(self) :
        super().__init__()
        self.setupUi(self)
        self.load_param()
        self.init_ui()
        self.init_timer()
        self.InitThread()
        self.init_ros()
        # self.__init_comm()
        # self.init_dxl()
        # self.heartbeat()

    def load_param(self):
        self.Init_Pos_AA         = config['Init_Pos_AA']
        self.Init_Pos_FE         = config['Init_Pos_FE']
        self.Init_Cur_FE         = config['Init_Cur_FE']
        self.PositionLimit_FE    = config['PositionLimit_FE']
        self.PositionMinLimit_FE = config['PositionMinLimit_FE']
        self.PositionLimit_AA    = config['PositionLimit_AA']
        self.PositionMinLimit_AA = config['PositionMinLimit_AA']
        self.CurrentLimit_FE     = config['CurrentLimit_FE']
        self.CurrentLimit_AA     = config['CurrentLimit_AA']
        self.Gain_position_FE    = config['Gain_position_FE']
        self.Gain_position_AA    = config['Gain_position_AA']
        self.Grasp_Target_Cur_AA = config['Grasp_Target_Cur_AA']
        self.Grasp_Target_Cur_FE = config['Grasp_Target_Cur_FE']
        self.Grasp_Target_Pos_AA = config['Grasp_Target_Pos_AA']
        self.Grasp_Target_Pos_FE = config['Grasp_Target_Pos_FE']
        self.Grasp_Ctrl_Mode_AA  = 1  #  0:current control,  1:current position control 
        self.Grasp_Ctrl_Mode_FE  = 0
        self.joint_positions     = np.zeros(NUM_JOINT)
        self.joint_currents      = np.zeros(NUM_JOINT,dtype=np.int16)

        self.setting_var_AA      = [self.Gain_position_AA, self.CurrentLimit_AA, self.PositionLimit_AA, self.PositionMinLimit_AA]
        self.setting_var_FE      = [self.Gain_position_FE, self.CurrentLimit_FE, self.PositionLimit_FE, self.PositionMinLimit_FE]
        self.control_var_AA      = [self.Grasp_Target_Cur_AA, self.Grasp_Target_Pos_AA]
        self.control_var_FE      = [self.Grasp_Target_Cur_FE, self.Grasp_Target_Pos_FE]

    def init_ui(self):

        ####### Set image #######
        pixmap = QPixmap('image.png')
        self.label_image.setPixmap(pixmap)
        self.label_image.setScaledContents(True)
        self.label_image2.setPixmap(pixmap)
        self.label_image2.setScaledContents(True)

        ####### Set Variables #######
        ### UI in Control ###
        self.AA_control_slidebar   = [self.AA1_control_slide, self.AA2_control_slide, self.AA3_control_slide, self.AA4_control_slide]
        self.AA_control_slideLabel = [self.AA1_control_label, self.AA2_control_label, self.AA3_control_label, self.AA4_control_label]
        self.FE_control_slidebar   = [self.FE1_control_slide, self.FE2_control_slide, self.FE3_control_slide, self.FE4_control_slide]
        self.FE_control_slideLabel = [self.FE1_control_label, self.FE2_control_label, self.FE3_control_label, self.FE4_control_label]
        self.AA_checkbox = [self.checkBox_AA1, self.checkBox_AA2, self.checkBox_AA3, self.checkBox_AA4]
        self.FE_checkbox = [self.checkBox_FE1, self.checkBox_FE2, self.checkBox_FE3, self.checkBox_FE4]
        self.AA_live_pos = [self.label_live_AA1_pos, self.label_live_AA2_pos, self.label_live_AA3_pos, self.label_live_AA4_pos]
        self.AA_live_cur = [self.label_live_AA1_cur, self.label_live_AA2_cur, self.label_live_AA3_cur, self.label_live_AA4_cur]
        self.FE_live_pos = [self.label_live_FE1_pos, self.label_live_FE2_pos, self.label_live_FE3_pos, self.label_live_FE4_pos]
        self.FE_live_cur = [self.label_live_FE1_cur, self.label_live_FE2_cur, self.label_live_FE3_cur, self.label_live_FE4_cur]

        ### UI in Setting ###
        self.AA_setting_slidebar   = [self.AA_D_slide, self.AA_I_slide, self.AA_P_slide, self.AA1_cur_slide, self.AA2_cur_slide, self.AA3_cur_slide, self.AA4_cur_slide, self.AA1_pos_slide, self.AA2_pos_slide, self.AA3_pos_slide, self.AA4_pos_slide, self.AA1_pos_slide_min, self.AA2_pos_slide_min, self.AA3_pos_slide_min, self.AA4_pos_slide_min]
        self.AA_setting_slideLabel = [self.AA_D_label, self.AA_I_label, self.AA_P_label, self.AA1_cur_label, self.AA2_cur_label, self.AA3_cur_label, self.AA4_cur_label, self.AA1_pos_label, self.AA2_pos_label, self.AA3_pos_label, self.AA4_pos_label, self.AA1_pos_label_min, self.AA2_pos_label_min, self.AA3_pos_label_min, self.AA4_pos_label_min]
        self.FE_setting_slidebar   = [self.FE_D_slide, self.FE_I_slide, self.FE_P_slide, self.FE1_cur_slide, self.FE2_cur_slide, self.FE3_cur_slide, self.FE4_cur_slide, self.FE1_pos_slide, self.FE2_pos_slide, self.FE3_pos_slide, self.FE4_pos_slide, self.FE1_pos_slide_min, self.FE2_pos_slide_min, self.FE3_pos_slide_min, self.FE4_pos_slide_min]
        self.FE_setting_slideLabel = [self.FE_D_label, self.FE_I_label, self.FE_P_label, self.FE1_cur_label, self.FE2_cur_label, self.FE3_cur_label, self.FE4_cur_label, self.FE1_pos_label, self.FE2_pos_label, self.FE3_pos_label, self.FE4_pos_label, self.FE1_pos_label_min, self.FE2_pos_label_min, self.FE3_pos_label_min, self.FE4_pos_label_min]


        ########## Set Connects ##########
        ####### UI in Control #######
        ### Set Buttons ###
        self.btn_torqueon.clicked.connect(self.buttonTorqueOn)
        self.btn_torqueoff.clicked.connect(self.buttonTorqueOff)
        self.btn_init.clicked.connect(self.buttonInit)
        self.btn_grasp.clicked.connect(self.buttonGrasp)
        self.btn_stretch.clicked.connect(self.buttonStretch)
        self.btn_AA_grasp_current_control.clicked.connect(self.button_AA_grasp_current_control)
        self.btn_AA_grasp_current_position_control.clicked.connect(self.button_AA_grasp_current_position_control)
        self.btn_FE_grasp_current_control.clicked.connect(self.button_FE_grasp_current_control)
        self.btn_FE_grasp_current_position_control.clicked.connect(self.button_FE_grasp_current_position_control)
        self.btn_act_on.clicked.connect(self.button_ACT_ON)
        self.btn_act_off.clicked.connect(self.button_ACT_OFF)
        

        ### Set Slidebars ###
        for idx, (slide, label) in enumerate(zip(self.AA_control_slidebar, self.AA_control_slideLabel)):
            slide.valueChanged.connect(lambda value, s=slide, l=label, i=idx: self.sliderControlValueChanged(s, l, i))
            # label.textChanged.connect(lambda value, s=slide, l=label: self.labelValueChanged(s, l))
        for idx, (slide, label) in enumerate(zip(self.FE_control_slidebar, self.FE_control_slideLabel)):
            slide.valueChanged.connect(lambda value, s=slide, l=label, i=idx: self.sliderControlValueChanged(s, l, i))
            # label.textChanged.connect(lambda value, s=slide, l=label: self.labelValueChanged(s, l))

        ### Set Checkboxes ###
        self.checkBox_AA_ALL.stateChanged.connect(lambda value, acb=self.checkBox_AA_ALL, cb=self.AA_checkbox: self.checkBox_ALL_changed(acb, cb))
        self.checkBox_FE_ALL.stateChanged.connect(lambda value, acb=self.checkBox_FE_ALL, cb=self.FE_checkbox: self.checkBox_ALL_changed(acb, cb))
        
        ####### UI in Setting #######
        ### Set Buttons ###
        self.btn_save.clicked.connect(self.buttonSave)

        ### Set Slidebars ###
        for idx, (slide, label) in enumerate(zip(self.AA_setting_slidebar, self.AA_setting_slideLabel)):
            slide.valueChanged.connect(lambda value, s=slide, l=label, i=idx: self.sliderSettingValueChanged(s, l, i))
            # label.textChanged.connect(lambda value, s=slide, l=label, i=idx: self.labelValueChanged(s, l))
            if idx >= 3:
                slide.valueChanged.connect(lambda value, s1=slide, s2=self.AA_control_slidebar[(idx-3)%4], idx=idx, m="AA": self.LimitValueChanged(s1, s2, idx, m))
            
        for idx, (slide, label) in enumerate(zip(self.FE_setting_slidebar, self.FE_setting_slideLabel)):
            slide.valueChanged.connect(lambda value, s=slide, l=label, i=idx: self.sliderSettingValueChanged(s, l, i))
            # label.textChanged.connect(lambda value, s=slide, l=label, i=idx: self.labelValueChanged(s, l))
            if idx >= 3:
                slide.valueChanged.connect(lambda value, s1=slide, s2=self.FE_control_slidebar[(idx-3)%4], idx=idx, m="FE": self.LimitValueChanged(s1, s2, idx, m))
        # for slide1, slide2 in zip(self.AA_setting_slidebar[3:], self.AA_control_slidebar*2):
        #     slide1.valueChanged.connect(lambda value, s1=slide1, s2=slide2: self.LimitValueChanged(s1, s2))
        # for slide1, slide2 in zip(self.FE_setting_slidebar[3:], self.FE_control_slidebar*2):
        #     slide1.valueChanged.connect(lambda value, s1=slide1, s2=slide2: self.LimitValueChanged(s1, s2))

        ####### Set init #######
        self.Init()

    def InitThread(self):
        self.thread = QThread()
        self.worker = Worker()
        self.stop_signal.connect(self.worker.stop)  # connect stop signal to worker stop method
        self.worker.moveToThread(self.thread)

        self.worker.finished.connect(self.thread.quit)  # connect the workers finished signal to stop thread
        self.worker.finished.connect(self.worker.deleteLater)  # connect the workers finished signal to clean up worker
        self.thread.finished.connect(self.thread.deleteLater)  # connect threads finished signal to clean up thread

        self.thread.started.connect(self.worker.run)
        self.thread.finished.connect(self.worker.stop)
        # self.thread.start()
        # # Start Button action:
        # self.btn_start.clicked.connect(self.thread.start)

        # # Stop Button action:
        # self.btn_stop.clicked.connect(self.stop_thread)
    
    def stop_thread(self):
        self.stop_signal.emit()

    def init_timer(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.run)
        self.timer.start(100)

    def run(self):
        self.read_joint_position()
        self.read_joint_current()
        # print("self.setting_var_AA : ", self.setting_var_AA)
        # print("-------------")
        # print("self.setting_var_AA : ", self.setting_var_AA)
        # print("self.setting_var_FE : ", self.setting_var_FE)
        # print("self.control_var_AA : ", self.control_var_AA)
        # print("self.control_var_FE : ", self.control_var_FE)

    ### Button Setting ###
    def buttonInit(self) :
        print("Init !!")
        self.init_dxl()

    def buttonTorqueOn(self) :
        self.torque_on()

    def buttonTorqueOff(self) :
        self.torque_off()

    def buttonGrasp(self) :
        self.btn_grasp.setStyleSheet("background-color: #66FF66;")
        self.btn_stretch.setStyleSheet("background-color: lightgray;")
        print("Grasp !!")
        self.grasp()

    def buttonStretch(self) :
        self.btn_stretch.setStyleSheet("background-color: #66FF66;")
        self.btn_grasp.setStyleSheet("background-color: lightgray;")
        print("Stretch !!")
        self.stretch()
    
    def button_AA_grasp_current_control(self) :
        self.label_AA_grasp_mode.setText("Grasp Control Mode : Current")
        self.label_AA_grasp_value.setText("Value : Goal Current")
        self.Grasp_Ctrl_Mode_AA = 0
        for slide, max, value in zip(self.AA_control_slidebar, self.CurrentLimit_AA, self.Grasp_Target_Cur_AA) :
            slide.setMinimum(0)
            slide.setMaximum(max)
            slide.setValue(value)
    
    def button_AA_grasp_current_position_control(self) :
        self.label_AA_grasp_mode.setText("Grasp Control Mode : Current Position")
        self.label_AA_grasp_value.setText("Value : Goal Position")
        self.Grasp_Ctrl_Mode_AA = 1
        for slide, max, min, value in zip(self.AA_control_slidebar, self.PositionLimit_AA, self.PositionMinLimit_AA, self.Grasp_Target_Pos_AA) :
            slide.setMinimum(min)
            slide.setMaximum(max)
            slide.setValue(value)
    
    def button_FE_grasp_current_control(self) :
        self.label_FE_grasp_mode.setText("Grasp Control Mode : Current")
        self.label_FE_grasp_value.setText("Value : Goal Current")
        self.Grasp_Ctrl_Mode_FE = 0
        for slide, max, value in zip(self.FE_control_slidebar, self.CurrentLimit_FE, self.Grasp_Target_Cur_FE) :
            slide.setMinimum(0)
            slide.setMaximum(max)
            slide.setValue(value)

    def button_FE_grasp_current_position_control(self) :
        self.label_FE_grasp_mode.setText("Grasp Control Mode : Current Position")
        self.label_FE_grasp_value.setText("Value : Goal Position")
        self.Grasp_Ctrl_Mode_FE = 1
        for slide, max, min, value in zip(self.FE_control_slidebar, self.PositionLimit_FE, self.PositionMinLimit_FE, self.Grasp_Target_Pos_FE) :
            slide.setMinimum(min)
            slide.setMaximum(max)
            slide.setValue(value)

    def button_ACT_ON(self):
        data=Bool()
        data.data = False
        self.act_pub.publish(data)

    def button_ACT_OFF(self):
        data=Bool()
        data.data = True
        self.act_pub.publish(data)
    
    def buttonSave(self) :
        config['Init_Pos_AA'] = self.Init_Pos_AA
        config['Init_Pos_FE'] = self.Init_Pos_FE 
        config['PositionLimit_FE'] = self.PositionLimit_FE
        config['PositionMinLimit_FE'] = self.PositionMinLimit_FE
        config['PositionLimit_AA'] = self.PositionLimit_AA
        config['PositionMinLimit_AA'] = self.PositionMinLimit_AA
        config['CurrentLimit_FE'] = self.CurrentLimit_FE
        config['CurrentLimit_AA'] = self.CurrentLimit_AA
        config['Gain_position_FE'] = self.Gain_position_FE
        config['Gain_position_AA'] = self.Gain_position_AA
        config['Grasp_Target_Cur_AA'] = self.Grasp_Target_Cur_AA
        config['Grasp_Target_Cur_FE'] = self.Grasp_Target_Cur_FE
        config['Grasp_Target_Pos_AA'] = self.Grasp_Target_Pos_AA
        config['Grasp_Target_Pos_FE'] = self.Grasp_Target_Pos_FE
        
        current_time = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        filename = f'hand_{current_time}.yaml'
        with open(os.path.join(script_directory, filename), 'w') as file:
            yaml.dump(config, file)
        print(f"Data saved to {filename}")

    ### Slider Setting ###
    def sliderValueChanged(self, slide, label):
        label.setText(str(slide.value()))
    
    def sliderSettingValueChanged(self, slide, label, idx):
        label.setText(str(slide.value()))
        if "AA" in slide.objectName():
            target_value = self.setting_var_AA
        else :
            target_value = self.setting_var_FE
        if idx < 3:
            target_value[0][idx] = slide.value()
        elif idx < 7:
            target_value[1][idx-3] = slide.value()
        elif idx < 11:
            target_value[2][idx-7] = slide.value()
        else :
            target_value[3][idx-11] = slide.value()

    def sliderControlValueChanged(self, slide, label, idx):
        label.setText(str(slide.value()))
        if "AA" in slide.objectName():
            target_value = self.control_var_AA
            mode = self.Grasp_Ctrl_Mode_AA
        else:
            target_value = self.control_var_FE
            mode = self.Grasp_Ctrl_Mode_FE

        target_value[0 if mode == 0 else 1][idx] = slide.value()

    def LimitValueChanged(self, slide1, slide2, idx, mode):
        index = idx - 3
        if mode in ["AA", "FE"]:
            setting_slidebar = self.AA_setting_slidebar if mode == "AA" else self.FE_setting_slidebar
            ctrl_mode = self.Grasp_Ctrl_Mode_AA if mode == "AA" else self.Grasp_Ctrl_Mode_FE

            if ctrl_mode == 0:  # current control
                if index < 4:
                    slide2.setMaximum(slide1.value())
                elif 4 <= index < 8:
                    setting_slidebar[idx+4].setMaximum(slide1.value())
                elif index >= 8:
                    setting_slidebar[idx-4].setMinimum(slide1.value())
            elif ctrl_mode == 1:  # current position control
                if 4 <= index < 8:
                    slide2.setMaximum(slide1.value())
                    setting_slidebar[idx+4].setMaximum(slide1.value())
                elif index >= 8:
                    slide2.setMinimum(slide1.value())
                    setting_slidebar[idx-4].setMinimum(slide1.value())

    def labelValueChanged(self, slide, label):
        text = label.text()
        if text == "":  
            pass
        elif text.isdigit():  
            slide.setValue(int(text))
        else:
            label.setText(str(slide.value()))

    ### Checkbox Setting ###
    def checkBox_ALL_changed(self, allcheckbox, checkbox):
        for cb in checkbox :
            print(allcheckbox.isChecked())
            cb.setCheckState(allcheckbox.checkState())


    ### Init Setting ###
    def Init(self):
        for slide, label, value in zip(self.AA_setting_slidebar, self.AA_setting_slideLabel, np.concatenate([self.Gain_position_AA, self.CurrentLimit_AA, self.PositionLimit_AA])):
            slide.setValue(value)
            label.setText(str(value))
        for slide, label, value in zip(self.FE_setting_slidebar, self.FE_setting_slideLabel, np.concatenate([self.Gain_position_FE, self.CurrentLimit_FE, self.PositionLimit_FE])):
            slide.setValue(value)
            label.setText(str(value))
        for slide, label, max1, max2, min, value1, value2 in zip(self.AA_control_slidebar, self.AA_control_slideLabel, self.CurrentLimit_AA, self.PositionLimit_AA, self.PositionMinLimit_AA, self.Grasp_Target_Cur_AA, self.Grasp_Target_Pos_AA) :
            if self.Grasp_Ctrl_Mode_AA == 0 :
                slide.setMinimum(0)
                slide.setMaximum(max1)
                slide.setValue(value1)
                label.setText(str(value1))
                self.label_AA_grasp_mode.setText("Grasp Control Mode : Current")
                self.label_AA_grasp_value.setText("Value : Goal Current")
            elif self.Grasp_Ctrl_Mode_AA == 1 :
                slide.setMinimum(min)
                slide.setMaximum(max2)
                slide.setValue(value2)
                label.setText(str(value2))
                self.label_AA_grasp_mode.setText("Grasp Control Mode : Current Position")
                self.label_AA_grasp_value.setText("Value : Goal Position")
        for slide, label, max1, max2, min, value1, value2 in zip(self.FE_control_slidebar, self.FE_control_slideLabel ,self.CurrentLimit_FE, self.PositionLimit_FE, self.PositionMinLimit_FE, self.Grasp_Target_Cur_AA, self.Grasp_Target_Pos_AA) :
            if self.Grasp_Ctrl_Mode_FE == 0 :
                slide.setMinimum(0)
                slide.setMaximum(max1)
                slide.setValue(value1)
                label.setText(str(value1))
                self.label_FE_grasp_mode.setText("Grasp Control Mode : Current")
                self.label_FE_grasp_value.setText("Value : Goal Current")
            elif self.Grasp_Ctrl_Mode_FE == 1 :
                slide.setMinimum(min)
                slide.setMaximum(max2)
                slide.setValue(value2)
                label.setText(str(value2))
                self.label_FE_grasp_mode.setText("Grasp Control Mode : Current Position")
                self.label_FE_grasp_value.setText("Value : Goal Position")

        # self.btn_AA_grasp_current_control.setEnabled(False)
        # self.btn_AA_grasp_current_position_control.setEnabled(True)
        # self.btn_FE_grasp_current_control.setEnabled(False)
        # self.btn_FE_grasp_current_position_control.setEnabled(True)

    def init_ros(self):
        rospy.init_node('tocabi_hand_ui', anonymous=True)
        self.current_pub = rospy.Publisher('/hand/r/current', Int16MultiArray, queue_size = 1)
        self.position_pub = rospy.Publisher('/hand/r/position', Int16MultiArray, queue_size = 1)
        self.act_pub = rospy.Publisher('/tocabi/terminate', Bool, queue_size = 1)

    def testCB(self, msg):
        print(msg)

    def heartbeat(self):
        # print('heart beat')
        #dxl_comm_result = self.groupSyncRead_position.txRxPacket()
        #print(self.groupSyncRead_current.last_errors)
        for i in DXL_ID:
            try:
                if self.groupSyncRead_current.last_errors[i] != 0:
                    print('[ID:%03d] !!!! HARDWARE ERROR !!!!!!' % i)
                    self.recovery(i)
            except:
                pass

    def recovery(self, id):
        dxl_comm_result, dxl_error = self.packetHandler.reboot(self.portHandler, id)
        rospy.sleep(0.1)
        self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
        #if dxl_comm_result != 0:
        #    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        #elif dxl_error != 0:
        #    print("%s" % self.packetHandler.getRxPacketError(dxl_error))

        #print("[ID:%03d] reboot Succeeded\n" % id)
        #print("[ID:%03d] torque on\n" % id)
        sys.stdout.flush()

    def __init_comm(self) :
        self.portHandler = dxl.PortHandler(DEVICENAME)
        self.packetHandler = dxl.PacketHandler(PROTOCOL_VERSION)
        self.groupSyncWrite_position = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_GOAL_POSITION, LEN_GOAL_POSITION)
        self.groupSyncRead_position = dxl.GroupSyncRead(self.portHandler, self.packetHandler, ADDR_XL330_PRESENT_POSITION, LEN_PRESENT_POSITION)
        self.groupSyncWrite_current = dxl.GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_XL330_GOAL_CURRENT, LEN_GOAL_CURRENT)
        self.groupSyncRead_current = dxl.GroupSyncRead(self.portHandler, self.packetHandler, ADDR_XL330_PRESENT_CURRENT, LEN_PRESENT_CURRENT)

        for i in DXL_ID	 :
            self.groupSyncRead_position.addParam(i)
        for i in DXL_ID_AA :
            self.groupSyncRead_current.addParam(i)
        for i in DXL_ID_FE :
            self.groupSyncRead_current.addParam(i)

        # Open port
        try: self.portHandler.clearPort()
        except: pass
        try: self.portHandler.closePort()
        except: pass
        if self.portHandler.openPort(): print("Succeeded to open the port")
        else: print("Failed to open the port")
        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE): print("Succeeded to change the baudrate")
        else: print("Failed to change the baudrate")

    def init_dxl(self):
        # Reboot all Dxl
        for i in DXL_ID :
            self.packetHandler.reboot(self.portHandler, i)

		# ALL joint Torque off
        for i in DXL_ID	:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE, TORQUE_DISABLE)
			
		# Change Operating mode
        for i in DXL_ID_AA :
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_OPERATING_MODE, CURRENT_POSITION_CONTROL_MODE)
			
        for i in DXL_ID_FE :
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_OPERATING_MODE, CURRENT_CONTROL_MODE)

        # Set Current Limit
        for i,l in zip(DXL_ID_FE, self.CurrentLimit_FE) :
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_CURRENT_LIMIT, l)

        for i,l in zip(DXL_ID_AA, self.CurrentLimit_AA) :
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_CURRENT_LIMIT, l)

        # AA joint Torque on and init pos
        for i,pos in zip(DXL_ID_AA, self.Init_Pos_AA):
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE, TORQUE_ENABLE)
            self.packetHandler.write4ByteTxRx(self.portHandler, i, ADDR_XL330_GOAL_POSITION, pos)

        # FE joint Torque on and current init
        for i,cur in zip(DXL_ID_FE, self.Init_Cur_FE):
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_DRIVING_MODE , 1) # CW
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE, TORQUE_ENABLE)
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_GOAL_CURRENT , cur)

        rospy.sleep(3.0)

        for i in range(4) :
            self.Init_Pos_FE[i] = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID_FE[i], ADDR_XL330_PRESENT_POSITION)[0]
		
        for i in DXL_ID_FE:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_DISABLE)
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_OPERATING_MODE, CURRENT_CONTROL_MODE)
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
            # self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_GOAL_CURRENT , 80)

        # self.torque_off()
        # FE joint Torque off and Change Operating Mode
        # for i in DXL_ID_FE:
        #     self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_DISABLE)
        #     self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_OPERATING_MODE, CURRENT_POSITION_CONTROL_MODE)
        #     self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)

        # Set PID gains of Controller
        for i in DXL_ID_FE :
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_D_GAIN_POSITION, self.Gain_position_FE[0])
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_I_GAIN_POSITION, self.Gain_position_FE[1])
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_P_GAIN_POSITION, self.Gain_position_FE[2])

        for i in DXL_ID_AA :
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_D_GAIN_POSITION, self.Gain_position_AA[0])
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_I_GAIN_POSITION, self.Gain_position_AA[1]) # No I gain to AA joints
            self.packetHandler.write2ByteTxRx(self.portHandler, i, ADDR_XL330_P_GAIN_POSITION, self.Gain_position_AA[2])

    def read_joint_position(self) :
        dxl_comm_result = self.groupSyncRead_position.txRxPacket()
        for i in range(4) :
            self.joint_positions[i] = self.groupSyncRead_position.getData(DXL_ID_AA[i], ADDR_XL330_PRESENT_POSITION, LEN_PRESENT_POSITION)
            self.AA_live_pos[i].setText(str(int(self.joint_positions[i])))
        for i in range(4) :
            self.joint_positions[i+4] = self.groupSyncRead_position.getData(DXL_ID_FE[i], ADDR_XL330_PRESENT_POSITION, LEN_PRESENT_POSITION)
            self.FE_live_pos[i].setText(str(int(self.joint_positions[i+4])))
        msg= Int16MultiArray()
        msg.data = self.joint_positions.tolist()
        self.position_pub.publish(msg)

    def read_joint_current(self) :
        dxl_current_result = self.groupSyncRead_current.txRxPacket()
        for i in range(4) :
            self.joint_currents[i] = np.array(self.groupSyncRead_current.getData(DXL_ID_AA[i], ADDR_XL330_PRESENT_CURRENT, LEN_PRESENT_CURRENT)).astype(np.int16)
            self.AA_live_cur[i].setText(str(int(self.joint_currents[i])))
        for i in range(4) :
            self.joint_currents[i+4] = np.array(self.groupSyncRead_current.getData(DXL_ID_FE[i], ADDR_XL330_PRESENT_CURRENT, LEN_PRESENT_CURRENT)).astype(np.int16)
            self.FE_live_cur[i].setText(str(int(self.joint_currents[i+4])))
        msg= Int16MultiArray()
        msg.data = self.joint_currents.tolist()
        self.current_pub.publish(msg)

    def torque_off(self) :
        # ALL joint Torque off
        for i in DXL_ID	:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_DISABLE)
        print('Torque OFF!')
    
    def torque_on(self) :
        # ALL joint Torque off
        for i in DXL_ID	:
            self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
        print('Torque ON!')
    
    def grasp(self):
        # 모드 변경 추가하기
        self.groupSyncWrite_current.clearParam()
        self.groupSyncWrite_position.clearParam()
        for i in range(4):
            if self.AA_checkbox[i].isChecked():
                if self.Grasp_Ctrl_Mode_AA == 0:     # current mode
                    self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_AA[i], ADDR_XL330_OPERATING_MODE, CURRENT_CONTROL_MODE)
                    self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_AA[i], ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
                    param_goal_current = [dxl.DXL_LOBYTE(dxl.DXL_LOWORD(self.Grasp_Target_Cur_AA[i])), dxl.DXL_HIBYTE(dxl.DXL_LOWORD(self.Grasp_Target_Cur_AA[i]))]
                    self.groupSyncWrite_current.addParam(DXL_ID_AA[i], param_goal_current)
                elif self.Grasp_Ctrl_Mode_AA == 1:   # current+position mode
                    self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_AA[i], ADDR_XL330_OPERATING_MODE, CURRENT_POSITION_CONTROL_MODE)
                    self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_AA[i], ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
                    param_goal_position = [dxl.DXL_LOBYTE(dxl.DXL_LOWORD(self.Grasp_Target_Pos_AA[i])), dxl.DXL_HIBYTE(dxl.DXL_LOWORD(self.Grasp_Target_Pos_AA[i])), dxl.DXL_LOBYTE(dxl.DXL_HIWORD(self.Grasp_Target_Pos_AA[i])), dxl.DXL_HIBYTE(dxl.DXL_HIWORD(self.Grasp_Target_Pos_AA[i]))]
                    self.groupSyncWrite_position.addParam(DXL_ID_AA[i], param_goal_position)

            if self.FE_checkbox[i].isChecked():
                if self.Grasp_Ctrl_Mode_FE == 0:     # current mode
                    self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_FE[i], ADDR_XL330_OPERATING_MODE, CURRENT_CONTROL_MODE)
                    self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_FE[i], ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
                    param_goal_current = [dxl.DXL_LOBYTE(dxl.DXL_LOWORD(self.Grasp_Target_Cur_FE[i])), dxl.DXL_HIBYTE(dxl.DXL_LOWORD(self.Grasp_Target_Cur_FE[i]))]
                    self.groupSyncWrite_current.addParam(DXL_ID_FE[i], param_goal_current)        
                elif self.Grasp_Ctrl_Mode_FE == 1:   # current+position mode
                    self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_FE[i], ADDR_XL330_OPERATING_MODE, CURRENT_POSITION_CONTROL_MODE)
                    self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_FE[i], ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
                    param_goal_position = [dxl.DXL_LOBYTE(dxl.DXL_LOWORD(self.Grasp_Target_Pos_FE[i])), dxl.DXL_HIBYTE(dxl.DXL_LOWORD(self.Grasp_Target_Pos_FE[i])), dxl.DXL_LOBYTE(dxl.DXL_HIWORD(self.Grasp_Target_Pos_FE[i])), dxl.DXL_HIBYTE(dxl.DXL_HIWORD(self.Grasp_Target_Pos_FE[i]))]
                    self.groupSyncWrite_position.addParam(DXL_ID_FE[i], param_goal_position)    

        dxl_comm_result_cur = self.groupSyncWrite_current.txPacket()
        dxl_comm_result_pos = self.groupSyncWrite_position.txPacket()

    def stretch(self):
        self.groupSyncWrite_current.clearParam()
        self.groupSyncWrite_position.clearParam()
        for i in range(4):
            self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_AA[i], ADDR_XL330_OPERATING_MODE, CURRENT_POSITION_CONTROL_MODE)
            self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_AA[i], ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
            param_goal_position = [dxl.DXL_LOBYTE(dxl.DXL_LOWORD(self.Init_Pos_AA[i])), dxl.DXL_HIBYTE(dxl.DXL_LOWORD(self.Init_Pos_AA[i])), dxl.DXL_LOBYTE(dxl.DXL_HIWORD(self.Init_Pos_AA[i])), dxl.DXL_HIBYTE(dxl.DXL_HIWORD(self.Init_Pos_AA[i]))]
            self.groupSyncWrite_position.addParam(DXL_ID_AA[i], param_goal_position)
            self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_FE[i], ADDR_XL330_OPERATING_MODE, CURRENT_CONTROL_MODE)
            self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID_FE[i], ADDR_XL330_TORQUE_ENABLE , TORQUE_ENABLE)
            param_goal_current = [dxl.DXL_LOBYTE(dxl.DXL_LOWORD(self.Init_Cur_FE[i])), dxl.DXL_HIBYTE(dxl.DXL_LOWORD(self.Init_Cur_FE[i]))]
            self.groupSyncWrite_current.addParam(DXL_ID_FE[i], param_goal_current)

        dxl_comm_result_cur = self.groupSyncWrite_current.txPacket()
        dxl_comm_result_pos = self.groupSyncWrite_position.txPacket()

if __name__ == "__main__" :
    app = QApplication(sys.argv)
    myWindow = HandCtrlGUI()
    myWindow.show()
    app.exec_()
    
    # shutdown
    print("Shut Down!!")
    myWindow.torque_off()
