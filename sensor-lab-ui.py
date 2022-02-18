from PyQt5 import QtWidgets, QtCore, QtSerialPort 
from PyQt5.QtWidgets import (QWidget, QPushButton,
        QFrame, QApplication)
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
import os
from random import randint
# from ui_form import Ui_Widget
from mainwindow import Ui_MainWindow
import re # to remove gibberish letter or symbol bytes in serial port data 
import time 

class MainWindow(QtWidgets.QMainWindow, Ui_MainWindow):
    
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        # setup initial value and GUI 
        self.setupUi(self)
        self.serial_port_addr = "/dev/tty.usbmodem145401"
        self.GUI_mode = 0
        self.ir_value = 0
        self.ultra_value = 0
        self.force_value = 0
        self.stepper_value = 0
        self.dc_value = 0 # unit is degree when dc_mode = 0; unit is rpm when dc_mode = 1 
        self.dc_mode = 0
        self.servo_value = 0
        self.motor_data_str = "G:0,DM:0,DV:0,V:0,S:0\n"
        self.serial_counter = 0

        self.time_axis = list(range(100))  # 100 time points
        self.ir_data = [0]*100  # 100 data points
        self.ultra_data = [0]*100  # 100 data points
        self.force_data = [0]*100  # 100 data points

        self.ir_plotView.setBackground('w')
        self.ultra_plotView.setBackground('w')
        self.force_plotView.setBackground('w')

        pen = pg.mkPen(color=(255, 0, 0))
        self.ir_data_line =  self.ir_plotView.plot(self.time_axis, self.ir_data, pen=pen)
        self.ultra_data_line =  self.ultra_plotView.plot(self.time_axis, self.ultra_data, pen=pen)
        self.force_data_line =  self.force_plotView.plot(self.time_axis, self.force_data, pen=pen)

        # self.disableMotorWidgets()

        self.tabWidget.setCurrentIndex(0)
        self.stepper_dial.setRange(0, 200)
        self.dc_dgr_dial.setRange(0, 360)
        self.dc_rpm_dial.setRange(90, 140)
        self.dc_rpm_lcd.display(90)
        self.servo_dial.setRange(0, 180)
        
        self.stepper_progressBar.setMinimum(0)
        self.stepper_progressBar.setValue(0)
        self.stepper_progressBar.setMaximum(200)

        self.dc_dgr_progressBar.setMinimum(0)
        self.dc_dgr_progressBar.setValue(0)
        self.dc_dgr_progressBar.setMaximum(360)

        self.dc_rpm_progressBar.setMinimum(90)
        self.dc_rpm_progressBar.setValue(90)
        self.dc_rpm_progressBar.setMaximum(140)

        self.servo_progressBar.setMinimum(0)
        self.servo_progressBar.setValue(0)
        self.servo_progressBar.setMaximum(180)
        
        self.stepper_dial.valueChanged.connect(self.stepperDial_changed)
        self.stepper_dial.sliderReleased.connect(self.stepperDial_updated)

        self.dc_dgr_dial.valueChanged.connect(self.dcDgrDial_changed)
        self.dc_dgr_dial.sliderReleased.connect(self.dcDgrDial_updated)

        self.dc_rpm_dial.valueChanged.connect(self.dcRpmDial_changed)
        self.dc_rpm_dial.sliderReleased.connect(self.dcRpmDial_updated)

        self.servo_dial.valueChanged.connect(self.servoDial_changed)
        self.servo_dial.sliderReleased.connect(self.servoDial_updated)

        self.serial = QtSerialPort.QSerialPort(
            self.serial_port_addr,
            baudRate = QtSerialPort.QSerialPort.Baud9600,     
            dataBits = QtSerialPort.QSerialPort.Data8,
            readyRead = self.receive
        )
        self.serial.open(QtCore.QIODevice.ReadWrite)

        self.read_timer = QtCore.QTimer()
        self.read_timer.setInterval(50)
        self.read_timer.timeout.connect(self.read_sensor_data)
        self.read_timer.start()

        self.write_timer = QtCore.QTimer()
        self.write_timer.setInterval(1000)
        self.write_timer.timeout.connect(self.write_motor_data)
        self.write_timer.start()

    def stepperDial_changed(self):
        dial_value = self.stepper_dial.value()
        self.stepper_lcd.display(str(dial_value))
        self.stepper_progressBar.setValue(dial_value)

    def stepperDial_updated(self):
        self.stepper_value = self.stepper_dial.value()

    def dcDgrDial_changed(self):
        dial_value = self.dc_dgr_dial.value()
        self.dc_dgr_lcd.display(str(dial_value))
        self.dc_dgr_progressBar.setValue(dial_value)

    def dcDgrDial_updated(self):
        self.dc_mode = 0
        self.dc_value = self.dc_dgr_dial.value()
        # print(self.dc_value)

    def dcRpmDial_changed(self):
        dial_value = self.dc_rpm_dial.value()
        self.dc_rpm_lcd.display(str(dial_value))
        self.dc_rpm_progressBar.setValue(dial_value)

    def dcRpmDial_updated(self):
        self.dc_mode = 1
        self.dc_value = self.dc_rpm_dial.value()
        # print(self.dc_value)

    def servoDial_changed(self):
        dial_value = self.servo_dial.value()
        self.servo_lcd.display(str(dial_value))
        self.servo_progressBar.setValue(dial_value)

    def servoDial_updated(self):
        self.servo_value = self.servo_dial.value()

    def disableMotorWidgets(self):
        self.stepperWidget.setEnabled(False)
        self.dcWidget_dgr.setEnabled(False)
        self.dcWidget_rpm.setEnabled(False)
        self.servoWidget.setEnabled(False)

    def enableMotorWidgets(self):
        self.stepperWidget.setEnabled(True)
        self.dcWidget_dgr.setEnabled(True)
        self.dcWidget_rpm.setEnabled(True)
        self.servoWidget.setEnabled(True)

    def disableSensorWidgets(self):
        self.irWidget.setEnabled(False)
        self.ultrasonicWidget.setEnabled(False)
        self.forceWidget.setEnabled(False)

    def enableSensorWidgets(self):
        self.irWidget.setEnabled(True)
        self.ultrasonicWidget.setEnabled(True)
        self.forceWidget.setEnabled(True)


    @QtCore.pyqtSlot()
    def receive(self):
        # print("received")
        while self.serial.canReadLine():
            if self.serial_counter == 0:
                time.sleep(1)
                self.serial_counter += 1
            ino_data = self.serial.readLine().data().decode()
            # print(ino_data)
            ino_text = ino_data.rstrip('\r\n')
            ino_data_array = re.findall(r"[SIUF]:(\d+)", ino_text)
            if len(ino_data_array) == 4:
                # print(ino_data_array)
                self.ultra_value = int(ino_data_array[2])
                self.ir_value = int(ino_data_array[1])
                self.force_value = int(ino_data_array[3])
                # self.stepper_value = int(ino_data_array[4])
                # self.dc_value = int(ino_data_array[5])
                # self.servo_value = int(ino_data_array[6])

    @QtCore.pyqtSlot()
    def write_motor_data(self):
        print("hello")
        self.motor_data_str = "G:%d,DM:%d,DV:%d,V:%d,S:%d\n" % (self.GUI_mode, self.dc_mode, self.dc_value, self.servo_value, self.stepper_value)
        print(self.motor_data_str)
        self.write_success = self.serial.write(self.motor_data_str.encode())
        print("write succeed: " + str(self.write_success))

    def read_sensor_data(self):
        # sensor_read_mode = "G:0,DM:0,DV:0,V:0,S:0\n"
        # self.serial.write(sensor_read_mode.encode())

        self.ultra_lcd.display(str(self.ultra_value))
        self.ir_lcd.display(str(self.ir_value))
        self.force_lcd.display(str(self.force_value))

        self.time_axis = self.time_axis[1:]  # Remove the first y element.
        self.time_axis.append(self.time_axis[-1] + 1)  # Add a new value 1 higher than the last.

        self.ultra_data = self.ultra_data[1:]  # Remove the first
        self.ultra_data.append(self.ultra_value)  # Add a new random value.
        self.ultra_data_line.setData(self.time_axis, self.ultra_data)  # Update the data.
        
        self.ir_data = self.ir_data[1:]  # Remove the first
        self.ir_data.append(self.ir_value)  # Add a new random value.
        self.ir_data_line.setData(self.time_axis, self.ir_data)  # Update the data.

        self.force_data = self.force_data[1:]  # Remove the first
        self.force_data.append(self.force_value)  # Add a new random value.
        self.force_data_line.setData(self.time_axis, self.force_data)  # Update the data.

        self.stepper_lcd.display(self.stepper_value)
        self.servo_lcd.display(self.servo_value)
        self.dc_dgr_lcd.display(self.dc_value)

        self.stepper_progressBar.setValue(self.stepper_value)
        self.servo_progressBar.setValue(self.servo_value)
        self.dc_dgr_progressBar.setValue(self.dc_value)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())