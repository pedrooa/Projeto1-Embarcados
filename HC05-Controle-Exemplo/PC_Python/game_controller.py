import serial
import argparse
import time
import logging
import pyvjoy # Windows apenas

class MyControllerMap:
    def __init__(self):
        self.button = {'Start': 1,'A': 2,'B':3}
        self.joystick = {'Left':4,'Right':5,'Up':6,'Down':7}
        


class SerialControllerInterface:

    # Protocolo
    # byte 1 -> Botão 1 (estado - Apertado 1 ou não 0)
    # byte 2 -> EOP - End of Packet -> valor reservado 'X'

    def __init__(self, port, baudrate):
        self.ser = serial.Serial(port, baudrate=baudrate)
        self.mapping = MyControllerMap()
        self.j = pyvjoy.VJoyDevice(1)
        self.incoming = '0'




    def update(self):
        ## Sync protocol
        while self.incoming != b'X':
            self.incoming = self.ser.read()
            logging.debug("Received INCOMING: {}".format(self.incoming))




        # for k,v in self.mapping.button.items() :
	       #  data = self.ser.read()
	       #  logging.debug("Received DATA: {}".format(data))
	       #  if data == b'1':
	       #      logging.info("Sending press")
	       #      self.j.set_button(v, 1)
	       #  elif data == b'0':
	       #      self.j.set_button(v, 0)

        data = self.ser.read()
        logging.debug("Received DATA: {}".format(data))
        if data == b'1':
            logging.info("Sending press")
            self.j.set_button(self.mapping.button['Start'], 1)
        elif data == b'0':
            self.j.set_button(self.mapping.button['Start'], 0)

        data = self.ser.read()
        data = self.ser.read()


        data = self.ser.read()
        if data == b'2':
            logging.info("Sending press")
            self.j.set_button(self.mapping.joystick['Left'], 1)
        elif data == b'3':
            logging.info("Sending press")
            self.j.set_button(self.mapping.joystick['Right'], 1)
        elif data == b'0':
            self.j.set_button(self.mapping.joystick['Right'], 0)
            self.j.set_button(self.mapping.joystick['Left'], 0)


        data = self.ser.read()
        if data == b'5':
            logging.info("Sending press")
            self.j.set_button(self.mapping.joystick['Up'], 1)
        elif data == b'4':
            logging.info("Sending press")
            self.j.set_button(self.mapping.joystick['Down'], 1)
        elif data == b'0':
            self.j.set_button(self.mapping.joystick['Up'], 0)
            self.j.set_button(self.mapping.joystick['Down'], 0)


        # data = self.ser.read()
        # self.j.set_axis(pyvjoy.HID_USAGE_X,data);


        # data = self.ser.read()
        # logging.debug("Received DATA: {}".format(data))

        # if data == b'3':
        #     logging.info("Sending press")
        #     self.j.set_button(self.mapping.button['B'], 1)
        # elif data == b'0':
        #     self.j.set_button(self.mapping.button['B'], 0)



        self.incoming = self.ser.read()


class DummyControllerInterface:
    def __init__(self):
        self.mapping = MyControllerMap()
        self.j = pyvjoy.VJoyDevice(1)

    def update(self):
        self.j.set_button(self.mapping.button['A'], 1)
        time.sleep(0.1)
        self.j.set_button(self.mapping.button['A'], 0)
        logging.info("[Dummy] Pressed A button")
        time.sleep(1)


if __name__ == '__main__':
    interfaces = ['dummy', 'serial']
    argparse = argparse.ArgumentParser()
    argparse.add_argument('serial_port', type=str)
    argparse.add_argument('-b', '--baudrate', type=int, default=9600)
    argparse.add_argument('-c', '--controller_interface', type=str, default='serial', choices=interfaces)
    argparse.add_argument('-d', '--debug', default=False, action='store_true')
    args = argparse.parse_args()
    if args.debug:
        logging.basicConfig(level=logging.DEBUG)

    print("Connection to {} using {} interface ({})".format(args.serial_port, args.controller_interface, args.baudrate))
    if args.controller_interface == 'dummy':
        controller = DummyControllerInterface()
    else:
        controller = SerialControllerInterface(port=args.serial_port, baudrate=args.baudrate)

    while True:
        controller.update()
