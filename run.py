#!/usr/bin/python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1
from ev3dev2.sensor.lego import UltrasonicSensor
from time import sleep, time

class Balance:
    def __init__(self, motor, sensor, midPoint):
        self.motor = LargeMotor(motor)
        self.sensor = UltrasonicSensor(sensor)
        self.midPoint = midPoint
        self.motorPositionUp = self.motor.position - 20
        self.motorPositionDown = self.motor.position + 20
        self.integral = 0
        self.integralCount = 0

    def getSensorState(self):
        """
        Returns sensor measure based on an axis positioned on 
        the mid point
        """
        return -1*(self.sensor.distance_centimeters-self.midPoint)

    def rotateAxis(self, speed):
        """
        Rotates axis on the tacho motor by speed value
        """
        # Limits rotation to limit object speed
        if self.motor.position >= self.motorPositionDown and speed > 0:
            self.motor.stop(stop_action="brake")
        elif self.motor.position <= self.motorPositionUp and speed < 0:
            self.motor.stop(stop_action="brake")
        else:
            #print(speed)
            self.motor.on(speed, block=False)


    def calculateSpeed(self, Kp=1, Kd=0, Ki=0, verbose=False):
        """
        Calculates speed in next cicle based on
        a PID control
        """
        initialMeasure = self.getSensorState()
        sleep(0.015)
        finalMeasure = self.getSensorState()

        # Fix closeness to the sensor
        if finalMeasure < -30:
            finalMeasure = 8.4

        speed = finalMeasure - initialMeasure
        
        if  -3 < finalMeasure < 3 and (self.integral <= 10 and self.integral >= -10 and self.integralCount < 10):
            self.integral=self.integral+(finalMeasure)
            self.integralCount = self.integralCount+1
        else:
            self.integral = 0
            self.integralCount = 0
        
        value = (Kp*finalMeasure)+(Kd*speed)+self.integral*Ki
        
        # Fix unbalanced values
        if value > 100:
            value = 100.0
        if value < -100:
            value = -100.0
        
        #print("\n-Distancia: %f, Velocidad %f, Integracion %f" % (finalMeasure, speed, self.integral))
        return value/3.2


def run(balance):
    """
    Work state loop of the system. It's needed to interrupt execution
    to stop the process.
    """
    while True:
        #start = time()
        balance.rotateAxis(balance.calculateSpeed(Kp=2.3, Kd=80.1, Ki=2))
        #elapsed= time()-start
        #print("Tiempo: %f" % elapsed)

def main():
    input("Coloca el motor con un angulo de 90 grados y pulsa enter")
    balance = Balance(motor=OUTPUT_A, sensor=INPUT_1, midPoint=0)   
    input("Coloca el coche en el centro y pulsa una tecla")
    while True:
        value = -1*balance.getSensorState()
        answer = input("El valor obtenido es %f, continuar?" % value)
        if answer == "y":
            break
    balance.midPoint = value
    input("Coloca la pelota y pulsa para empezar")
    run(balance)

if __name__ == "__main__":
    main()
