#written and tested for python3

#####################################################################################################
# Filename      :   sowiso_ctrl_v0-2.py
# Description   :   Automating the SoWiSo
# Author        :   Zinzen
# modification  :   2021/11/21
#####################################################################################################
#####################################################################################################

#####################################################################################################
#====================================================================================================
#==============================                                        ==============================
#==============================            IMPORT LIBRARIES            ==============================
#==============================                                        ==============================
#====================================================================================================
#####################################################################################################
import RPi.GPIO as GPIO
import time
import datetime
from ADCDevice import *

#####################################################################################################
#====================================================================================================
#==============================                                        ==============================
#==============================     DEFINITION OF GLOBAL VARIABLES     ==============================
#==============================                                        ==============================
#====================================================================================================
#####################################################################################################

###YAW MOTOR PINS
    # NOTE: the turn motor is a simple brushed DC motor
        # it is operated via a L293D MCC (motor control chip), which has 4 input/output channels on 2 drivers
        # yaw uses channel 2 --> pins 3/4
        # turn direction is determined by GPIO pins on inputs 3, 4
        # toggling HI vs LOW on input toggles supply voltage direction on output pin.  
        # IF both pins are LOW or HIGH ==> no voltage b/w pins => no movement
yaw_enable_pin = 32         # this pin is enabling ch3/4 driver = GPIO.12
yaw_motor_in1_pin = 29      # this pin is for motor polarity    = GPIO.5
yaw_motor_in2_pin = 31      # this pin is for motor polarity    = GPIO.6

### ACTUATOR PINS
    # NOTE: the linear actuator operates like a regular DC motor with integrated limit switches 
        # it is operated via a L293D MCC (motor control chip), which has 4 input/output channels on 2 drivers 
        # pitch uses channel 1 --> pins 1/2
        # pitch direction is determined by GPIO pins on inputs 3, 4
        # toggling HI vs LOW on input toggles supply voltage direction on output pin.  
        # IF both pins are LOW or HIGH ==> no voltage b/w pins => no movement
pitch_enable_pin = 36       # this pin is enabling ch1/2 driver = GPIO.16
pitch_motor_in1_pin = 40    # this pin is for motor polarity    = GPIO.21
pitch_motor_in2_pin = 38    # this pin is for motor polarity    = GPIO.20

### SENSOR PINS
yaw_sensor_pwr_pin = 16         # this pin will be toggled HIGH if sensing for reed switches         = GPIO.23
yaw_sensor_monitor_pin = 18     # this pin will monitor for voltage drop the reed switch would cause = GPIO.24

pitch_sensor_pwr_pin = 10       # this pin will be toggled HIGH if sensing for reed switches         = GPIO.15
pitch_sensor_monitor_pin = 12   # this pin will monitor for voltage drop the reed switch would cause = GPIO.18

transistor_pin = 8              # pin to adress transistor array: if LOW => MOSFET=pass-through     = GPIO.14

### RGB PINS
red_pin = 22                    # this pin will be toggled LOW to make the LED red                  = GPIO.25
green_pin = 26                  # this pin will be toggled LOW to make the LED green                = GPIO.7
blue_pin = 24                   # this pin will be toggled LOW to make the LED blue                 = GPIO.8

### ADC variables
adc = ADCDevice()               # Define an ADCDevice class object
v_ref = 2.5                     # sets conversion factor at ADC level 255 to compute voltage  ADC. Internal Ref is 2.5V or 3.3V

###global variables

blink_on = 0.3                  # sets time an RGB light remains on when blinking
blink_off = 0.15                # sets time an RGB light remains off when blinking

resting_interval = 60*2         #sets the time in between optimizations

yaw_pos = 0                     # parameter to keep track of yaw position
yaw_pos_in_deg = 0
pitch_pos = 0                   # parameter to keep track of pitch position
pitch_pos_in_deg = 0
global DC
DC = 100
global move_time
move_time = 0
global degree_yaw_turn_time
degree_yaw_turn_time = 0.1
global degree_ext_time
degree_ext_time = 0.34
global degree_retract_time
degree_retract_time = 0.256
global ext_retr_conversion
ext_retr_conversion = 1.328
global optimization_interval
optimization_interval = 60*60

####################################################################################################
#====================================================================================================
#==============================                                        ==============================
#==============================       DEFINING MODULAR FUNCTIONS       ==============================
#==============================                                        ==============================
#====================================================================================================
#####################################################################################################
def setup():    # inial setup: checks for adc, configures GPIO pins, creates PWMs, Defines several common parameters
    print('\n','\n','\n','[setup() function has been called -- now running basic set-up] ...)','\n')

    ### INITIAL GPIO SET-UP
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)                    # use physical GPIO numbering
    
    ## MOTORS
    GPIO.setup(yaw_motor_in1_pin, GPIO.OUT)     # set pin to OUTPUT mode
    GPIO.output(yaw_motor_in1_pin, GPIO.LOW)    # set initial pin output to LOW
    GPIO.setup(yaw_motor_in2_pin, GPIO.OUT)     # set pin to OUTPUT mode
    GPIO.output(yaw_motor_in2_pin, GPIO.LOW)    # set initial pin output to LOW
    GPIO.setup(yaw_enable_pin, GPIO.OUT)        # set pin to OUTPUT mode
    GPIO.output(yaw_enable_pin, GPIO.LOW)        # set initial pin output to LOW

    global yaw_pwm    
    yaw_pwm = GPIO.PWM(yaw_enable_pin,500)      # create PWM and set Frequence to 500Hz
    yaw_pwm.start(0)                            # initial set PWM DC to 0%

    GPIO.setup(pitch_motor_in1_pin, GPIO.OUT)   # set pin to OUTPUT mode
    GPIO.output(pitch_motor_in1_pin, GPIO.LOW)  # set initial pin output to LOW
    GPIO.setup(pitch_motor_in2_pin, GPIO.OUT)   # set pin to OUTPUT mode
    GPIO.output(pitch_motor_in2_pin, GPIO.LOW)  # set initial pin output to LOW
    GPIO.setup(pitch_enable_pin, GPIO.OUT)      # set pin to OUTPUT mode
    GPIO.output(pitch_enable_pin, GPIO.LOW)      # set initial pin output to LOW

    global pitch_pwm    
    pitch_pwm = GPIO.PWM(pitch_enable_pin,500)  # create PWM and set Frequence to 500Hz
    pitch_pwm .start(0)                         # initial set PWM DC to 0%

    ## SENSORS
    GPIO.setup(yaw_sensor_pwr_pin, GPIO.OUT)    # set pin to OUTPUT mode
    GPIO.output(yaw_sensor_pwr_pin, GPIO.LOW)   # set initial pin output to LOW
    GPIO.setup(yaw_sensor_monitor_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # set sensor pin to PULL UP INPUT mode
    
    GPIO.setup(pitch_sensor_pwr_pin, GPIO.OUT)  # set pin to OUTPUT mode
    GPIO.output(pitch_sensor_pwr_pin, GPIO.LOW) # set initial pin output to LOW
    GPIO.setup(pitch_sensor_monitor_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP) # set sensor pin to PULL UP INPUT mode

    GPIO.setup(transistor_pin, GPIO.OUT)        # set pin to OUTPUT mode
    GPIO.output(transistor_pin, GPIO.LOW)       # set initial pin output to LOW

    ## RGB
    GPIO.setup(red_pin, GPIO.OUT)               # set pin to OUTPUT mode
    GPIO.output(red_pin, GPIO.LOW)              # set initial pin output to LOW
    GPIO.setup(green_pin, GPIO.OUT)             # set pin to OUTPUT mode
    GPIO.output(green_pin, GPIO.LOW)            # set initial pin output to LOW
    GPIO.setup(blue_pin, GPIO.OUT)              # set pin to OUTPUT mode
    GPIO.output(blue_pin, GPIO.LOW)             # set initial pin output to LOW

    ### ADC SET-UP
        #(ADC = analog-digital converter)
        #NOTE: we are using the ADC 'ADS7830', which is an SI-bus enabled ADC. 
        #The ADS7830 has 8-bit resolution, which means it has 2^8 = 256 levels
    global adc  #makes the adc variable available globally
    if(adc.detectI2C(0x4b)):        # Detect the ADS7830 (ADC)
        adc = ADS7830()  
        print('The correct ADC device was connencted:',adc)
    else:
        print('Correct I2C address was not found, \n'
            'Please use command ''i2cdetect -y 1'' to check the I2C address! \n'
            'Program Exit. \n');
        exit(-1)
#====================================================================================================
def rgb(color):

    if (color == 'red'):
        GPIO.output(red_pin,GPIO.HIGH)      
        GPIO.output(green_pin,GPIO.LOW)    
        GPIO.output(blue_pin,GPIO.LOW)     

    elif (color == 'green'):
        GPIO.output(red_pin,GPIO.LOW)    
        GPIO.output(green_pin,GPIO.HIGH)    
        GPIO.output(blue_pin,GPIO.LOW)   

    elif (color == 'blue'):
        GPIO.output(red_pin,GPIO.LOW)      
        GPIO.output(green_pin,GPIO.LOW)    
        GPIO.output(blue_pin,GPIO.HIGH)

    elif (color == 'white'):
        GPIO.output(red_pin,GPIO.HIGH)      
        GPIO.output(green_pin,GPIO.HIGH)    
        GPIO.output(blue_pin,GPIO.HIGH)

    elif (color == 'yellow'):
        GPIO.output(red_pin,GPIO.HIGH)      
        GPIO.output(green_pin,GPIO.HIGH)    
        GPIO.output(blue_pin,GPIO.LOW)

    elif (color == 'cyan'):
        GPIO.output(red_pin,GPIO.LOW)      
        GPIO.output(green_pin,GPIO.HIGH)    
        GPIO.output(blue_pin,GPIO.HIGH)

    elif (color == 'pink'):
        GPIO.output(red_pin,GPIO.HIGH)      
        GPIO.output(green_pin,GPIO.LOW)    
        GPIO.output(blue_pin,GPIO.HIGH)

    elif (color == 'off'):
        GPIO.output(red_pin,GPIO.LOW)      
        GPIO.output(green_pin,GPIO.LOW)    
        GPIO.output(blue_pin,GPIO.LOW)

    else:
        GPIO.output(red_pin,GPIO.LOW)      
        GPIO.output(green_pin,GPIO.LOW)    
        GPIO.output(blue_pin,GPIO.LOW)
        print('wrong color specified')
#====================================================================================================
def rgb_blink(color,times):

    for i in range (0,times,1):
        if (color == 'sequence'):
            rgb('red')
            time.sleep(blink_on)
            rgb('green')
            time.sleep(blink_on)
            rgb('blue')
            time.sleep(blink_on)
            rgb('white')
            time.sleep(blink_on)
        else:
            rgb(color)    
        time.sleep(blink_on)
        rgb('off')
        time.sleep(blink_off)
#====================================================================================================
def start_motor(yaw_or_pitch, direction, DC):
    
    print ('start_motor() function has been called \n'
        '...with arguments -->  Motor     = ',yaw_or_pitch,'\n'
        '                       Direction = ',direction,'\n'
        '                       with DC   = ',DC,'%\n')

    if (yaw_or_pitch == 'yaw'):
        yaw_pwm.ChangeDutyCycle(DC)         # set yaw DC value; activates the MCC yaw channel
        if (direction == 'left'):
            motor_pin1 = yaw_motor_in1_pin 
            motor_pin2 = yaw_motor_in2_pin
        elif (direction == 'right'):
            motor_pin1 = yaw_motor_in2_pin 
            motor_pin2 = yaw_motor_in1_pin
        else:
            print ('wrong direction parameter entered -- direction must exactly be the string >left< or >right<')     
    elif (yaw_or_pitch == 'pitch'):
        pitch_pwm.ChangeDutyCycle(DC)       # set yaw DC value; activates the MCC yaw channel
        if (direction == 'extend'):
            motor_pin1 = pitch_motor_in2_pin
            motor_pin2 = pitch_motor_in1_pin
        elif (direction == 'retract'):
            motor_pin1 = pitch_motor_in1_pin 
            motor_pin2 = pitch_motor_in2_pin
        else:
            print ('wrong direction parameter entered -- direction must exactly be the string >extend< or >retract<')
    else:
        print('wrong motor address entered --> must exactly be the string >yaw< or >pitch<')

    global direction_move
    direction_move = direction
    global yaw_or_pitch_move
    yaw_or_pitch_move = yaw_or_pitch
    global DC_move
    DC_move = DC

    GPIO.output(motor_pin1,GPIO.HIGH)       # motor pin1 output HIGH level
    GPIO.output(motor_pin2,GPIO.LOW)        # motor pin2 output LOW level
    
    print (yaw_or_pitch,' motor has been started with ',DC,'% DC.  MOVEMENT is to ',direction)
#====================================================================================================
def stop_motors():   # stops all motors 
        
    print('\n','\n','\n','[stop motors() function has been called -- now stopping all motors] ...)','\n')
    
    GPIO.output(yaw_motor_in1_pin,GPIO.LOW)         # yaw motor pin1 output LOW level
    GPIO.output(yaw_motor_in2_pin,GPIO.LOW)         # yaw motor pin1 output LOW level
    yaw_pwm.ChangeDutyCycle(0)                      # sets yaw DC value to 0; deactivates the MCC channel

    GPIO.output(pitch_motor_in1_pin,GPIO.LOW)       # pitch motor pin1 output LOW level
    GPIO.output(pitch_motor_in2_pin,GPIO.LOW)       # pitch motor pin1 output LOW level
    pitch_pwm.ChangeDutyCycle(0)                    # sets pitch DC value to 0; deactivates the MCC channel

    print('motors have been stopped')

    time.sleep(1)                                 # settle time befiore reengagement is possible
#====================================================================================================
def run_motor(move_time):   # runs motors for specified time and recalculates yaw or pitch position 
      
    global yaw_pos
    global yaw_pos_in_deg 
    global pitch_pos
    global pitch_pos_in_deg

    print('\n','\n','\n','run_motor() function has been called ...)','\n')

    #the following if loop serves as a safety precaution tp prevent extension past 90°
    if ((direction_move == 'extend') and (pitch_pos_in_deg+(move_time/degree_ext_time) >= 90)):
        move_time = ((90 - pitch_pos_in_deg) * degree_ext_time)
    else:
        pass

    time.sleep(move_time)                   #this is to allow the actual movement

    print('movement complete.')

    #the following if loop redefines yaw position after movement and sets the parameter within the 0° - 360° range
    if (yaw_or_pitch_move == 'yaw'):
        print('old yaw poistion was ',yaw_pos_in_deg)
        if (direction_move == 'right'):
            yaw_pos = yaw_pos + move_time
        elif (direction_move == 'left'):
            yaw_pos = yaw_pos - move_time
        else:
            print('...SOMETHING WENT WRONG !!!!!!!...')
        yaw_pos_in_deg = round(yaw_pos/degree_yaw_turn_time,3)
        while (yaw_pos_in_deg >= 360):
            yaw_pos_in_deg = yaw_pos_in_deg - 360
        while (yaw_pos_in_deg < 0):
            yaw_pos_in_deg = yaw_pos_in_deg + 360
        print('new yaw poistion is ~',yaw_pos_in_deg,'°')

    #the following if loop redefines pitch position after movement and sets the parameter within the 0° - 90° range
    if (yaw_or_pitch_move == 'pitch'):
        print('old pitch poistion was ',pitch_pos_in_deg)
        if (direction_move == 'extend'):
            pitch_pos = pitch_pos + move_time
        elif (direction_move == 'retract'):
            pitch_pos = pitch_pos - move_time*ext_retr_conversion
        else:
            print('...SOMETHING WENT WRONG !!!!!!!...')
        pitch_pos_in_deg = round(pitch_pos/degree_ext_time,3)
        if (pitch_pos_in_deg < 0):
            pitch_pos_in_deg = 0
        elif (pitch_pos_in_deg>90):
            pitch_pos_in_deg = 90
        print('new pitch poistion is ~',pitch_pos_in_deg, '° extension')

    stop_motors()
#====================================================================================================
def sensing_toggle(self):   # simply toggles sensing parameter b/w True and False 

    global sensing
    sensing = not sensing
#====================================================================================================
def yaw_sensor():       # activates sensing for yaw reed switch detection 
        
    print('\n','\n','\n','[yaw_sensor() function has been called -- now listening for reed switch] ...)','\n')

    global sensing

    GPIO.output(yaw_sensor_pwr_pin, GPIO.HIGH)
    time.sleep(0.1)
    sensing=True

    GPIO.add_event_detect(yaw_sensor_monitor_pin,GPIO.FALLING,callback=sensing_toggle,bouncetime=200)
    
    while(sensing==True):
        time.sleep(0.1) 

    GPIO.output(yaw_sensor_pwr_pin, GPIO.LOW)
    GPIO.remove_event_detect(yaw_sensor_monitor_pin)

    print('Reed switch has been activated and detected -- sensor-loop has been exited.')
#====================================================================================================
def pitch_sensor():     # activates sensing for pitch reed switch detection 
        
    print('\n','\n','\n','[pitch_sensor() function has been called -- now listening for reed switch] ...)','\n')

    global sensing

    GPIO.output(pitch_sensor_pwr_pin, GPIO.HIGH)
    time.sleep(0.1)
    sensing=True

    GPIO.add_event_detect(pitch_sensor_monitor_pin,GPIO.FALLING,callback=sensing_toggle,bouncetime=200)

    while(sensing==True):
        time.sleep(0.1) 

    GPIO.output(pitch_sensor_pwr_pin, GPIO.LOW)
    GPIO.remove_event_detect(pitch_sensor_monitor_pin)

    print('Reed switch has been activated and detected -- sensor-loop has been exited.')
#====================================================================================================
def find_yaw_parameters():      # determines time needed for yaw revolution: 'full_yaw_turn_time', 'degree_yaw_turn_time', & zeros 'yaw_pos'
    
    global full_yaw_turn_time
    full_yaw_turn_time = 36
    global degree_yaw_turn_time
    degree_yaw_turn_time = 0.1
    global yaw_pos
    yaw_pos = 0
    global yaw_pos_in_deg
    yaw_pos_in_deg=0
    total_yaw_turn_time = 0
    yaw_iterations = 1

    print('find_yaw_parameters() function has been called...')

    ## Find starting position
    start_motor('yaw','right',100)
    print ('Moving ',yaw_or_pitch_move,' to ',direction_move,' with ',DC_move,'% DC...\n'
        '...for 3 sec. to reach an initial starting position.')
    run_motor(3)
    start_motor('yaw', 'left', 100)
    print ('Moving ',yaw_or_pitch_move,' to ',direction_move,' with ',DC_move,'% DC...\n'
        '...to look for the Zero-position (listening for reed switch).')
    yaw_sensor()
    stop_motors()                                   #stops all motors    
    print('Reed switch sensor found. Now initiating turn revolution time estimation.')    
    time.sleep(1)                                   #short rest interval

    ## Measure turn time
    for i in range(1,yaw_iterations+1,1):
        start = time.time()                             #call and set time
        start_motor('yaw', 'left', 100)
        print ('Moving ',yaw_or_pitch_move,' to ',direction_move,' with ',DC_move,'% DC...')
        time.sleep(2)
        yaw_sensor()                                    #activtate sensor() function
        stop_motors()                                   #stops all motors    
        stop = time.time()                              #call and set time
        yaw_turn_time = round(stop-start,3)    #determine time elapsed and store
        total_yaw_turn_time = total_yaw_turn_time + yaw_turn_time

    full_yaw_turn_time = total_yaw_turn_time/yaw_iterations
    print('full turn took ',full_yaw_turn_time,' seconds')    
    degree_yaw_turn_time = round((full_yaw_turn_time/360),3)        #conversion into seconds per degree yaw turn
    print('\n''estimated turn time per degree yaw is ',degree_yaw_turn_time,'seconds')

    yaw_pos = 0                                     #zeros current yaw position
    yaw_pos_in_deg = 0
    print('YAW postion has been zero-ed')
#====================================================================================================
def find_pitch_parameters():    # determines 'full_ext_time', 'degree_ext_time', 'full_retract_time', 'degree_retract_time', & zeros 'pitch_pos'

    print('find_pitch_parameters() function has been called \n'
        '-- will commence in 10 seconds after initial adjustments to prevent extension damage...')


    global full_ext_time
    global degree_ext_time
    global full_retract_time
    global degree_retract_time
    global pitch_pos
    global pitch_pos_in_deg
    global ext_retr_conversion
    pitch_iterations = 1
    all_extensions = 0
    all_retractions = 0

    ## Find starting position

    # first retract to prevent over-extension (limit switches will prevent over-retraction)
    start_motor('pitch', 'retract', 100)
    print ('Moving ',yaw_or_pitch_move,' to ',direction_move,' with ',DC_move,'% DC...\n'
        '...initial retraction -- likely to limit switch -- to prevent over-extension.')
    time.sleep(5)               # 5 sec wait before retracting to then determine strating position
    stop_motors()

    # then extend a little to prepare to find lower boundary
    start_motor('pitch', 'extend', 100)
    print ('Moving ',yaw_or_pitch_move,' to ',direction_move,' with ',DC_move,'% DC...\n'
        '...initial extension -- to assure reed switch release -- to allow finding retracted position.')
    time.sleep(5)               # 5 sec wait before retracting to then determine strating position
    stop_motors()

    # retract to find starting position
    start_motor('pitch', 'retract', 100)
    print ('Moving ',yaw_or_pitch_move,' to ',direction_move,' with ',DC_move,'% DC...'
        '...looking for retracted position.')
    pitch_sensor()
    stop_motors()                                   #stops all motors    
    print('Reed switch sensor found. Now initiating pitch extension time estimation.')

    ## Measure full EXTENSION time
    for i in range(1,pitch_iterations+1,1):

        ## Measure full EXTENSION time
        start = time.time()                             #call and set time

        start_motor('pitch', 'extend', 100)
        print ('Moving ',yaw_or_pitch_move,' to ',direction_move,' with ',DC_move,'% DC...')
        time.sleep(5)                                   #initial delay to 'escape' previous sensor
        pitch_sensor()                                  #activtate sensor() function
        stop_motors()                                   #stops all motors    

        stop = time.time()                              #call and set time
        full_ext_time = round(stop-start,3)             #calculates extension time, rounds value to 3 decimals

        print('full EXTENSION in iteration ',i,' of ',pitch_iterations,' took ',full_ext_time,' seconds')

        all_extensions = all_extensions+full_ext_time
        time.sleep(1)                                   #short rest interval

        ## Measure full RETRACTION time
        start = time.time()                             #call and set time

        start_motor('pitch', 'retract', 100)
        print ('Moving ',yaw_or_pitch_move,' to ',direction_move,' with ',DC_move,'% DC...')
        time.sleep(5)                                   #initial delay to 'escape' previous sensor
        pitch_sensor()                                  #activtate sensor() function
        stop_motors()                                   #stops all motors    

        stop = time.time()                              #call and set time
        full_retract_time = round(stop-start,3)         #calculates extension time, rounds value to 3 decimals

        print('full RETRACTION in iteration ',i,' of ',pitch_iterations,' took ',full_retract_time,' seconds')

        all_retractions = all_retractions+full_retract_time
        time.sleep(1)                                   #short rest interval


    # average pitch times:
    full_ext_time = round(all_extensions/pitch_iterations,3)
    print('average full EXTENSION time is ',full_ext_time,' seconds')
    degree_ext_time = round(full_ext_time/90,3)
    print('~ time for EXTENSION by 1° is ',degree_ext_time,' seconds')

    full_retract_time = round(all_retractions/pitch_iterations,3)
    print('average full RETRACTION time is ',full_retract_time,' seconds')
    degree_retract_time = round(full_retract_time/90,3)
    print('~ time for RETRACTION by 1° is ',degree_retract_time,' seconds')

    ext_retr_conversion = round(full_ext_time/full_retract_time,3)
    print('conversion factor b/w slower extension and faster retraction is ',ext_retr_conversion)

    # additional retraction to assure hitting the limit switch in actuator:
    start_motor('pitch', 'retract', 100)
    print ('Moving ',yaw_or_pitch_move,' to ',direction_move,' with ',DC_move,'% DC...')
    time.sleep(2)                                   #initial delay to 'escape' previous sensor
    stop_motors()                                   #stops all motors  

    pitch_pos = 0                                       #zeros current pitch position
    pitch_pos_in_deg = 0                             #zeros current pitch position
    print('PITCH postion has been zero-ed')
#====================================================================================================
def return_to_origin():

    global yaw_pos
    global yaw_pos_in_deg
    global pitch_pos
    global pitch_pos_in_deg

    print('return_to_origin() function has been called... first returning yaw, then pitch')

    ## return YAW to origin
    if(yaw_pos_in_deg <= 180):
        return_direction = 'left'
    else:
        return_direction = 'right'
    start_motor('yaw', return_direction, 100)
    print ('Moving yaw to ',return_direction,' with ',DC,'% DC...')
    GPIO.remove_event_detect(yaw_sensor_monitor_pin)
    yaw_sensor()                                    #activtate sensor() function
    stop_motors()                                   #stops all motors
    yaw_pos = 0
    yaw_pos_in_deg = 0    
    print('Reed switch sensor found. This is the yaw origin --> yaw position is now zeroed.')    

    ## Find Pitch origin
    if (pitch_pos_in_deg < 10):
        start_motor('pitch', 'retract', 100)
        time.sleep((pitch_pos_in_deg * degree_retract_time)+2)
    else:
        start_motor('pitch', 'retract', 100)
        if(pitch_pos_in_deg>75):
            time.sleep(2)
        else:
            pass
        GPIO.remove_event_detect(pitch_sensor_monitor_pin)
        pitch_sensor()
        print('Reed switch sensor found.') 
        time.sleep(2)
        stop_motors()
        print('This is the pitch origin --> pitch position is now zeroed.')    

    print('The SoWiSo has been returned to origin')
#====================================================================================================
def measure_solar():            # measures value on adc and defines voltage variables 
            
    global volt_solar
    print('\n','\n','\n','[measure_solar() function has been called -- now measuring solar voltage] ...','\n')
    
    sum_adc_solar_value = 0         # initially sets variable to 0
    iterations = 10                 # number of iterations to be averaged across
    iteration_interval = 0.1        # time between iterative measurements 
    #Voltage divider setup
    R1 = 470                        # Resistor 1 in kilo-Ohm
    R2 = 47+4.7                     # Resistor 2 in kilo-Ohm


    print('Averaging solar voltage ADC measurements in current position...')
    
    GPIO.output(transistor_pin,GPIO.HIGH)
    time.sleep(0.2)

    for i in range(1, iterations+1, 1):                                 # conduct consecutive reads
        adc_solar_value = adc.analogRead(0)                             # reads the value (0-255) on adc channel 0
        print('measurement', i, ': the mesured ADC Value is ',adc_solar_value)
        sum_adc_solar_value = sum_adc_solar_value + adc_solar_value     # sum the ADC values over the iterations
        time.sleep(iteration_interval)                                  # time between measurements
    
    GPIO.output(transistor_pin,GPIO.LOW)

    avg_adc_solar_value = sum_adc_solar_value / iterations              # calculates the average ADC value
    print('The average ADC value in this position is', avg_adc_solar_value)
    
    volt_solar_on_adc = (v_ref * avg_adc_solar_value) / 255             # calculate the approximate actual voltage on the 8-bit ADC (after the voltage splitter)
    
    volt_solar = (volt_solar_on_adc * (R1 + R2)) / R2                   # calculate the approximate actual voltage generated by the solar cell (w/o the voltage splitter) 
                                                            
    print ('This computes to: \n'
               '~', round(volt_solar_on_adc, 2) ,'V on the ADC post V-splitter; ... therefore \n'
               '~', round(volt_solar, 2), 'V is generated in the current position by the solar cell')
#====================================================================================================
def assess_light():             # initial light level assessment if optimization is sensible 
            
    print('\n','\n','\n','[assess_light() function has been called -- now checking light levels','\n')
    
    global light_level
    
    light_sufficiency_threshold = 0.1    #MUST SET!! -- DETERMINES estimated voltage level above which optimzation makes sense
    darkness_threshold = 0.05               #MUST SET!! -- DETERMINES estimated voltage level below which it is considered to be dark

    measure_solar()

    if (volt_solar >=light_sufficiency_threshold):
        light_level = 'sufficient'
    elif (volt_solar>=darkness_threshold):
        light_level = 'twilight'
    else:
        light_level = 'dark'

    print('the measured light level indicates: ',light_level)
#====================================================================================================
def sample_120():               # measures solar in 3 yaw positions, then moves to the best 
            
    print('\n','\n','\n','[sample_120() function has been called -- now checking solar in 3 yaw positions','\n')

    measure_solar()
    volt_solar_at_0 = volt_solar

    start_motor('yaw', 'left', 100)
    run_motor(120*degree_yaw_turn_time)
    measure_solar()
    volt_solar_at_120 = volt_solar
    
    start_motor('yaw', 'left', 100)
    run_motor(120*degree_yaw_turn_time)
    measure_solar()
    volt_solar_at_240 = volt_solar

    if ((volt_solar_at_240 >= volt_solar_at_0) and (volt_solar_at_240 >= volt_solar_at_120)):
        pass
    elif ((volt_solar_at_120 >= volt_solar_at_0) and (volt_solar_at_120 >= volt_solar_at_240)):
        start_motor('yaw', 'right', 100)
        run_motor(120*degree_yaw_turn_time)
    elif ((volt_solar_at_0 >= volt_solar_at_120) and (volt_solar_at_0 >= volt_solar_at_240)):
        start_motor('yaw', 'left', 100)
        run_motor(120*degree_yaw_turn_time)
    print('starting optimization from this yaw position: ',yaw_pos_in_deg,'°')
#====================================================================================================
def toggle_opt_direction(optimize_what):

    global yaw_direction
    global pitch_direction

    if (optimize_what == 'yaw'):
        if (yaw_direction == 'left'):
            yaw_direction = 'right'
        elif (yaw_direction == 'right'):
            yaw_direction = 'left'
        else:
            print('something went wrong - check arguments and variables')
        print('the ',optimize_what,' direction has been toggled to ',yaw_direction)

    elif (optimize_what == 'pitch'):
        if (pitch_direction == 'extend'):
            pitch_direction = 'retract'
        elif (pitch_direction == 'retract'):
            pitch_direction = 'extend'
        else:
            print('something went wrong - check arguments and variables')
    else:
        print('something went wrong - check arguments and variables')

        print('the ',optimize_what,' direction has been toggled to ',pitch_direction)
#===================================================================================================
def optimize_yaw():

    global yaw_or_pitch
    yaw_or_pitch = 'yaw'
    global yaw_direction

    print('\n','\n','\n','[optimize_yaw() function has been called -- now finding optimal yaw position','\n')

    optimize = True
    yaw_deg_step = 15

    yaw_direction = 'right'         # start optimizing in this direction
    dir_tracker_0 = 1.0             # the following lines set up the tracking for directional changes
    dir_tracker_1 = 0.3             #   # the values are simply being passed back
    dir_tracker_2 = 0.3             #   # 0.3 is simply a dummy value that cannot sum to 0
    dir_tracker_3 = 0.3

    measure_solar()
    volt_solar_t0 = volt_solar
    
    while (optimize == True):
        start_motor(yaw_or_pitch, yaw_direction, 100)
        run_motor(yaw_deg_step * degree_yaw_turn_time)

        #the following sequence passes the directional tracking values backwards with each iteration
        dir_tracker_3 = dir_tracker_2   
        dir_tracker_2 = dir_tracker_1
        dir_tracker_1 = dir_tracker_0

        #this measures current solar value and updates variables to compare voltage with previous position
        measure_solar()
        volt_solar_t1 = volt_solar_t0
        volt_solar_t0 = volt_solar

        #the following IF argument toggles direction and inverts the tracking value when movement decreased voltage
        if (volt_solar_t0 <= volt_solar_t1):
            toggle_opt_direction(yaw_or_pitch)      #toggles the direction
            dir_tracker_0 = dir_tracker_0 * -1      #inverts the most recent direction value
        else:
            pass

        if ((dir_tracker_0 + dir_tracker_1 + dir_tracker_2 + dir_tracker_3) == 0):
            optimize = False
            #start_motor(yaw_or_pitch, yaw_direction, 100)
            #run_motor(yaw_deg_step * degree_yaw_turn_time)
        else:
            pass

    print('A YAW OPTIMUM has been found. \n (note that the optimum should be approximately the ',yaw_deg_step,'° rotational step size used for optimization.)')
#====================================================================================================
def optimize_pitch():

    global yaw_or_pitch
    yaw_or_pitch = 'pitch'
    global pitch_direction
    pitch_direction = 'retract'     # start optimizing in this direction

    print('\n','\n','\n','[optimize_pitch() function has been called -- now finding optimal pitch position','\n')

    optimize = True
    pitch_deg_step = 7.5

    dir_tracker_0 = 1.0         # the following lines set up th tracking for directional changes
    dir_tracker_1 = 0.3             # the values are simply being passed back
    dir_tracker_2 = 0.3             # 0.3 is simply a dummy value that cannot sum to 0
    dir_tracker_3 = 0.3

    measure_solar()
    volt_solar_t0 = volt_solar
    
    while (optimize == True):
        start_motor(yaw_or_pitch, pitch_direction, 100)
        if(pitch_direction == 'retract'):
            run_motor(pitch_deg_step * degree_retract_time)
        elif(pitch_direction == 'extend'):
            run_motor(pitch_deg_step * degree_ext_time)

        #the following sequence passes the directional tracking values backwards with each iteration
        dir_tracker_3 = dir_tracker_2   
        dir_tracker_2 = dir_tracker_1
        dir_tracker_1 = dir_tracker_0

        #this measures current solar value and updates variables to compare voltage with previous position
        measure_solar()
        volt_solar_t1 = volt_solar_t0
        volt_solar_t0 = volt_solar

        #the following IF argument toggles direction and inverts the tracking value when movement results in decreased voltage
        if ((volt_solar_t0 <= volt_solar_t1) or (pitch_pos_in_deg == (0 or 90))):
            toggle_opt_direction(yaw_or_pitch)      #toggles the direction
            dir_tracker_0 = dir_tracker_0 * -1      #inverts the most recent direction value
        else:
            pass

        if ((dir_tracker_0 + dir_tracker_1 + dir_tracker_2 + dir_tracker_3) == 0):
            optimize = False
            #start_motor(yaw_or_pitch, pitch_direction, 100)
            #if(pitch_direction == 'retract'):
            #    run_motor(pitch_deg_step * degree_retract_time)
            #elif(pitch_direction == 'extend'):
            #    run_motor(pitch_deg_step * degree_ext_time)
        else:
            pass

    print('A PITCH OPTIMUM has been found. \n (note that the optimum should be approximately the ',pitch_deg_step,'° step size used for optimization.)')
#====================================================================================================
def destroy():                  # sets all output pins to LOW, stops PWMs, runs cleanup
        
    print('\n','\n','\n','[destroy() function has been called -- now initiating clean-up] ...','\n')
       
    ##all output pins to low   

    GPIO.output(yaw_motor_in1_pin, GPIO.LOW)
    GPIO.output(yaw_motor_in2_pin, GPIO.LOW)
    GPIO.output(yaw_enable_pin, GPIO.LOW)
    
    GPIO.output(pitch_motor_in1_pin, GPIO.LOW)
    GPIO.output(pitch_motor_in2_pin, GPIO.LOW)
    GPIO.output(pitch_enable_pin, GPIO.LOW)
    
    GPIO.output(yaw_sensor_pwr_pin, GPIO.LOW)
    GPIO.output(pitch_sensor_pwr_pin, GPIO.LOW) 
    
    yaw_pwm.stop()                                  # stop yaw PWM
    pitch_pwm.stop()                                # stop pitch PWM

    GPIO.cleanup()                                  # Release all GPIO assignments

    print('...\n','...\n','PROGRAM HAS ENDED!','\n','\n')
#====================================================================================================
#====================================================================================================

def workflow():
    ### available functions:
    # setup()                   # inial setup: checks for adc, configures GPIO pins, creates PWMs, defines common parameters
    # rgb(color)    
    # rgb_blink(color,times)
    # start_motor(yaw_or_pitch, direction, DC)
    # stop_motors()             # stops all motors and zeros all DCs
    # run_motor(move_time)      
    # sensing_toggle()          # toggles 'sensing' variable b/w 'True' and 'False' 
    # yaw_sensor()              # activates sensing for yaw reed switch detection
    # pitch_sensor()            # activates sensing for pitch reed switch detection
    # find_yaw_parameters()     # determines time needed for yaw revolution 'full_yaw_turn_time', 'degree_yaw_turn_time', & zeros 'yaw_pos'
    # find_pitch_parameters()   # determines 'full_ext_time', 'degree_ext_time', 'full_retract_time', 'degree_retract_time', & zeros 'pitch_pos'
    # return_to_origin()
    # measure_solar()           # measures value on adc and defines voltage variables 
    # assess_light()            # initial light level assessment if optimization is sensible
    # sample_120()              # measures solar in 3 yaw positions, then moves to the best
    # optimize_yaw()
    # optimize_pitch()
    # destroy()                 # sets all output pins to LOW, stops PWMs, runs cleanup
    ##workflow()                # --- this function ---

    setup()
    
    global now
    now = datetime.datetime.now()
    global sowiso_starttime
    sowiso_starttime = now.replace(hour=6, minute=0, second=0, microsecond=0)
    global sowiso_stoptime
    sowiso_stoptime = now.replace(hour=23, minute=59, second=0, microsecond=0)
    
    print('commencing "sowiso_ctrl.py" control program')
    rgb_blink('sequence',3)
    rgb_blink('green',5)
    print('now = ',now)
    print('start / stop = ',sowiso_starttime,' / ',sowiso_stoptime)

    proceed = input('how to proceed? \n' 
        '"s" = standard program\n'
        '"d" = demo program\n')   
    if (proceed == 's'):
        optimization_interval = 60*60
    elif (proceed == 'd'):
        optimization_interval = 2*60

    while True:
        print ('entered while loop with option ',proceed)

        while ((now >= sowiso_starttime) and (now <= sowiso_stoptime)):
            assess_light()

            while ((light_level == 'dark') and (now <= sowiso_stoptime)):
                print('light level is "dark" --> waiting 60 minutes')
                time.sleep(60*60)
                now = datetime.datetime.now()
                assess_light()

            while ((light_level == 'twilight') and (now <= sowiso_stoptime)):
                print('light level is "twilight" --> waiting 30 minutes')
                time.sleep(30*60)
                now = datetime.datetime.now()
                assess_light()

            while (light_level == 'sufficient'):

                #find_parameters
                find_yaw_parameters()
                find_pitch_parameters()
                #move pitch to ~45°
                start_motor('pitch','extend',100)
                run_motor(degree_ext_time*45)
                #sample light at 3 120°-offset yaw positions 
                sample_120()

                while (light_level == 'sufficient'):
                    #optimize yaw and pitch
                    optimize_yaw()
                    optimize_pitch()
                    time.sleep(optimization_interval)
                    now = datetime.datetime.now()
                    assess_light()

        time.sleep(60*60)        #this is the waiting time during off-hours
        print('The current time is ',now,',\n'
        '   which is not between the SoWiSo'"'"'s defined operating hours of ',sowiso_starttime,' – ',sowiso_stoptime,'\n'
        '   ==> GOOD NIGHT !!!')



#====================================================================================================
#====================================================================================================
#====================================================================================================

if __name__ == '__main__':     # Program entrance
    print ('PROGRAM IS STARTING ... ')
    try:
        workflow()
    except KeyboardInterrupt:  # Press ctrl-c to end the program.
       
        destroy()




