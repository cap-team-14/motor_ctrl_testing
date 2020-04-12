#include <Arduino.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include "Timer.h"
#include <mcp_can_stm.h>


#define pin_esc PA9                 // ESC PWM pin

// global safety override
bool motor_run = 1;

// global current time tracker
unsigned long int c_time = millis();


/* #region  Position Sensor Defs */
#define num_sensors 2
uint8_t pot_pins [num_sensors] = {PB0, PB1};
uint16_t start_pos [num_sensors] = {0, 0};
uint16_t end_pos  [num_sensors] = {0, 0};
double pot_1_val = 0;
double pot_2_val = 0;
double actuator_pos = 0;
double actuator_pos_1 = 0;
double actuator_pos_2 = 0;
/* #endregion */

/* #region  PID */
// PID vars
double pid_sp, pid_in, pid_out;
double Kp = 0.25, Ki = 0, Kd = 0;

int ss_offset = 2;                  // acceptable steady state offset (percent)

// MatLab values (clutch):
// double Kp = 0.409610917794219;
// double Ki = 0.205094462119216;
// double Kd = 0.0300209726049592;

// autotune vars
byte ATuneModeRemember = AUTOMATIC;
double aTuneStep = 5, aTuneNoise = 2, aTuneStartValue = 0;
unsigned int aTuneLookBack = 5;
boolean tuning = false;

// PID objects
PID pid(&actuator_pos, &pid_out, &pid_sp, Kp, Ki, Kd, DIRECT);
PID_ATune aTune(&actuator_pos, &pid_out);
/* #endregion */

/* #region  CAN Bus */
#define MCP_CS PB12
#define MCP_INT PB11
// CAN bus def
MCP_CAN CAN;
uint32_t last_can_update = 0;	// last time a CAN message was received
uint16_t can_timeout = 5000;					// time without CAN message before error occurs
uint16_t transmit_ID = 0xA0;	        // CAN Bus transmit ID (messages will be sent with this ID)
uint32_t recv_id = 0x86;		        // CAN Bus receive ID (messages with this ID will be read)
uint8_t recv_len;				        // length of received message
uint8_t recv_msg[9];			        // the message (4 bytes)
uint8_t buff [8];	                    // message to be sent
/* #endregion */

/* #region  Scheduled Tasks */
// Schedule potentiometer read at 200 Hz
Timer sched_read_pos(5, &c_time);
Timer sched_serial_report(5, &c_time);
Timer sched_check_queue(10, &c_time);
/* #endregion */

/* #region  Function Prototypes */
uint16_t td(int);
void calibrate(bool);
void pid_setup();
void change_auto_tune();
void auto_tune_helper(boolean);
void serial_send();
void serial_receive();
void console_input();
void implausibility();
bool can_check();
void can_process();
/* #endregion */

/* #region  Command Queue */
struct Operation{
    uint8_t position;       // commanded position
    uint16_t delay;          // delay before going to commanded position
    uint32_t e_time;        // entry time into queue
};

Operation command_queue [10];
uint8_t next_command = 0;
uint8_t last_command = -1;
/* #endregion */

// main body
void setup(){
    pinMode(pin_esc, PWM);
    pwmWrite(pin_esc, td(0));       // command motor to remain stationary

    // initialize potition sensors
    for(int i=0; i<num_sensors; i++){
        pinMode(pot_pins[i], INPUT_ANALOG);
    }

    // pull sensor calibration from EEPROM
    uint16_t eep [4];
    for(int i=0; i<num_sensors*2; i++){
        eep[i] = EEPROM.read(i);
    }

    // check if EEPROM contains useful data
    for(int i=0; i<num_sensors; i++){
        start_pos[i] = eep[i] == 65535 ? 0 : eep[i];
        end_pos[i] = eep[i+2] == 65535 ? 0 : eep[i+2];
    }

    Serial.begin();                         // initialize Serial
    while (!Serial.available()){}           // DEBUG: wait for serial before proceeding
    Serial.flush();                         // clear serial buffer after receiving input
    Serial.println("Initialized");

    // initialize CAN Bus
    if(!(CAN_OK == CAN.begin(CAN_1000KBPS, MCP_CS))){\
        Serial.println("Can error");
		// warning = 0;
	}

	CAN.init_Filt(0, 0, recv_id);

    // update tasks to see which should run
    sched_read_pos.update();
    sched_serial_report.update();
    sched_check_queue.update();

    pid_setup();                            // run PID setup function
}

void loop(){
    console_input();                        // DEBUG: get instructions from Serial
    can_check();
    c_time = millis();

    // update tasks to see which should run
    sched_read_pos.update();
    sched_serial_report.update();
    sched_check_queue.update();
    
    // get position data
    if (sched_read_pos.query()){
        // we read from both inputs and then diregard the second reading if only 1 sensor is present
        pot_1_val = analogRead(pot_pins[0]);
        pot_2_val = analogRead(pot_pins[1]);
        actuator_pos_1 = map(pot_1_val, start_pos[0], end_pos[0], 0, 10000)/100.0;
        actuator_pos_2 = map(pot_2_val, start_pos[1], end_pos[1], 0, 10000)/100.0;
        //implausibility();                   // check implausibility
    }

    // if using PID AutoTune
    if (tuning){
        byte val = (aTune.Runtime());
        if (val != 0){
            tuning = false;
        }
        if (!tuning){ //we're done, set the tuning parameters
            Kp = aTune.GetKp();
            Ki = aTune.GetKi();
            Kd = aTune.GetKd();
            pid.SetTunings(Kp, Ki, Kd);
            auto_tune_helper(false);
        }
    } else {
        // figure out what PID wants motor to do
        pid.Compute();
    }

    // if PID is enabled
    if(pid.GetMode()){
        // safety checks

        // acceptable steady state error check
        pid_out = ((actuator_pos < pid_sp + ss_offset) && (actuator_pos > pid_sp - ss_offset)) ? 0 : pid_out;

        // soft limit checking
        pid_out = (actuator_pos >= 100 && pid_out > 0) ? 0 : pid_out;
        pid_out = (actuator_pos <= 0 && pid_out < 0) ? 0 : pid_out;

        // global safety override check
        pid_out = (motor_run) ? pid_out : 0;

        // if all safety checks passed
        pwmWrite(pin_esc, td(pid_out));
    }

    // DEBUG: send data over serial
    if(sched_serial_report.query()){
        //serial_send();
    }

    // check if a command has been queued
    // only really used to send step inputs for tuning and characterization
    if(sched_check_queue.query()){
        for(uint8_t i = next_command; i<10; i++){
            Operation op = command_queue[i];
            if(op.delay != 0 && (c_time - op.e_time > op.delay)){
                Serial.print("c_time: "); Serial.print(c_time); Serial.print(" e_time: "); Serial.print(op.e_time);
                Serial.print(" delay: "); Serial.println(op.delay);
                Serial.print("Setting motor throttle to: "); Serial.println(op.position);
                pwmWrite(pin_esc, td(op.position));
                //pid_sp = op.position;
                next_command = (next_command == 9) ? 0 : next_command + 1;
            }
        }
    }
}


bool can_check(){
    // check for CAN Comms error and process any waiting CAN messages
		if(CAN_MSGAVAIL == CAN.checkReceive()){
            Serial.println("CAN Message received");
			can_process();
			last_can_update = c_time;
		} else if((c_time - last_can_update > can_timeout)){
            // TODO: process error case
		}
}


void can_process(){
    // get message from buffer
    CAN.readMsgBuf(&recv_len, recv_msg);
    // verify that message was meant for us
	if(CAN.getCanId() == recv_id){
        // byte 1
        motor_run = recv_msg[0] & 0b10000000;                   // motor enable
        uint8_t pos = (recv_msg[0] & 0b01111111);               // commanded position

        // check that commanded position is valid then goto it if it is
        if(pos >= 0 && pos <= 100){
            pid_sp = pos;
        }

        // byte 2
        bool cal = (recv_msg[1] & 0b10000000) >> 7;             // calibrate toggle
        bool cal_m = (recv_msg[1] & 0b01000000) >> 6;           // calibration mode, select 0% or 100%
        bool conf_req = (recv_msg[1] & 0b00100000) >> 5;        // configuration has been requested
        bool conf_exp = (recv_msg[1] & 0b00010000) >> 4;        // expect to receive configuration

        // if we are expecting to receive configuration, extract it
        if(conf_exp){
            // bytes 3 + 4 -> kp
            uint16_t Kp_int = (recv_msg[2] << 8);
            Kp_int &= (0xFF00 | recv_msg[3]);
            Kp = Kp_int / 6000.0;

            // bytes 5 + 6 -> ki
            uint16_t Ki_int = (recv_msg[4] << 8);
            Ki_int &= (0xFF00 | recv_msg[5]);
            Kp = Ki_int / 6000.0;

            // bytes 7 + 8 -> ki
            uint16_t Kd_int = (recv_msg[6] << 8);
            Kd_int &= (0xFF00 | recv_msg[7]);
            Kd = Kd_int / 6000.0;
        }

        if(cal){
            calibrate(cal_m);
        }

        Serial.print("Motor enable: "); Serial.print(motor_run);
        Serial.print("\tCommanded Percent: "); Serial.println(pos);
    }
}


// Throttle Implausibility Check function
void implausibility(){
    // check delta between sensor readings
    if(abs(actuator_pos_1 - actuator_pos_2) < 10){
        actuator_pos = (actuator_pos_1 + actuator_pos_2)/2;
        motor_run = 1;
    } else {
        // if delta is greater than ten, there's an issue
        Serial.print("Implausibility. TPS1: "); Serial.print(actuator_pos_1); Serial.print(", TPS2: ");
        Serial.println(actuator_pos_2);
        actuator_pos = 0;
        motor_run = 0;
    }
}


// function to process commands from serial buffer
void console_input(){
    if (Serial.available()){
        char c = Serial.read();
        if ('c' == c){
            Serial.println(c);
            //calibrate();
        } else if ('p' == c) {
            Serial.println("Values Stored Currently:");
            Serial.print("\tStart Position: ");
            Serial.print(start_pos[0]);
            Serial.print(" ");
            Serial.println(start_pos[1]);
            Serial.print("\tEnd Position: ");
            Serial.print(end_pos[0]);
            Serial.print(" ");
            Serial.println(end_pos[1]);
        } else if ('l' == c) {
            Serial.println("Live Values:");
            Serial.print("\tCurrent Actuator Position: ");
            Serial.println(pid_in);
        } else if ('r' == c) {
            pwmWrite(pin_esc, td(0));
            Serial.print("Enter desired position: ");
            while (!Serial.available()){}
            int temp = Serial.parseInt();
            if(temp <= 100 && temp >= 0){
                pid_sp = temp;
                Serial.print("\nGoing to position: "); Serial.println(pid_sp);
            } else {
                Serial.println("Entered value outside limits");
            }
        } else if ('t' == c) {
            pwmWrite(pin_esc, td(0));
            change_auto_tune();
        } else if('k' == c) {
            pwmWrite(pin_esc, td(0));
            char quit = 0;
            while ('q' != quit){
                Serial.println("Current params:");
                Serial.print("\tKp: "); Serial.println(Kp, 6);
                Serial.print("\tKi: "); Serial.println(Ki, 6);
                Serial.print("\tKd: "); Serial.println(Kd, 6);
                Serial.println("Enter \"p\" to set Kp");
                Serial.println("Enter \"i\" to set Ki");
                Serial.println("Enter \"d\" to set Kd");
                Serial.println("Enter \"q\" to return to normal operation");
                while (!Serial.available()){}
                char c = Serial.read();
                if ('p' == c){
                    Serial.println("Enter new Kp: ");
                    while (!Serial.available()){}
                    Kp = Serial.parseFloat();
                } else if('i' == c){
                    Serial.println("Enter new Ki: ");
                    while (!Serial.available()){}
                    Ki = Serial.parseFloat();
                } else if('d' == c){
                    Serial.println("Enter new Kd: ");
                    while (!Serial.available()){}
                    Kd = Serial.parseFloat();
                } else if('q' == c){
                    Serial.println("Setting params to: ");
                    Serial.print("\tKp: "); Serial.println(Kp);
                    Serial.print("\tKi: "); Serial.println(Ki);
                    Serial.print("\tKd: "); Serial.println(Kd);

                    pid.SetTunings(Kp, Ki, Kd);
                    quit = 'q';
                }
            }
        } else if('s' == c) {
            Serial.println("STOPPING MOTOR");
            pwmWrite(pin_esc, td(0));
            while(true){
                while (!Serial.available()){}
                char c = Serial.read();
                if ('s' == c){
                    Serial.println("RESUMING MOTOR");
                    break;
                }
            }
        } else if ('i' == c){
            pwmWrite(pin_esc, td(0));
            Serial.println("Send input.");
            Serial.println("Enter \"s\" for step input.");
            while(true){
                while (!Serial.available()){}
                char c = Serial.read();
                if ('q' == c){
                    Serial.println("Returning to normal operation.");
                    break;
                } else if('s' == c){
                    Serial.println("Enter Start Value and End Value:");
                    while (!Serial.available()){}
                    int8_t step [2] = {0, 0};
                    for(uint8_t i=0; i<2; i++){
                        int c = Serial.parseInt();
                        if (c < 0 || c > 100){
                            Serial.println("Invalid value detected, returning to normal operation.");
                            break;
                        }
                        step[i] = c;
                        while (!Serial.available()){}
                    }
                    Serial.print("Step input starting at "); Serial.print(step[0]) ; Serial.print(" and ending at ");
                    Serial.println(step[1]);
                    Serial.print("Setting motor throttle to: "); Serial.println(step[0]);
                    pwmWrite(pin_esc, td(step[0]));
                    //pid_sp = step[0];
                    Operation op;
                    op.delay = 300;
                    op.e_time = millis();
                    op.position = step[1];

                    if(last_command == 9){
                        command_queue[0] = op;
                    } else {
                        command_queue[++last_command] = op;
                    }
                    break;
                }
            }
        } else if('m' == c){
            bool mode = pid.GetMode();
            pid.SetMode(!mode);
            Serial.print("Toggling PID status to "); Serial.println(!mode);
        }
    }
}


// configure and enable the PID
void pid_setup(){
    // pid_sp = 50;                    // DEBUG: inital PID setpoint
    pid.SetOutputLimits(-5, 50);    // PID output limits
    pid.SetSampleTime(5);           // PID refresh rate

    pid.SetMode(AUTOMATIC);         // AUTOMATIC: enabled; MANUAL: disabled
    
    // if tuning, change appropriate settings
    if (tuning){
        tuning = false;
        change_auto_tune();
        tuning = true;
    }
}


// convert duty cycle (in percent) to PWM value
uint16_t td(int percent)
{
    percent += 50;                              // shift PID output so that it's in the range 0->100
    //percent = (percent >= 51 && percent < 55) ? 55 : percent;
    //percent = (percent <= 49 && percent > 45) ? 45 : percent;

    // int temp = 65535*((percent/100.0)*(1750+250))/1800;
    return 65535 * map(percent, 0, 100, 250, 1750) / 1800;
}


// function to calibrate the position sensor(s)
void calibrate(bool high_low){
    pwmWrite(pin_esc, td(0));

    if(!high_low){
        start_pos[0] = analogRead(pot_pins[0]);
        start_pos[1] = analogRead(pot_pins[1]);
        EEPROM.write(0, start_pos[0]);
        EEPROM.write(1, start_pos[1]);
    } else {
        end_pos[0] = analogRead(pot_pins[0]);
        end_pos[1] = analogRead(pot_pins[1]);
        EEPROM.write(2, end_pos[0]);
        EEPROM.write(3, end_pos[1]);
    }

    // Serial.println("Calibrate");
    // char quit = 0;
    // while ('q' != quit){
    //     Serial.println("Enter \"0\" to set zero point or \"1\" to set fully open point");
    //     Serial.println("Enter \"q\" to return to normal operation");
    //     while (!Serial.available()){}
    //     char c = Serial.read();
    //     if ('0' == c){
    //         start_pos[0] = analogRead(pot_pins[0]);
    //         start_pos[1] = analogRead(pot_pins[1]);
    //         Serial.print("Zero position read as: ");
    //         Serial.print(start_pos[0]);
    //         Serial.print(" ");
    //         Serial.println(start_pos[1]);
    //     } else if ('1' == c) {
    //         end_pos[0] = analogRead(pot_pins[0]);
    //         end_pos[1] = analogRead(pot_pins[1]);
    //         Serial.print("Open position read as: ");
    //         Serial.print(end_pos[0]);
    //         Serial.print(" ");
    //         Serial.println(end_pos[1]);
    //     } else if ('q' == c) {
    //         quit = 'q';

    //         // store these values in EEPROM
    //         EEPROM.write(0, start_pos[0]);
    //         EEPROM.write(1, start_pos[1]);
    //         EEPROM.write(2, end_pos[0]);
    //         EEPROM.write(3, end_pos[1]);

    //         Serial.println("Exiting Calibration Mode");
    //         Serial.print("Start Values: ");
    //         Serial.print(start_pos[0]);
    //         Serial.print(" ");
    //         Serial.println(start_pos[1]);
    //         Serial.print("End Values: ");
    //         Serial.print(end_pos[0]);
    //         Serial.print(" ");
    //         Serial.println(end_pos[1]);
    //     }
    // }
}


// autotune helper function
void change_auto_tune(){
    if (!tuning){
        //Set the output to the desired starting frequency.
        pid_out = aTuneStartValue;
        aTune.SetNoiseBand(aTuneNoise);
        aTune.SetOutputStep(aTuneStep);
        aTune.SetLookbackSec((int)aTuneLookBack);
        aTune.SetControlType(1);
        auto_tune_helper(true);
        tuning = true;
    } else { //cancel autotune
        aTune.Cancel();
        tuning = false;
        auto_tune_helper(false);
    }
}


// autotune helper function
void auto_tune_helper(boolean start){
    if (start){
        ATuneModeRemember = pid.GetMode();
    } else {
        pid.SetMode(ATuneModeRemember);
    }
}


// function to report data over serial
void serial_send(){
    Serial.print(c_time);
    Serial.print(",");
    //Serial.print("setpoint: ");
    //Serial.print(pid_sp);
    //Serial.print(",");
    // Serial.print("\tinput: ");
    Serial.print(actuator_pos);
    Serial.print(",");
    // Serial.print("\toutput: ");
    Serial.println(pid_out);
    if (tuning){
        Serial.println("\ttuning mode");
    } //else {
    //     Serial.print("\tKp: ");
    //     Serial.print(pid.GetKp());
    //     Serial.print(" ");
    //     Serial.print("\tKi: ");
    //     Serial.print(pid.GetKi());
    //     Serial.print(" ");
    //     Serial.print("\tKd: ");
    //     Serial.print(pid.GetKd());
    //     Serial.println();
    // }
}