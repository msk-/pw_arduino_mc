/*****************************************************
*
* Pi Wars Robot Software (PWRS) Motor Controller
*
* Copyright (c) 2014 Matt Kingston (mattkingston@gmail.com)
*
* PWRS is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* PWRS is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with PWRS.  If not, see <http://www.gnu.org/licenses/>.
*
* This is the motor controller software, in the form of an
* Arduino sketch.
*
*****************************************************/

/**************************************************
* Includes
***************************************************/

/* None */

/**************************************************
* Defines
***************************************************/

/****************** CONSTANTS *******************/
#define US_PER_SEC    (1000*1000)

/******************* UTILITY ********************/
#define MAX(a,b) ((a > b) ? a : b)

/******************** LIMITS ********************/
#define UINT8_MAX 0xFF

/******************* PIN DEFS *******************/
#define EN_RHS        6   /*  enable  A,  must  be  PWM  */
#define RHS_PIN1      5
#define RHS_PIN2      7
#define EN_LHS        9   /*  enable  B,  must  be  PWM  */
#define LHS_PIN1      8
#define LHS_PIN2      10

/* Quadrature encoder pins. */
#define QUAD_RHS_0      3
#define QUAD_RHS_1      4
#define QUAD_LHS_0      11
#define QUAD_LHS_1      12

#define LED           13

/****************** MOTOR DEFS ******************/
#define FULL_POWER  255

#define MOTOR_RHS     EN_RHS
#define MOTOR_LHS     EN_LHS

/***************** CONTROL DEFS *****************/
/* Maximum frequency. Total guess at this stage. Might need calibration. */
#define MAX_FREQ 200

/* Gain values for PID control */
#define  K_P  1.2
#define  K_I  0.0
#define  K_D  0.0

/****************** COMMS DEFS ******************/
#define  HEADER_BYTE               0xFF
#define  CMD_LHS_FWD               0x00
#define  CMD_LHS_BACK              0x01
#define  CMD_RHS_FWD               0x02
#define  CMD_RHS_BACK              0x03
#define  CMD_ACK                   0x04
#define  CMD_STALL                 0x05
#define  CMD_RHS_CLICKS_REMAINING  0x06
#define  CMD_LHS_CLICKS_REMAINING  0x06

/**************************************************
* Data Types
**************************************************/

typedef enum quad_state_e
{
    qs_backward = 0,
    qs_forward = 1,
    qs_no_change = 2,
    qs_error = 3
} quad_state_e;

typedef enum read_state_e
{
    read_state_none,
    read_state_header,
    read_state_command,
    read_state_speed_0,
    read_state_speed_1,
    read_state_ticks_0,
    read_state_ticks_1
} read_state_e;

typedef enum motor_control_e
{
    motor_coast,
    motor_forward,
    motor_backward,
    motor_brake
} motor_control_e;

typedef struct control_frame_t
{
    uint8_t command;
    uint16_t freq;
    uint16_t ticks;
} control_frame_t;

typedef struct motor_state_t
{
    /* TODO: 
    uint8_t enable_pin;
    uint8_t out_pin_1;
    uint8_t out_pin_2;
    */
    quad_state_e desired_dir;
    quad_state_e current_dir;
    uint8_t curr_duty;
    uint8_t curr_quad_state;
    uint16_t target_freq;
    uint16_t ticks_remaining;
    /* Stores the times of the most recent two unprocessed motor quadrature
     * state change events in order to calculate speed. */
    unsigned long last_change;
    unsigned long dt_us;
    uint32_t freq;
} motor_state_t;

/**************************************************
* Public Data
**************************************************/

/* Quadrature encoder interpretation matrix */
/* QSCM[old_val][curr_val] */
static const quad_state_e QSCM[4][4] = 
{
    {  qs_no_change,  qs_backward,   qs_forward,    qs_error      },
    {  qs_forward,    qs_no_change,  qs_error,      qs_backward   },
    {  qs_backward,   qs_error,      qs_no_change,  qs_forward    },
    {  qs_error,      qs_forward,    qs_backward,   qs_no_change  }
};

/******************** STATE *********************/
static motor_state_t rhs_state;
static motor_state_t lhs_state;

/**************************************************
* Public Functions
***************************************************/

/**
 * Standard Arduino setup function
 */
void setup()
{
    /* Set up pin directions */
    pinMode(EN_RHS, OUTPUT);
    pinMode(EN_LHS, OUTPUT);
    pinMode(RHS_PIN1, OUTPUT);
    pinMode(RHS_PIN2, OUTPUT);
    pinMode(LHS_PIN1, OUTPUT);
    pinMode(LHS_PIN2, OUTPUT);
    pinMode(LED, OUTPUT);
    pinMode(QUAD_RHS_0, INPUT);
    pinMode(QUAD_RHS_1, INPUT);
    pinMode(QUAD_LHS_0, INPUT);
    pinMode(QUAD_LHS_1, INPUT);

    /* TODO: Set all pin initial states (i.e. motors stopped) */

    /* Initialise state */
    memset(&rhs_state, 0, sizeof(rhs_state));
    memset(&lhs_state, 0, sizeof(lhs_state));

    /* Start listening for instructions */
    Serial.begin(115200);
}


/**************************************************
* Private Functions
***************************************************/

/*
 * Set power/direction of a single motor.
 */
void control_motor(int en_pin, int in_0, int state_0, int in_1, int state_1, int duty)
{
    digitalWrite(in_0, state_0);
    digitalWrite(in_1, state_1);
    analogWrite(en_pin, duty);
}

/* Instruct a specific motor to move forward/backward/stop/coast.
 * Examples:
 *   RHS motor half-speed forward:
 *     motor(MOTOR_RHS, motor_forward, 0xF0);
 *   LHS motor full-speed backward:
 *     motor(MOTOR_LHS, motor_backward, 0xFF);
 */
void set_motor_params(int motor_ind, motor_control_e mode, uint8_t duty)
{
    int en_pin = (motor_ind == MOTOR_RHS) ? EN_RHS : EN_LHS;
    int in_0 = (motor_ind == MOTOR_RHS) ? RHS_PIN1 : LHS_PIN1;
    int in_1 = (motor_ind == MOTOR_RHS) ? RHS_PIN2 : LHS_PIN2;

    switch(mode)
    {
    case motor_coast:
        digitalWrite(en_pin, LOW);
        break;

    case motor_forward:
        control_motor(en_pin, in_0, HIGH, in_1, LOW, duty);
        break;

    case motor_backward:
        control_motor(en_pin, in_0, LOW, in_1, HIGH, duty);
        break;

    case motor_brake:
        control_motor(en_pin, in_0, LOW, in_1, LOW, duty);
        break;
    }
}

/* Called when a control frame is received over serial. Manages requested state
 * updates */
void handle_ctrl_frame_received(const control_frame_t* frame)
{
    switch (frame->command)
    {
    case CMD_LHS_FWD:
        Serial.write('1');
        lhs_state.desired_dir = qs_forward;
        lhs_state.ticks_remaining = frame->ticks;
        lhs_state.target_freq = frame->freq;
        break;
    case CMD_LHS_BACK:
        Serial.write('3');
        lhs_state.desired_dir = qs_backward;
        lhs_state.ticks_remaining = frame->ticks;
        lhs_state.target_freq = frame->freq;
        break;
    case CMD_RHS_FWD:
        Serial.write('2');
        rhs_state.desired_dir = qs_forward;
        rhs_state.ticks_remaining = frame->ticks;
        rhs_state.target_freq = frame->freq;
        break;
    case CMD_RHS_BACK:
        Serial.write('4');
        rhs_state.desired_dir = qs_backward;
        rhs_state.ticks_remaining = frame->ticks;
        rhs_state.target_freq = frame->freq;
        break;
    }
}

/* Read from the serial port and update the motor state depending on the command
 * received. Additionally; echo any received commands back to the serial */
void get_instruction()
{
    static read_state_e read_state = read_state_none;
    static control_frame_t new_ctrl_frame;
    uint8_t read_byte, checksum;
    if (Serial.available())
    {
        read_byte = Serial.read();
        switch (read_state)
        {
        case read_state_none:
            if (read_byte == HEADER_BYTE)
            {
                read_state = read_state_header;
                checksum = 0;
                memset(&new_ctrl_frame, 0, sizeof(new_ctrl_frame));
            }
            break;
        case read_state_header:
            new_ctrl_frame.command = read_byte;
            read_state = read_state_command;
            checksum ^= read_byte;
            break;
        case read_state_command:
            new_ctrl_frame.freq = read_byte << 8;
            read_state = read_state_speed_0;
            checksum ^= read_byte;
            break;
        case read_state_speed_0:
            new_ctrl_frame.freq += read_byte;
            read_state = read_state_speed_1;
            checksum ^= read_byte;
            break;
        case read_state_speed_1:
            new_ctrl_frame.ticks = read_byte << 8;
            read_state = read_state_ticks_0;
            checksum ^= read_byte;
            break;
        case read_state_ticks_0:
            new_ctrl_frame.ticks += read_byte;
            read_state = read_state_ticks_1;
            checksum ^= read_byte;
            break;
        case read_state_ticks_1:
            checksum = (checksum == HEADER_BYTE) ? 0 : checksum;
            if (checksum == read_byte)
            {
                handle_ctrl_frame_received(&new_ctrl_frame);
                /* Write ack back over serial */
                #if 0
                Serial.write(HEADER_BYTE);
                Serial.write(CMD_ACK);
                Serial.write(CMD_ACK ^ 0x00);
                #endif
            }
            else
            {
                /* TODO: write nack back over serial */
            }
            read_state = read_state_none;
            break;
        }
    }
}

/* Updates quadrature information for one motor */
void read_update_motor_quadrature(int motor_ind)
{
    motor_state_t* motor_state = (motor_ind == MOTOR_RHS) ? &rhs_state : &lhs_state;
    int q_pin_0 = (motor_ind == MOTOR_RHS) ? QUAD_RHS_0 : QUAD_LHS_0;
    int q_pin_1 = (motor_ind == MOTOR_RHS) ? QUAD_RHS_1 : QUAD_LHS_1;
    uint8_t new_pin_state = digitalRead(q_pin_0) + (digitalRead(q_pin_1) << 1);
    quad_state_e new_quad_state = QSCM[motor_state->curr_quad_state][new_pin_state];
    unsigned long now = micros();

    switch (new_quad_state)
    {
    case qs_no_change:
        /* TODO: if this happens for too long, and we have a desired dir +
         * speed, we've probably stalled */
        break;
    case qs_error:
        /* TODO */
        break;
    case qs_forward:
        /* Fall-through intentional */
    case qs_backward:
        motor_state->current_dir = new_quad_state;
        motor_state->dt_us = now - motor_state->last_change;
        motor_state->freq = (US_PER_SEC) / motor_state->dt_us;
        motor_state->last_change = now;
        break;
    }
}

/* Updates quadrature information for each motor */
void read_update_quadrature()
{
    read_update_motor_quadrature(MOTOR_RHS);
    read_update_motor_quadrature(MOTOR_LHS);
}

/* Uses error between observed speed and desired speed to calculate PWM duty
 * (power to motor) in order to achieve desired speed */
void update_motor_control(int motor_ind)
{
    motor_state_t* motor = (motor_ind == MOTOR_RHS) ? &rhs_state : &lhs_state;
    uint16_t error = motor->target_freq - motor->freq;
    uint16_t new_power = K_P * error;
    uint8_t duty = MAX(UINT8_MAX, new_power);
    set_motor_params(motor_ind, motor_forward, duty);
}

/* Updates the output control state for each motor */
void update_control_state()
{
    update_motor_control(MOTOR_RHS);
    update_motor_control(MOTOR_LHS);
}

/**
 * Standard Arduino loop function
 */
void loop()
{
    get_instruction();
    read_update_quadrature();
    update_control_state();
}

