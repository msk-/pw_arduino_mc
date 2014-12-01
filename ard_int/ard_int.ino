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
/* Microseconds per second */
static const unsigned long US_PER_SEC = 1000UL * 1000UL;

/******************* UTILITY ********************/
#define MAX(a,b) ((a > b) ? a : b)

/******************** LIMITS ********************/
#define UINT8_MAX 0xFF
#define MOVING_AVERAGE_SIZE 10

/******************* PIN DEFS *******************/
#define EN_RHS        6   /*  enable  A,  must  be  PWM  */
#define RHS_PIN1      5
#define RHS_PIN2      7
#define EN_LHS        9   /*  enable  B,  must  be  PWM  */
#define LHS_PIN1      8
#define LHS_PIN2      10

/* Quadrature encoder pins. */
/* Rename these to QUAD_RHS_YEL and QUAD_RHS_WHITE. Similarly LHS. */
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
/* TODO: used? */
#define MAX_FREQ 200

/* Gain values for PID control */
static const double K_P = 0.9;
static const double K_I = (double)1 / (double)US_PER_SEC;
static const double K_D = 0.0001;

/****************** COMMS DEFS ******************/
#define  HEADER_BYTE               0xFF
#define  CMD_LHS_FWD               0x00
#define  CMD_LHS_BACK              0x01
#define  CMD_RHS_FWD               0x02
#define  CMD_RHS_BACK              0x03
#define  CMD_ACK                   0x04
#define  CMD_STALL                 0x05
#define  CMD_RHS_CLICKS_REMAINING  0x06
#define  CMD_LHS_CLICKS_REMAINING  0x07
#define  CLICKS_UPDATE_FREQUENCY   0x08

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
    read_state_speed,
    read_state_clicks
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
    uint16_t clicks;
} control_frame_t;

typedef struct motor_state_t
{
    /* TODO: 
    uint8_t enable_pin;
    uint8_t out_pin_1;
    uint8_t out_pin_2;
    */
    size_t quad_click_ix;
    unsigned long quad_click_times[MOVING_AVERAGE_SIZE];
    motor_control_e desired_dir;
    quad_state_e current_dir;
    int32_t accumulated_error;
    uint8_t curr_duty;
    uint8_t curr_quad_pin_state;
    int32_t target_freq;
    uint16_t clicks_remaining;
    unsigned long last_control_update_us;
    unsigned long dt_us;
    int32_t dfreq;
    int32_t freq;
} motor_state_t;

/**************************************************
* Private function declarations
***************************************************/

static void control_motor(int en_pin, int in_0, int state_0, int in_1, int state_1, int duty);
static void set_motor_params(int motor_ind, motor_control_e mode, uint8_t duty);
static void handle_ctrl_frame_received(const control_frame_t* frame);
static void get_instruction();
static void update_motor_quadrature_data(int motor_ind);
static void update_quadrature_data();
static void update_motor_control(int motor_ind);
static void update_control_state();
static void send_state_update();
static void initialise_motor(motor_state_t* state);

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
    initialise_motor(&rhs_state);
    initialise_motor(&lhs_state);

    /* Start listening for instructions */
    Serial.begin(115200);
}

/**
 * Standard Arduino loop function
 */
void loop()
{
    get_instruction();
    update_quadrature_data();
    update_control_state();
    send_state_update();
}

/**************************************************
* Private Functions
***************************************************/

/**
 * Set start-up state of motors
 */
static void initialise_motor(motor_state_t* state)
{
    state->desired_dir = motor_forward;
    state->current_dir = qs_no_change;
    state->accumulated_error = 0;
    state->curr_duty = 0;
    state->curr_quad_pin_state = 0;
    state->target_freq = 0;
    state->clicks_remaining = 0;
    state->last_control_update_us = 0;
    state->dt_us = 0;
    state->dfreq = 0;
    state->freq = 0;
    state->quad_click_ix = 0;
    memset(&state->quad_click_times, 0, sizeof(state->quad_click_times));
}

/*
 * Set power/direction of a single motor.
 */
static void control_motor(int en_pin, int in_0, int state_0, int in_1, int state_1, int duty)
{
    digitalWrite(in_0, state_0);
    digitalWrite(in_1, state_1);
    analogWrite(en_pin, duty);
}

/* Instruct a specific motor to move forward/backward/stop/coast.
 * Examples:
 *   RHS motor half-speed forward:
 *     motor(MOTOR_RHS, motor_forward, 0x80);
 *   LHS motor full-speed backward:
 *     motor(MOTOR_LHS, motor_backward, 0xFF);
 */
static void set_motor_params(int motor_ind, motor_control_e mode, uint8_t duty)
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
static void handle_ctrl_frame_received(const control_frame_t* frame)
{
    switch (frame->command)
    {
    case CMD_LHS_FWD:
        lhs_state.desired_dir = motor_forward;
        lhs_state.clicks_remaining = frame->clicks;
        lhs_state.target_freq = frame->freq;
        lhs_state.accumulated_error = 0;
        break;
    case CMD_LHS_BACK:
        lhs_state.desired_dir = motor_backward;
        lhs_state.clicks_remaining = frame->clicks;
        lhs_state.target_freq = frame->freq;
        lhs_state.accumulated_error = 0;
        break;
    case CMD_RHS_FWD:
        rhs_state.desired_dir = motor_forward;
        rhs_state.clicks_remaining = frame->clicks;
        rhs_state.target_freq = frame->freq;
        rhs_state.accumulated_error = 0;
        break;
    case CMD_RHS_BACK:
        rhs_state.desired_dir = motor_backward;
        rhs_state.clicks_remaining = frame->clicks;
        rhs_state.target_freq = frame->freq;
        rhs_state.accumulated_error = 0;
        break;
    }
}

/* Read from the serial port and update the motor state depending on the command
 * received. Additionally; echo any received commands back to the serial */
static void get_instruction()
{
    static read_state_e read_state = read_state_none;
    static control_frame_t new_ctrl_frame;
    static uint8_t checksum;
    uint8_t read_byte;
    if (Serial.available())
    {
        read_byte = Serial.read();
        if (read_byte == HEADER_BYTE)
        {
            read_state = read_state_header;
            checksum = HEADER_BYTE;
        }
        else
        {
            switch (read_state)
            {
            case read_state_none:
                break;
            case read_state_header:
                new_ctrl_frame.command = read_byte;
                read_state = read_state_command;
                checksum ^= read_byte;
                break;
            case read_state_command:
                new_ctrl_frame.freq = read_byte << 1;
                read_state = read_state_speed;
                checksum ^= read_byte;
                break;
            case read_state_speed:
                new_ctrl_frame.clicks = read_byte << 1;
                read_state = read_state_clicks;
                checksum ^= read_byte;
                break;
            case read_state_clicks:
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
                #if 0
                else
                {
                    /* TODO: write nack back over serial? */
                }
                #endif
                read_state = read_state_none;
                break;
            }
        }
    }
}

/* Updates quadrature information for one motor */
static void update_motor_quadrature_data(int motor_ind)
{
    motor_state_t* motor = (motor_ind == MOTOR_RHS) ? &rhs_state : &lhs_state;
    int q_pin_0 = (motor_ind == MOTOR_RHS) ? QUAD_RHS_0 : QUAD_LHS_0;
    int q_pin_1 = (motor_ind == MOTOR_RHS) ? QUAD_RHS_1 : QUAD_LHS_1;
    uint8_t new_quad_pin_state = digitalRead(q_pin_0) + (digitalRead(q_pin_1) << 1);
    quad_state_e new_quad_state = QSCM[motor->curr_quad_pin_state][new_quad_pin_state];
    motor->curr_quad_pin_state = new_quad_pin_state;
    unsigned long now = micros();
    int32_t old_freq = motor->freq;
    size_t oldest_quad_click_ix = (motor->quad_click_ix + 1) % MOVING_AVERAGE_SIZE;

    switch (new_quad_state)
    {
    case qs_error:
        /* TODO */
        break;
    case qs_forward:
        /* TODO: we should probably handle forward and backward movement
         * separately. (Perhaps backward motion should just be negative forward
         * motion for the purpose of calculating motor power). */
        /* Fall-through intentional */
    case qs_backward:
        /* TODO: Increase clicks remaining if we're going in the wrong
         * direction? */
        motor->current_dir = new_quad_state;
        motor->clicks_remaining -= (motor->clicks_remaining > 0) ? 1 : 0;
        motor->dt_us = now - motor->quad_click_times[oldest_quad_click_ix];
        motor->freq = (motor->dt_us == 0) ? 0 : 
            (MOVING_AVERAGE_SIZE * US_PER_SEC) / motor->dt_us;
        motor->dfreq = motor->freq - old_freq;
        motor->quad_click_ix = oldest_quad_click_ix;
        motor->quad_click_times[oldest_quad_click_ix] = now;
        break;
    case qs_no_change:
        /* TODO: if qs_no_change happens for too long, and we have a desired
         * direction and speed, we've probably stalled */
        motor->dt_us = now - motor->quad_click_times[oldest_quad_click_ix];
        motor->freq = (motor->dt_us == 0) ? 0 : 
            (MOVING_AVERAGE_SIZE * US_PER_SEC) / motor->dt_us;
        motor->dfreq = motor->freq - old_freq;
        break;
    }
}

/* Updates quadrature information for each motor */
static void update_quadrature_data()
{
    update_motor_quadrature_data(MOTOR_RHS);
    update_motor_quadrature_data(MOTOR_LHS);
}

/* Uses error between observed speed and desired speed to calculate PWM duty
 * (power to motor) in order to achieve desired speed */
static void update_motor_control(int motor_ind)
{
    motor_state_t* motor = (motor_ind == MOTOR_RHS) ? &rhs_state : &lhs_state;
    unsigned long now = micros();
    unsigned long control_dt = now - motor->last_control_update_us;
    if (motor->clicks_remaining > 0)
    {
        int32_t error = motor->target_freq - motor->freq;
        motor->accumulated_error += error * control_dt;
        int32_t proportional_term = K_P * error;
        int32_t integral_term = K_I * motor->accumulated_error;
        int32_t derivative_term = (control_dt == 0) ? 0 : K_D * (motor->dfreq / control_dt);
        int32_t new_power = proportional_term + integral_term; /* + derivative_term; */
        uint8_t duty = (new_power < 0) ? 0 :
                       ((new_power > UINT8_MAX) ? UINT8_MAX : new_power);
        #if 0
        Serial.write('\n');
        Serial.print(motor->clicks_remaining);
        Serial.write('\t');
        Serial.print(motor->target_freq);
        Serial.write('\t');
        Serial.print(motor->freq);
        Serial.write('\t');
        Serial.print(error);
        Serial.write('\t');
        Serial.print(proportional_term);
        Serial.write('\t');
        Serial.print(integral_term);
        Serial.write('\t');
        Serial.print(derivative_term);
        Serial.write('\t');
        Serial.print(new_power);
        Serial.write('\t');
        Serial.print(duty);
        Serial.write('\t');
        Serial.print(motor->accumulated_error);
        #endif
        set_motor_params(motor_ind, motor->desired_dir, duty);
    }
    else
    {
        /* TODO: motor_brake or motor_coast? */
        set_motor_params(motor_ind, motor_brake, 0);
    }
    motor->last_control_update_us = now;
}

/* Updates the output control state for each motor */
static void update_control_state()
{
    update_motor_control(MOTOR_RHS);
    update_motor_control(MOTOR_LHS);
}

static void send_state_update()
{
    static uint8_t msg_countdown = 0;
    uint8_t clicks_remaining = (lhs_state.clicks_remaining >> 1) & 0xFF;
    if (msg_countdown == 0)
    {
        Serial.write(HEADER_BYTE);
        Serial.write(CMD_LHS_CLICKS_REMAINING);
        Serial.write(clicks_remaining);
        Serial.write(HEADER_BYTE ^ CMD_LHS_CLICKS_REMAINING ^ clicks_remaining);
    }

    clicks_remaining = (rhs_state.clicks_remaining >> 1) & 0xFF;
    if (msg_countdown == 0)
    {
        Serial.write(HEADER_BYTE);
        Serial.write(CMD_RHS_CLICKS_REMAINING);
        Serial.write(clicks_remaining);
        Serial.write(HEADER_BYTE ^ CMD_RHS_CLICKS_REMAINING ^ clicks_remaining);
        msg_countdown = CLICKS_UPDATE_FREQUENCY;
    }

    msg_countdown--;
}

