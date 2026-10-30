#pragma once
/* Exported functions prototypes ---------------------------------------------*/
#include "current_control.hpp"
#include "foc_mode.hpp"
#include "main.h"
#include "hall_sensor.hpp"
#include "motor_parameters.hpp"
#include "observer.hpp"
#include "pi_controller.hpp"
#include "pll.hpp"
#include "speed_control.hpp"
#include "startup.hpp"
#include "state_machine.hpp"
#include "types.hpp"
#include <cmath>
#include <cstdint>
#include <algorithm>

constexpr uint32_t  LED_PERIOD_MS   = 100;
constexpr float     DEG             = motor_control::PI / 180.0f;
constexpr uint8_t   ADC_BIT         = 12;
constexpr float     SHUNT_AMP_GAIN  = 20.0f;
constexpr float     SHUNT_RES_OHM   = 0.0015f;     // 0.0015 Ohm

constexpr float     I_MAX           = 20.0f;
constexpr float     VDC_MAX         = 30.0f;
constexpr float     VDC_MIN         = 12.0f;

constexpr float     I_ALIGN         = 1.0f;
constexpr uint32_t  ALIGN_MS        = 1000;

constexpr float     TS              = 1.0f / 20'000.0f;           // ISR period [s]
constexpr float     OMEGA_CAL       = motor_control::TWO_PI;     // Hall calibration: 1 Hz electrical [rad/s]
constexpr int       CAL_TURNS       = 4;                         // 4 electrical turns each way = 1 mechanical turn

constexpr float     SUPERVISOR_DT   = 0.001f;                    // Supervisor period [s]
constexpr float     I_OL            = 2.0f;                      // open-loop iq [A]
constexpr float     OMEGA_OL        = 500.0f * motor_control::TWO_PI / 60.0f * 4.0f;  // 500 rpm mechanical ≈ 209 rad/s electrical
constexpr float     OL_RAMP         = OMEGA_OL / 1.0f;           // reach the target speed in 1 s [rad/s²]
constexpr uint32_t  OL_HOLD_MS      = 2000;                      // wait at the target speed (to watch the SMO)
constexpr float     IQ_DOWN_STEP    = 0.002f;                    // iq decrease step in OL [A/ms]: 2 A in 1 s
constexpr float     IQ_OL_MIN       = 0.1f;                      // lowest iq when decreasing [A]
constexpr float     DELTA_SWITCH    = 0.3f;                      // switch to CL when |θ_ol − θ_smo| is below this [rad] ≈ 17°
constexpr float     OFFSET_STEP     = 0.005f;                    // OL→SMO angle offset fade step [rad/ms]: 1 rad in 200 ms

constexpr float     IQ_REF_MAX      = 5.0f;                      // iq limit in CURRENT mode [A]

constexpr float     SL_MIN_RPM      = 300.0f;                    // lowest speed reference in sensorless mode [rpm]
constexpr float     SL_LOST_RPM     = 200.0f;                    // SMO speed below this → lock is lost [rpm]
constexpr uint32_t  SL_LOST_MS      = 50;                        // fault if it stays below for this long [ms]

constexpr float SPEED_REF_MAX_RPM = 3560.0f;   // 57BLF02 nominal rated: = 3000 rpm
                                               //                tested: = 3560 rpm
extern "C" TIM_HandleTypeDef    htim3;
extern "C" TIM_HandleTypeDef    htim8;
extern "C" ADC_HandleTypeDef    hadc3;
extern "C" ADC_HandleTypeDef    hadc4;
extern "C" ADC_HandleTypeDef    hadc5;
extern "C" FDCAN_HandleTypeDef  hfdcan1;
extern "C" SPI_HandleTypeDef    hspi2;
extern "C" PCD_HandleTypeDef    hpcd_USB_FS;
extern "C" CORDIC_HandleTypeDef hcordic;
extern "C" DMA_HandleTypeDef    hdma_adc4;
extern "C" DMA_HandleTypeDef    hdma_adc5;

/* Shared data: each group has one writer -----------------------------------*/

// Writer: ADC3 ISR (20 kHz). Threads only read.
struct Measurements
{
    float                    Vabc_phase_divider_voltage[3];
    volatile float           theta_e_rad_hall;
    volatile float           theta_e_rad_pll;
    volatile float           theta_e_rad_smo;
    volatile float           theta_ol;

    volatile float           speed_rpm_hall;
    volatile float           speed_rpm_smo;
    motor_control::Iabc_t    Iabc;
    motor_control::Vabc_t    Vabc;
    motor_control::SinCos_t  SinCos;
    motor_control::Idq_ref_t idq_ref;
    motor_control::duties_t  duties;
    motor_control::V_AlphaBeta_t v_alphabeta;
    motor_control::I_AlphaBeta_t i_alphabeta;
    motor_control::Z_alphabeta_t z_alphabeta;
    uint8_t                  hall_state = 0;
};

// Writer: Supervisor. ISR reads. angle_src/loop change only through set_foc_mode().
struct FocConfig
{
    motor_control::AngleSource  angle_src       = motor_control::AngleSource::FIXED;
    motor_control::LoopMode     loop            = motor_control::LoopMode::OFF;
    volatile bool               record_hall_cal = false;   // record the OPEN_LOOP angle for Hall calibration
    motor_control::Idq_ref_t    idq_ref         = {};
    motor_control::SinCos_t     fixed_sincos    = {};      // FIXED angle
    volatile float              speed_ref_rpm   = 0.0f;
    volatile float              ol_speed        = 0.0f;    // open-loop angle speed [rad/s]
    volatile float              angle_offset    = 0.0f;    // added to the SMO angle [rad], fades to 0 after the OL→SMO switch
};

// Writer: Monitor. ISR and other threads read.
struct Status
{
    volatile float              theta_deg;
    volatile float              rpm;
    volatile int                dir;
    volatile float              Vdc;
    motor_control::Idq_fdb_t    idq_fdb;
};

struct Commands
{
    volatile bool  start         = false;
    volatile bool  stop          = false;
    volatile bool  reset_fault   = false;
    volatile bool  calibrate     = false;   // Hall calibration (only in IDLE)
    volatile bool  offset_cal    = false;   // refresh the current offset (only in IDLE, CiA 402 Switch On)
    volatile motor_control::SensorMode sensor = motor_control::SensorMode::SENSORLESS;  // fixed at START
    volatile motor_control::LoopMode   loop   = motor_control::LoopMode::SPEED;         // fixed at START
    volatile float speed_ref_rpm = 0.0f;    // in SPEED mode
    volatile float iq_ref        = 0.0f;    // in CURRENT mode [A]
};

// Writer: Supervisor (init and offsetCalPoll). ISR reads.
struct AdcCalib
{
    float    adc_ref_voltage         = 3.3f;
    float    volt_per_lsb            = adc_ref_voltage / ((1U << ADC_BIT) - 1U);
    float    amp_per_lsb             = volt_per_lsb / SHUNT_RES_OHM / SHUNT_AMP_GAIN;
    uint16_t amplifier_offset_raw[3] = {2048, 2048, 2048};
    bool     offset_valid            = false;   // at least one offset measurement is done (INIT waits for it)
};

// Writer: DMA (ADC4, ADC5).
struct AdcDma
{
    uint16_t adc_vdc_vrefint_raw[2];
    uint16_t Vabc_phase_raw[3];
};

extern Measurements meas;
extern FocConfig    foc_cfg;
extern Status       status;
extern AdcCalib     adc_cal;
extern AdcDma       adc_dma;
extern Commands     cmd;

// Fault latch: it has many writers (ISRs, Monitor, Supervisor), so it is kept outside the groups on purpose.
extern volatile motor_control::FaultCode  fault;

extern motor_control::HallSensor          hall;
extern motor_control::current_control     foc;
extern motor_control::speed_control       speed_ctrl;
extern motor_control::StateMachine        motor_state_machine;
extern motor_control::AnglePLL            pll;
extern motor_control::AnglePLL            pll_smo;
extern motor_control::smo_observer        smo;
extern motor_control::OpenLoopAngle       open_loop;

motor_control::SinCos_t cordicSinCos(float theta);

uint8_t readHallPins();
float   voltage_divider_get_voltage(float voltage, float kohm_top, float kohm_bottom);
void    set_foc_mode(motor_control::AngleSource angle_src, motor_control::LoopMode loop);
void    DRV_ENABLE_SET();
void    DRV_ENABLE_RESET();
void    powerOn();
void    powerOff();
void    offsetCalStart();
bool    offsetCalBusy();
void    offsetCalSample();
void    offsetCalPoll();
void    update_duties(motor_control::duties_t duties);
motor_control::Iabc_t get_abc_currents();
motor_control::Vabc_t get_abc_voltages();
float get_motor_angle_hal();
motor_control::Idq_ref_t get_idq_ref(float omega_ref_rpm, float omega_fdbck_rpm);




