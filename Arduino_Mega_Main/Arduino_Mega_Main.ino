/**
 * Autonomous Boat Control System
 * 
 * This program controls an autonomous boat with two modes of operation:
 * - Manual Mode: Wireless control via RF24 remote - 3 motors
 * - Automatic Mode: Navigation based on visual buoy detection (Raspberry Pi data) - 2 motors
 * 
 * The image processed by the Raspberry Pi has a resolution of 1280x720 pixels.
 * The Raspberry sends the centroid coordinates (cx, cy) and mass of the detected buoy.
 * 
 */

#include <Servo.h>        // Library for servo motor control
#include <SPI.h>          // SPI communication for RF24 module
#include <nRF24L01.h>     // Definitions for NRF24L01 module
#include <RF24.h>         // Main functionality for RF24 module

//---- CONSTANT DEFINITIONS ----//

// RF24 module pins and configuration
#define PINO_CE 9
#define PINO_CSN 10
const byte ENDERECO_RF24[6] = "00001";

// Motor pins (servos)
#define PINO_MOTOR_A 7
#define PINO_MOTOR_B 6
#define PINO_MOTOR_C 5

// RGB LED pins
#define PINO_LED_AMARELO 47  // Yellow component of RGB LED
#define PINO_LED_VERDE 46    // Green component of RGB LED

// Operation mode values
#define MODO_MANUAL 0
#define MODO_AUTOMATICO 1

// Commands received via RF24
#define CMD_MOTOR_A 1       // Prefix for Motor A commands
#define CMD_MOTOR_B 2       // Prefix for Motor B commands
#define CMD_MOTOR_C 3       // Prefix for Motor C commands
#define CMD_MODO 4          // Prefix for mode change
#define VAL_MODO_MANUAL 100 // Value to select Manual Mode
#define VAL_MODO_AUTO 200   // Value to select Automatic Mode

// Parameters for automatic control
#define CENTRO_IMAGEM_X 640       // Horizontal center of the image (1280x720)
#define PWM_CENTRO 1500           // Neutral PWM for motors
#define TOLERANCIA_CENTRADO_X 40  // Tolerance for considering the buoy centered on X axis (in pixels)
#define TOLERANCIA_CENTRADO_Y 20  // Tolerance for considering the buoy centered on Y axis (in pixels)
#define Y_MINIMO_PROXIMO 650      // Minimum Y value to consider the buoy close

// Proportional controller gains
#define KP_X_APROXIMACAO 0.5      // Gain for X control during approach
#define KP_Y_APROXIMACAO 0.7      // Gain for Y control during approach
#define KP_X_POSICIONAMENTO 0.5   // Gain for X control during positioning
#define KP_Y_POSICIONAMENTO 0.5   // Gain for Y control during positioning

// PWM Limits
#define PWM_MINIMO 1000           // Minimum PWM value for motors
#define PWM_MAXIMO 2000           // Maximum PWM value for motors
#define PWM_PROCURA 1300          // PWM for Motor C during buoy search

// LED blink time and data timeout
#define INTERVALO_PISCAR 500
#define TEMPO_MAX_SEM_DADOS 1000  // 1 second

//---- GLOBAL OBJECTS ----//
RF24 radio(PINO_CE, PINO_CSN);  // Object for RF communication
Servo motorA;                   // Servo for Motor A
Servo motorB;                   // Servo for Motor B
Servo motorC;                   // Servo for Motor C

//---- GLOBAL VARIABLES ----//

// Variables for RF communication
uint16_t dados = 0;             // Data received by RF24
int valor = 0;                  // Value extracted from received data
bool erro_detetado = false;     // Flag to indicate system error

// Operation mode control
int modo = MODO_MANUAL;         // Current mode (starts in manual)
int modo_anterior = -1;         // Control to detect mode changes

// Blinking LED control
unsigned long ultima_piscada = 0;
bool estado_led = false;

// Time control for Raspberry Pi data
unsigned long tempoUltimoDadoRasp = 0;
const unsigned long TEMPO_MAX_SEM_DADOS_CONST = TEMPO_MAX_SEM_DADOS;

// Variables for Raspberry Pi data
String dados_raspberry = "";    // String of received data
int cx = 0;                     // Centroid X coordinate
int cy = 0;                     // Centroid Y coordinate
int massa = 0;                  // Mass of detected object (currently unused)
bool dadosRecebidos = false;

// Variables for proportional control
int erro = 0;                     // Error between current and desired position
int termo_p = 0;                  // Proportional term
bool modo_posicionamento = false; // Flag to indicate if in positioning mode

// PWM values for motors
int pwm_a = PWM_CENTRO;
int pwm_b = PWM_CENTRO;
int pwm_c = PWM_CENTRO;

/**
 * Configures hardware and initializes system components
 * 
 * Initializes:
 * - Serial communication for debug and Raspberry Pi
 * - RF24 module for wireless communication
 * - Servo motors to neutral positions
 * - RGB LED to yellow (manual mode)
 */
void setup() {
  // Initialize serial communications
  Serial.begin(9600);           // For debugging
  Serial3.begin(9600);          // For communication with Raspberry Pi
  
  // Configure RF24 radio module
  radio.begin();
  radio.openReadingPipe(0, ENDERECO_RF24);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();

  // Configure servo motors
  motorA.attach(PINO_MOTOR_A);
  motorB.attach(PINO_MOTOR_B);
  motorC.attach(PINO_MOTOR_C);
  
  // Initialize servos to neutral position
  motorA.writeMicroseconds(PWM_CENTRO);
  motorB.writeMicroseconds(PWM_CENTRO);
  motorC.writeMicroseconds(PWM_CENTRO);

  // Configure RGB LED pins
  pinMode(PINO_LED_AMARELO, OUTPUT);
  pinMode(PINO_LED_VERDE, OUTPUT);
  
  // Start with yellow LED (manual mode)
  definir_led_amarelo();

  Serial.println(">> System started with 3 motors. Waiting for data...");
}

/**
 * Processes mode change commands received via RF24
 * 
 * @param valor The value of the received command
 * @return true if the mode is valid, false otherwise
 * 
 * Valid values:
 * - VAL_MODO_MANUAL (100): Activates manual mode
 * - VAL_MODO_AUTO (200): Activates automatic mode
 */
bool processar_mudanca_modo(int valor) {
  bool modo_valido = true;
  
  // Determine new mode based on received value
  switch (valor) {
    case VAL_MODO_MANUAL:
      modo = MODO_MANUAL;
      erro_detetado = false;
      break;
      
    case VAL_MODO_AUTO:
      modo = MODO_AUTOMATICO;
      erro_detetado = false;
      break;
      
    default:
      modo_valido = false;
      erro_detetado = true;
      break;
  }

  // Update indicators and notify change, only if mode has changed
  if (modo != modo_anterior) {
    if (modo == MODO_MANUAL) {
      Serial.println(">> Command: MANUAL MODE");
      definir_led_amarelo();
    } 
    else if (modo == MODO_AUTOMATICO) {
      Serial.println(">> Command: AUTOMATIC MODE");
      definir_led_verde();
      // Stop Motor C when switching to automatic mode
      motorC.writeMicroseconds(PWM_CENTRO);
      Serial.println(">> Motor C stopped (automatic mode)");
    }
    modo_anterior = modo;
  }
  
  return modo_valido;
}

/**
 * Sets RGB LED to yellow (Manual Mode)
 */
void definir_led_amarelo() {
  digitalWrite(PINO_LED_AMARELO, HIGH);
  digitalWrite(PINO_LED_VERDE, LOW);
}

/**
 * Sets RGB LED to green (Automatic Mode)
 */
void definir_led_verde() {
  digitalWrite(PINO_LED_AMARELO, LOW);
  digitalWrite(PINO_LED_VERDE, HIGH);
}

/**
 * Blinks the RGB LED in yellow to indicate error
 * 
 * Blinks with interval defined by INTERVALO_PISCAR
 */
void piscar_led_amarelo() {
  unsigned long agora = millis();
  if (agora - ultima_piscada >= INTERVALO_PISCAR) {
    ultima_piscada = agora;
    estado_led = !estado_led;
    
    if (estado_led) {
      definir_led_amarelo();
    } else {
      digitalWrite(PINO_LED_AMARELO, LOW);
      digitalWrite(PINO_LED_VERDE, LOW);
    }
  }
}

/**
 * Processes manual mode commands received via RF24
 * 
 * @param comando The command identifier (1 for Motor A, 2 for Motor B, 3 for Motor C)
 * @param valor The PWM value to apply to the motor (1000-2000)
 * 
 * Valid commands:
 * - CMD_MOTOR_A (1): Controls Motor A
 * - CMD_MOTOR_B (2): Controls Motor B
 * - CMD_MOTOR_C (3): Controls Motor C
 */
void processar_comando_manual(int comando, int valor) {
  switch (comando) {
    case CMD_MOTOR_A:
      motorA.writeMicroseconds(valor);
      Serial.print(">> PWM Motor A: ");
      Serial.println(valor);
      break;
      
    case CMD_MOTOR_B:
      motorB.writeMicroseconds(valor);
      Serial.print(">> PWM Motor B: ");
      Serial.println(valor);
      break;
      
    case CMD_MOTOR_C:
      motorC.writeMicroseconds(valor);
      Serial.print(">> PWM Motor C: ");
      Serial.println(valor);
      break;
      
    default:
      Serial.println(">> ERROR: Invalid motor command!");
      break;
  }

  // delay(500); // Debug motor values in manual mode
}

/**
 * Extracts data from the string received from Raspberry Pi
 * 
 * @param dados_str The received string in format "cx,cy,mass"
 * @return true if data was extracted successfully, false otherwise
 * 
 * Expected format: "cx,cy" where cx and cy are integer coordinates
 * Example: "320,480" represents cx=320, cy=480
 */
bool extrair_dados_raspberry(String dados_str) {
  dados_str.trim();  // Remove spaces and control characters

  int separador = dados_str.indexOf(',');

  // Check if comma is in a valid position
  if (separador > 0 && separador < dados_str.length() - 1) {
    // Extract the two parts of the string
    String parte1 = dados_str.substring(0, separador);
    String parte2 = dados_str.substring(separador + 1);

    parte1.trim();
    parte2.trim();

    // Convert to integer values
    cx = parte1.toInt();
    cy = parte2.toInt();

    // Update timestamp of last reception
    tempoUltimoDadoRasp = millis();

    Serial.print("Raspberry - cx: ");
    Serial.print(cx);
    Serial.print(" | cy: ");
    Serial.println(cy);

    return true;
  } else {
    Serial.print(">> ERROR: Invalid format! Received data: ");
    Serial.println(dados_str);
    erro_detetado = true;
    return false;
  }
}

/**
 * Calculates PWM values for motors in automatic mode
 * based on the detected buoy position
 * 
 * Implements a proportional controller with two modes:
 * - Approach: When the buoy is far, focus on approaching
 * - Positioning: When the buoy is close, maintain position
 * 
 * Only motors A and B are used in automatic mode
 */
void calcular_controlo_automatico() {
  // Calculate raw errors relative to image center
  int erroX = CENTRO_IMAGEM_X - cx;  // Horizontal error
  int erroY = Y_MINIMO_PROXIMO - cy; // Vertical error

  // Check if centered within tolerances
  bool centradoX = abs(erroX) < TOLERANCIA_CENTRADO_X;
  bool centradoY = abs(erroY) < TOLERANCIA_CENTRADO_Y;

  // Determine if entering positioning mode
  // (buoy close and centered on both axes)
  if (cy >= Y_MINIMO_PROXIMO && centradoX && centradoY) {
    modo_posicionamento = true;
  } else {
    modo_posicionamento = false;
  }

  // Apply dead zone for X axis if already centered
  if (centradoX) {
    erroX = 0;
  }

  // Apply dead zone for Y axis depending on mode
  if (!modo_posicionamento && centradoY) {
    erroY = 0;  // In approach, stop when already centered in Y
  }
  // In positioning, always correct Y to maintain position

  // Select gains according to mode
  float KpX = modo_posicionamento ? KP_X_POSICIONAMENTO : KP_X_APROXIMACAO;
  float KpY = modo_posicionamento ? KP_Y_POSICIONAMENTO : KP_Y_APROXIMACAO;

  // Calculate control signals
  int sinalVirar = erroX * KpX;    // Positive = turn right
  int sinalAvancar = erroY * KpY;  // Positive = advance

  // Combine signals to generate motor PWM
  // Differential system: A+turn, B-turn for rotation
  int motorA_output = PWM_CENTRO + sinalAvancar + sinalVirar;
  int motorB_output = PWM_CENTRO + sinalAvancar - sinalVirar;

  // Apply safety limits to PWM values
  motorA_output = constrain(motorA_output, PWM_MINIMO, PWM_MAXIMO);
  motorB_output = constrain(motorB_output, PWM_MINIMO, PWM_MAXIMO);

  // Send commands to motors
  motorA.writeMicroseconds(motorA_output);
  motorB.writeMicroseconds(motorB_output);

  // Debug: show current control state
  Serial.print("MODE: ");
  Serial.print(modo_posicionamento ? "POSITIONING" : "APPROACH");
  Serial.print(" | errorX: ");
  Serial.print(erroX);
  Serial.print(" | errorY: ");
  Serial.print(erroY);
  Serial.print(" | PWM A: ");
  Serial.print(motorA_output);
  Serial.print(" | PWM B: ");
  Serial.println(motorB_output);

  // delay(500); // Debug motor values in automatic mode
}

/**
 * Function to rotate on axis until buoy is found
 * 
 * When no data from Raspberry Pi for more than TEMPO_MAX_SEM_DADOS,
 * the boat rotates on itself using only Motor C to search for the buoy.
 * Motors A and B remain neutral during search.
 */
void procurarBoia() {
  // Keep motors A and B neutral
  motorA.writeMicroseconds(PWM_CENTRO);
  motorB.writeMicroseconds(PWM_CENTRO);

  // Turn on only Motor C for rotation
  motorC.writeMicroseconds(PWM_PROCURA);

  Serial.println("No buoy: rotating to search...");
}

/**
 * Main program loop, executed continuously
 * 
 * Performs the following tasks:
 * 1. Processes received RF24 commands
 * 2. Manages LED indicators according to state
 * 3. Executes automatic control when active
 * 4. Searches for buoy when no data from Raspberry Pi
 */
void loop() {
  // Check if data is available from RF24 module
  if (radio.available()) {
    radio.read(&dados, sizeof(dados));
    
    // Extract command type (prefix of data)
    int comando = dados / 10000;
    
    // Process command according to type
    switch (comando) {
      case CMD_MODO:
        // Process mode change command (format: 40000 + value)
        valor = dados % 40000;
        if (!processar_mudanca_modo(valor)) {
          Serial.println(">> Invalid mode received. Yellow light blinking.");
        }
        break;
        
      case CMD_MOTOR_A:
      case CMD_MOTOR_B:
      case CMD_MOTOR_C:
        // Process motor commands (only in manual mode)
        if (modo == MODO_MANUAL) {
          valor = dados % (comando * 10000);
          processar_comando_manual(comando, valor);
        }
        break;
        
      default:
        // Unknown command
        Serial.println(">> ERROR: Unknown command received!");
        erro_detetado = true;
        break;
    }
  }
  
  // LED indicator management
  if (erro_detetado) {
    piscar_led_amarelo();  // Blink yellow in case of error
  } else {
    // Fixed LED according to current mode
    if (modo == MODO_MANUAL) {
      definir_led_amarelo();
    } else if (modo == MODO_AUTOMATICO) {
      definir_led_verde();
    }
  }
  
  // Automatic mode: process data from Raspberry Pi
  if (modo == MODO_AUTOMATICO && !erro_detetado) {
    unsigned long tempoAtual = millis();
    
    // Check timeout of Raspberry Pi data
    if ((tempoAtual - tempoUltimoDadoRasp) > TEMPO_MAX_SEM_DADOS) {
      // No data: rotate on own axis using Motor C
      procurarBoia();
    } else {
      // With data: Motor C stays neutral
      motorC.writeMicroseconds(PWM_CENTRO);
    }

    // Process data received from Raspberry Pi
    if (Serial3.available()) {
      String dados = Serial3.readStringUntil('\n');
      dados.trim(); // Remove control characters

      if (dados.length() > 0) {
        if (extrair_dados_raspberry(dados)) {
          calcular_controlo_automatico();
          tempoUltimoDadoRasp = millis();
        } else {
          // Invalid data: clear residual buffer
          while (Serial3.available()) {
            Serial3.read();
          }
        }
      }
    }
  }
}