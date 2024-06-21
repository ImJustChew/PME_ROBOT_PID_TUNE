#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PID_v1.h>

// Define motor and encoder pins for one motor
#define IN1_1 12
#define IN1_2 13
#define DC1_1 23
#define DC1_2 22

#define IN2_1 32  // PWM_1 pin for motor 2
#define IN2_2 14  // PWM_2 pin for motor 2
#define DC2_1 21  // Encoder pin A for motor 2
#define DC2_2 19  // Encoder pin B for motor 2

#define IN3_1 25  // PWM_1 pin for motor 3
#define IN3_2 26  // PWM_2 pin for motor 3
#define DC3_1 18  // Encoder pin A for motor 3
#define DC3_2 5   // Encoder pin B for motor 3

#define IN4_1 27  // PWM_1 pin for motor 4
#define IN4_2 15  // PWM_2 pin for motor 4
#define DC4_1 4   // Encoder pin A for motor 4
#define DC4_2 2   // Encoder pin B for motor 4

// Define PWM channels
#define PWM_CHANNEL_1_1 0
#define PWM_CHANNEL_1_2 1

#define PWM_CHANNEL_2_1 2
#define PWM_CHANNEL_2_2 3

#define PWM_CHANNEL_3_1 4
#define PWM_CHANNEL_3_2 5

#define PWM_CHANNEL_4_1 6
#define PWM_CHANNEL_4_2 7

// variables

// speed of 200 is already max.

double input1, output1, setpoint1 = 0, kp1 = 2.6, ki1 = 1, kd1 = 0.08;  // MOTOR 1
PID myPID1(&input1, &output1, &setpoint1, kp1, ki1, kd1, DIRECT);  // PID_v1

double input2, output2, setpoint2 = 0, kp2 = 2.5, ki2 = 1, kd2 = 0.08;  // MOTOR 2
PID myPID2(&input2, &output2, &setpoint2, kp2, ki2, kd2, DIRECT);  // PID_v1

double input3, output3, setpoint3 = 0, kp3 = 3, ki3 = 6, kd3 = 0.06;  // MOTOR 3
PID myPID3(&input3, &output3, &setpoint3, kp3, ki3, kd3, DIRECT);  // PID_v1

double input4, output4, setpoint4 = 0, kp4 = 2.5, ki4 = 1, kd4 = 0.08;  // MOTOR 4
PID myPID4(&input4, &output4, &setpoint4, kp4, ki4, kd4, DIRECT);  // PID_v1


// Encoder objects
ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

void controlMotor(int output, int pwmChannel1, int pwmChannel2) {
    if (output > 0) {
        ledcWrite(pwmChannel1, (int)output);
        ledcWrite(pwmChannel2, 0);
    } else {
        ledcWrite(pwmChannel1, 0);
        ledcWrite(pwmChannel2, (int)-output);
    }
}


int getEncoderSpeed1() {
    static long lastPosition = 0;
    static unsigned long lastTime = 0;
    long newPosition = encoder1.getCount();
    unsigned long now = micros();
    double dt = (double)(now - lastTime) / (double)1000;
    long dx = newPosition - lastPosition;
    lastPosition = newPosition;
    lastTime = now;
    if (dt == 0) return 0;  // avoid division by zero
    return (double)dx / dt;
}

int getEncoderSpeed2() {
    static long lastPosition = 0;
    static unsigned long lastTime = 0;
    long newPosition = encoder2.getCount();
    unsigned long now = micros();
    double dt = (double)(now - lastTime) / (double)1000;
    long dx = (newPosition - lastPosition) * 2; // 2 is the gear ratio
    lastPosition = newPosition;
    lastTime = now;
    if (dt == 0) return 0;  // avoid division by zero
    return (double)dx / dt;
}

int getEncoderSpeed3() {
    static long lastPosition = 0;
    static unsigned long lastTime = 0;
    long newPosition = encoder3.getCount();
    unsigned long now = micros();
    double dt = (double)(now - lastTime) / (double)1000;
    long dx = newPosition - lastPosition;
    lastPosition = newPosition;
    lastTime = now;
    if (dt == 0) return 0;  // avoid division by zero
    return (double)dx / dt;
}

int getEncoderSpeed4() {
    static long lastPosition = 0;
    static unsigned long lastTime = 0;
    long newPosition = encoder4.getCount();
    unsigned long now = micros();
    double dt = (double)(now - lastTime) / (double)1000;
    long dx = newPosition - lastPosition;
    lastPosition = newPosition;
    lastTime = now;
    if (dt == 0) return 0;  // avoid division by zero
    return (double)dx / dt;
}

void setup() {
    // Initialize encoders
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder1.attachFullQuad(DC1_1, DC1_2);
    encoder2.attachFullQuad(DC2_1, DC2_2);
    encoder3.attachFullQuad(DC3_1, DC3_2);
    encoder4.attachFullQuad(DC4_1, DC4_2);
    // Initialize PWM channels
    ledcSetup(PWM_CHANNEL_1_1, 5000, 8);  // 5 kHz frequency, 8-bit resolution
    ledcSetup(PWM_CHANNEL_1_2, 5000, 8);
    ledcSetup(PWM_CHANNEL_2_1, 5000, 8);
    ledcSetup(PWM_CHANNEL_2_2, 5000, 8);
    ledcSetup(PWM_CHANNEL_3_1, 5000, 8);
    ledcSetup(PWM_CHANNEL_3_2, 5000, 8);
    ledcSetup(PWM_CHANNEL_4_1, 5000, 8);
    ledcSetup(PWM_CHANNEL_4_2, 5000, 8);

    // Attach PWM channels to GPIO pins
    ledcAttachPin(IN1_1, PWM_CHANNEL_1_1);
    ledcAttachPin(IN1_2, PWM_CHANNEL_1_2);
    ledcAttachPin(IN2_1, PWM_CHANNEL_2_1);
    ledcAttachPin(IN2_2, PWM_CHANNEL_2_2);
    ledcAttachPin(IN3_1, PWM_CHANNEL_3_1);
    ledcAttachPin(IN3_2, PWM_CHANNEL_3_2);
    ledcAttachPin(IN4_1, PWM_CHANNEL_4_1);
    ledcAttachPin(IN4_2, PWM_CHANNEL_4_2);

    Serial.begin(115200);
    if (!Serial) return;

    // myPID.SetMode(AUTOMATIC);  // the PID is turned on
    // myPID.SetOutputLimits(-255, 255);
    // setpoint = 0;

    myPID1.SetMode(AUTOMATIC);  // the PID is turned on
    myPID1.SetOutputLimits(-200, 200);
    setpoint1 = 30;

    myPID2.SetMode(AUTOMATIC);  // the PID is turned on
    myPID2.SetOutputLimits(-200, 200);
    setpoint2 = 30;

    myPID3.SetMode(AUTOMATIC);  // the PID is turned on
    myPID3.SetOutputLimits(-200, 200);
    setpoint3 = 30;

    myPID4.SetMode(AUTOMATIC);  // the PID is turned on
    myPID4.SetOutputLimits(-200, 200);
    setpoint4 = 30;
}

void loop() {
    // getEncoderSpeed();
    // // serial read a setpoint value
    if (Serial.available() > 0) {
        String input = Serial.readString();
        int value = input.toInt();
        setpoint1 = value;
        setpoint2 = value;
        setpoint3 = value;
        setpoint4 = value;
    }
    // read input "kp ki kd\n" values for m2  
    // if (Serial.available() > 0) {
    //     String input = Serial.readString();
    //     int kp, ki, kd;
    //     sscanf(input.c_str(), "%d %d %d", &kp, &ki, &kd);
    //     kp2 = kp;
    //     ki2 = ki;
    //     kd2 = kd;
    //     myPID2.SetTunings(kp2, ki2, kd2);
    // }
    // // flip setpoint2 0 and 30 every 5 seconds
    // if (millis() % 10000 < 5000) {
    //     setpoint2 = 0;
    // } else {
    //     setpoint2 = 30;
    // }

    // Read encoder value
    input1 = getEncoderSpeed1();
    input2 = getEncoderSpeed2();
    input3 = getEncoderSpeed3();
    input4 = getEncoderSpeed4();

    // Compute PID
    myPID1.Compute();
    myPID2.Compute();
    myPID3.Compute();
    myPID4.Compute();

    // Control motor
    controlMotor(output1, PWM_CHANNEL_1_1, PWM_CHANNEL_1_2);
    controlMotor(output2, PWM_CHANNEL_2_1, PWM_CHANNEL_2_2);
    controlMotor(output3, PWM_CHANNEL_3_1, PWM_CHANNEL_3_2);
    controlMotor(output4, PWM_CHANNEL_4_1, PWM_CHANNEL_4_2);
    
    // Print values for plotter
    Serial.print(setpoint1);
    Serial.print(" ");
    Serial.print(input1);
    Serial.print(" ");
    Serial.print(input2);
    Serial.print(" ");
    Serial.print(input3);
    Serial.print(" ");
    Serial.print(input4);
    Serial.print(" ");
    Serial.print(output1);
    Serial.print(" ");
    Serial.print(output2);
    Serial.print(" ");
    Serial.print(output3);
    Serial.print(" ");
    Serial.print(output4);
    Serial.println();
    delay(100);

}