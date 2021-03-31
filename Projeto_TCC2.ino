// *****---André Soares Diniz---*****//
//
//Inclui as bibliotecas necessarias
//
#include <EmonLib.h>             //ler corrente e tensao
#include <Wire.h>                //comunicação I2C com o MPU6050
#include <LiquidCrystal_I2C.h>   //Indicação no display

// Configura as Bibliotecas
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
EnergyMonitor emon1; // Cria instancia de medição corrente e tensao.
//
// Configura Botoes e variaveis
//
int button = 12; //define botao para inciar processo de medição da velocidade'
const int emissor = 4;
const int receptor = 5;
unsigned long timer_display = 0;

// Realizando as definiçoes de variaveis

// ----- Variáveis globais -----
// Variavel para armazenar o tempo entre cada amostra
float accX;
float accY;
float accZ;
float auxX;
float auxY;
float auxZ;
float gX;
float gY;
float gZ;
float ta;
float tp = 0;
float sx;
float sy;
float sz;
float s0x = 0;
float s0y = 0;
float s0z = 0;
float troca;
float thetax = 0;
float thetay = 0;
float thetaz = 0;
int i, in, start;
float vel;
float massa;
float cmMsec;
float s0xv[0];
float s0xmedio;

unsigned long ttotal = 0;
unsigned long tf = 0;
unsigned long t_amostra = 0;
unsigned long  t_display = 0;


static float media = 0.0;
static int indice = 1;
bool comunicacao;
String Status;

// Variaveis para verificar se o objeto está parado
unsigned long t_parado = 0;
unsigned long t = 0;
bool parado = false;
// Variavel para guardar a distancia deslocada
float dist = 0;
//Inicializa o sensor nos pinos definidos acima

#define MPU6050_INT_STATUS         0x3A   // R  
#define MPU6050_ACCEL_XOUT_H       0x3B   // R  
#define MPU6050_ACCEL_XOUT_L       0x3C   // R  
#define MPU6050_ACCEL_YOUT_H       0x3D   // R  
#define MPU6050_ACCEL_YOUT_L       0x3E   // R  
#define MPU6050_ACCEL_ZOUT_H       0x3F   // R  
#define MPU6050_ACCEL_ZOUT_L       0x40   // R  
#define MPU6050_TEMP_OUT_H         0x41   // R  
#define MPU6050_TEMP_OUT_L         0x42   // R  
#define MPU6050_GYRO_XOUT_H        0x43   // R  
#define MPU6050_GYRO_XOUT_L        0x44   // R  
#define MPU6050_GYRO_YOUT_H        0x45   // R  
#define MPU6050_GYRO_YOUT_L        0x46   // R  
#define MPU6050_GYRO_ZOUT_H        0x47   // R  
#define MPU6050_GYRO_ZOUT_L        0x48   // R  

#define MPU6050_PWR_MGMT_1         0x6B   // R/W
#define MPU6050_WHO_AM_I           0x75   // R
#define MPU6050_I2C_ADDRESS 0x68
#define STOP_OFFSET  900

typedef union accel_t_gyro_union
{
  struct
  {
    uint8_t x_accel_h;
    uint8_t x_accel_l;
    uint8_t y_accel_h;
    uint8_t y_accel_l;
    uint8_t z_accel_h;
    uint8_t z_accel_l;
    uint8_t t_h;
    uint8_t t_l;
    uint8_t x_gyro_h;
    uint8_t x_gyro_l;
    uint8_t y_gyro_h;
    uint8_t y_gyro_l;
    uint8_t z_gyro_h;
    uint8_t z_gyro_l;
  } reg;
  struct
  {
    int16_t x_accel;
    int16_t y_accel;
    int16_t z_accel;
    int16_t temperature;
    int16_t x_gyro;
    int16_t y_gyro;
    int16_t z_gyro;
  } value;
};

void setup() {
  lcd.begin (16, 2);            // Inicia Display.
  Wire.begin();                 // Inicializa comunicação do I2C.

  pinMode(button, INPUT);       // Define botao para inicializar.
  pinMode(emissor, OUTPUT);
  pinMode(receptor, INPUT);


  int error;
  uint8_t c;
  in = 0;

  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);

  t_amostra = micros();

  // Inicia o Display e a Balança
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" INICIALIZANDO!");
  lcd.setCursor(0, 1);
  lcd.print("BALANCA DE WATT!");
  Serial.begin(9600);           // Inicializa a porta serial.
  emon1.voltage(0, 272, 1.7);// Configura a instancia: pino, valor de calibração, fase da tensão
  emon1.current(1, 111.1); //Configura a instancia: pino, valor de calibração
  delay(2500);

}

void loop() {
  in = 0;
  // lê a comunicação doPython
  int Sts = Serial.read();
  delay(5);
  if (Sts == 79) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("COMUNICACAO OK");
    comunicacao = 1;
    delay(1000);
  }
  else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("COMUNICACAO NOK");
    Serial.println("Comunicação com O Python NOK!");
    comunicacao = 0;
    delay(2000);
  }

  while (true) {

    float aux_ac = float(accX);
    int error;
    double dT;
    double Irms = emon1.calcIrms(1480);
    accel_t_gyro_union accel_t_gyro;
    error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));

    uint8_t swap;

#define SWAP(x,y) swap = x; x = y; y = swap

    SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
    SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
    SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
    SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
    SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
    SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
    SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);

    accX = (accel_t_gyro.value.x_accel);
    accY = (accel_t_gyro.value.y_accel);
    accZ = (accel_t_gyro.value.z_accel);

    dT = ((double)accel_t_gyro.value.temperature + 12412.0) / 340.0;
    accX = accX / 16384;
    if (accX >= 1)
    {
      accX = 1;
    }
    if (accX <= -1)
    {
      accX = -1;
    }
    accX = asin(accX) * 180 / 3.1415;
    accY = accY / 16384;
    if (accY >= 1)
    {
      accY = 1;
    }
    if (accY <= -1)
    {
      accY = -1;
    }
    accY = asin(accY) * 180 / 3.1415;
    accZ = accZ / 16384;
    if (accZ >= 1)
    {
      accZ = 1;
    }
    if (accZ <= -1)
    {
      accZ = -1;
    }
    accZ = asin(accZ) * 180 / 3.1415;

    troca = accX;
    accX = accY;
    accY = troca;

    gX = (accel_t_gyro.value.x_gyro);
    gY = (accel_t_gyro.value.y_gyro);
    gZ = (accel_t_gyro.value.z_gyro);

    // Passa para graus/seg
    gX = gX * 0.00875;
    gY = gY * 0.00875;
    gZ = gZ * 0.00875;

    // Integração de X
    ta = millis() * 0.001 - tp;
    tp = millis() * 0.001;
    sx = gX * ta + thetax; // thetax foi realimentado aqui para funcionar o filtro!!!!!!!!!
    s0x = sx;
    sy = gY * ta + thetay;
    s0y = sy;
    sz = gZ * ta + thetaz;
    s0z = sz;

    thetax = 0.98 * sx + (1 - 0.98) * accX;
    thetay = 0.98 * sy + (1 - 0.98) * accY;
    thetaz = 0.98 * sz + (1 - 0.98) * accZ;

    s0xv[in] =  s0x;

    //Serial.println(s0xv[in]);

    // Calculo da tensao

    emon1.calcVI(17, 2000);
    float tensao = emon1.Vrms;

    /*    // Calcula a distancia e a velocidade

        // establish variables for duration of the ping,
        // and the distance result in inches and centimeters:
        long duration, inches, cm;

        // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
        // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
        digitalWrite(emissor, LOW);
        delayMicroseconds(2);
        digitalWrite(emissor, HIGH);
        delayMicroseconds(5);
        digitalWrite(emissor, LOW);


        //o receptor faz a leitura do estado do pino e se retorna HIGH ele informa
        duration = pulseIn(receptor, HIGH);

        // converte a distância
        cm = microsecondsToCentimeters(duration);
        //Serial.print(cm);
        //Serial.print(" cm,");

        // Calculo da massa
        ttotal = tf - t;
        vel = cmMsec / 100 / t;
        massa = Irms * tensao / (9.81 * vel);*/

    float med = mediaMovel(s0y);

    // Envia valores para a Serial e para o LCD
    if (comunicacao == 1) {
      Serial.print("Eixo X:");
      Serial.print("; ");
      Serial.print(s0x, 2);
      Serial.print(";");
      Serial.print("Eixo y:; ");
      Serial.print(s0y, 2);
      Serial.print("; ");
      Serial.print("Eixo Z:; ");
      Serial.print(s0z, 2);
      Serial.print("; ");
      Serial.print("Tensão:; ");
      Serial.print(tensao);
      Serial.print("; ");
      Serial.print("Corrente:; ");
      Serial.print(Irms);
      Serial.print("; ");
      Serial.print("Temperatura:; ");
      Serial.print(abs(dT), 2);
      Serial.print("; ");
      Serial.print("Giro X:; ");
      Serial.print(gX, 2);
      Serial.print("; ");
      Serial.print("Giro Y:; ");
      Serial.print(gY, 2);
      Serial.print("; ");
      Serial.print(" ");
      Serial.print("Giro Z:; ");
      Serial.print(gZ, 2);
      Serial.print("; ");/*
    Serial.print(vel);
    Serial.print(" ");
*/
      Serial.println(F(""));
    }


    lcd.clear();
    lcd.setCursor(0, 0);
    // lcd.print("Eixo X: ");
    // lcd.print(s0x);
    lcd.print("Tensao: ");
    lcd.setCursor(8, 0);
    lcd.print(tensao);
    lcd.setCursor(0, 1);
    lcd.print("Eixo X m: ");
    lcd.setCursor(10, 1);
    lcd.print(abs(med), 2);

    delay(0);
  }
}

// Calculo da media movel

float mediaMovel(float xis) {
  static float media = 0.0;
  static int indice = 1;

  if (indice == 0 || indice == 7) {
    indice = 1;
    media = 0.0;
  }

  media = media + (xis - media) / indice++;

  return media;
}

int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while (Wire.available() && i < size)
  {
    buffer[i++] = Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}

int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}


int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
