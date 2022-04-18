#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     2 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const int currentSensor = A0;
const int voltageSensor = A1;

const int voltageSensorBanco3 = A2;
const int voltageSensorBanco2 = A3;
const int voltageSensorBanco1 = A4;

const int releCarregamento = 8;
const int releSaidaBateria = 4;
//const int releVoltagem1 = 9;
//const int releVoltagem2 = 10;
//const int releVoltagem3 = 11;

const int releArduino = 9;

const int releResistor1 = 5;
const int releResistor2 = 6;
const int releResistor3 = 7;

unsigned int x=0;
long tempo=30000, tempoAnterior=-tempo;

float vOUT = 0.0;
float vIN = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;
float value = 0.0;
float deltaBMS = 0.07;
int zeroAmps = 532; //552; //548; //ou 556; // 566 //569; medição sem corrente


float deltaPack =-0.07;
float delta2e3 = -0.03;
float delta3 = 0.08;
  
float tensaoMinima = 9.99;
float tensaoDesliga = 9.90;

float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,correnteCarga_mA=0.0,minCorrente=220.0,maxCorrente=2000.0;
float voltagemBanco1=0.0, voltagemBanco2=0.0, voltagemBanco3=0.0, voltagemBanco2e3=0.0, voltagemPack=0.0;

#define aref_voltage 4.367

void setup() {

  analogReference(EXTERNAL);
  
  Serial.begin(9600); //Start Serial Monitor to display current read value on Serial monitor

 if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

 // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
  delay(2000);


  pinMode(releCarregamento,OUTPUT);
  digitalWrite(releCarregamento,HIGH);
 
//  pinMode(releVoltagem1,OUTPUT);
//  digitalWrite(releVoltagem1,HIGH);
//  pinMode(releVoltagem2,OUTPUT);
//  digitalWrite(releVoltagem2,HIGH);
//  pinMode(releVoltagem3,OUTPUT);
//  digitalWrite(releVoltagem3,HIGH);


  pinMode(releArduino,OUTPUT);
  digitalWrite(releArduino,HIGH );  // Não entendi porque esse cara tem de estar high para estar ligado...
  pinMode(releSaidaBateria,OUTPUT);
  digitalWrite(releSaidaBateria,LOW );  // Não entendi porque esse cara tem de estar high para estar ligado...

  
  pinMode(releResistor1,OUTPUT);
  digitalWrite(releResistor1,HIGH);
  pinMode(releResistor2,OUTPUT);
  digitalWrite(releResistor2,HIGH);
  pinMode(releResistor3,OUTPUT);
  digitalWrite(releResistor3,HIGH);

  //analogReference(DEFAULT); //(Porta 5V), INTERNAL, EXTERNAL
 
}

void loop() {

   display.clearDisplay();
   display.setCursor(0,0);             // Start at top-left corner
   display.setTextSize(1);
   delay(1000);


   // ------------------------------------------------------------------------------
   // ---------------- CORRENTE ----------------------------------------------------
   // ------------------------------------------------------------------------------
   Samples=0.0;
   for (int x = 0; x < 150; x++){ //Get 150 samples
      AcsValue = analogRead(currentSensor);     //Read current sensor values  
      Samples = Samples + AcsValue;  //Add samples together
      delay (3); // let ADC settle before next sample 3ms
   }
   AvgAcs=Samples/150.0;    //Taking Average of Samples
   
   correnteCarga_mA = (zeroAmps - AvgAcs) / 22.0 * 1000;
   //correnteCarga_mA += 291; 
   
   
   //correnteCarga_mA = ( (aref_voltage * AvgAcs/1024.0) - aref_voltage/2)/0.185;
   //correnteCarga_mA = AvgAcs / 512.0 * 5; // 5 A max
   //correnteCarga_mA -= 0.100; //Ajuste para dar 0 mA quando sem carga.

   Serial.print("Corrente = ");
   Serial.print(correnteCarga_mA);//Print the read current on Serial monitor
   Serial.print("@");        
   Serial.println(AvgAcs);

   display.print("I=");
   display.print(correnteCarga_mA);//Print the read current on Serial monitor
   display.print("@");
   display.print(AvgAcs);
   
   // Desliga tudo se houver pico de corrente
   if (correnteCarga_mA >= 2500 or correnteCarga_mA <= -2500) {
     digitalWrite(releArduino,LOW); // desliga a fonte do arduino e só liga via botão.   
   }


   // ------------------------------------------------------------------------------
   // ------------- VOLTAGEM 
   // ------------------------------------------------------------------------------


   if (millis() > tempoAnterior + tempo) {

        
        tempoAnterior = millis();    

        //desligar quando a voltagem atingir um limite de voltagem nas células
         
        // Leitura da voltagem do carregador
        value = analogRead(voltageSensor);
        vOUT = value * aref_voltage / 1024.0;
        vIN = vOUT / (R2/(R1+R2));
        vIN += 0.28;
      
        // A tensão vindo do carregador precisa ter a tensão controlada
        // Caso esteja fora dos parâmetros não manda corrente para a bateria
        //if (vIN < 12.7 & correnteCarga_mA < maxCorrente)
        if (correnteCarga_mA < maxCorrente)
             digitalWrite(releCarregamento,LOW); //Low ativa o relê para carregar.
        else digitalWrite(releCarregamento,HIGH);

        //todo: ler de novo a corrente e checar rapidamente se é superior ao limite.


        // Desliga as luzes que estiverem eventualmente acesas
        digitalWrite(releResistor1,HIGH);
        digitalWrite(releResistor2,HIGH);
        digitalWrite(releResistor3,HIGH);
        delay(3000);
      
        // Mede a voltagem do primeiro S (0.0V a 12V)
        voltagemPack = leVoltagem(2);
      
        // Mede a voltagem do segundo S (0.0V a 8.4V)
        voltagemBanco2e3 = leVoltagem(1);
      
        // Mede a voltagem do terceiro S (0.0V a 4.2V)
        voltagemBanco3 = leVoltagem(3);

        voltagemBanco2 = voltagemBanco2e3 - voltagemBanco3;

        voltagemBanco1 = voltagemPack - voltagemBanco2e3;
      
      
        Serial.print("Pack: ");
        Serial.println(voltagemPack, 2);
        Serial.print("2e3 (0.0 a 8.4V): ");
        Serial.println(voltagemBanco2e3, 2);
        Serial.print("1 (8.4 a 12.6V): ");
        Serial.println(voltagemBanco1, 2);
        Serial.print("2 (4.2 a 8.4V): ");
        Serial.println(voltagemBanco2, 2);
        Serial.print("3 (0 a 4.2V): ");
        Serial.println(voltagemBanco3, 2);

        
        // 1
        if (voltagemBanco1 > voltagemBanco2 or voltagemBanco1 > voltagemBanco3) {
          if (voltagemBanco1 > min(voltagemBanco2,voltagemBanco3) + deltaBMS){
            Serial.println("acende a luz para o banco1");
            display.print("luz1");
            digitalWrite(releResistor1,LOW);
          }
        }
        // 2
        if (voltagemBanco2 > voltagemBanco1 or voltagemBanco2 > voltagemBanco3) {
          if (voltagemBanco2 > min(voltagemBanco1,voltagemBanco3) + deltaBMS){
            Serial.println("acende a luz para o banco2");
            display.print(", luz2");
            digitalWrite(releResistor2,LOW);
          }
        }
        // 3
        if (voltagemBanco3 > voltagemBanco1 or voltagemBanco3 > voltagemBanco2) {
          if (voltagemBanco3 > min(voltagemBanco1,voltagemBanco2) + deltaBMS){
            Serial.println("acende a luz para o banco3");
            display.print("luz3");
            digitalWrite(releResistor3,LOW);
          }
        }


   //float carga = voltagemPack/12.6*100;
   // y = -300 + 100x
   
   //delay(2000);
   }

   float carga = -300 + 100*voltagemPack/3;

   display.print("@");
   display.println(vIN);

   //display.println();
   display.setTextSize(3);             // Draw 2X-scale text
   display.setTextColor(SSD1306_WHITE);        // Draw white text

   display.print(carga,1);
   display.println("%");

   display.setTextSize(1);
   
   display.print(voltagemPack,2);
   display.print(", ");
   display.print(voltagemBanco2e3,2);
   display.print(", ");
   display.println(voltagemBanco3,2);

   display.print(voltagemBanco1,2);
   display.print(", ");
   display.print(voltagemBanco2,2);
   display.print(", ");
   display.println(voltagemBanco3,2);
   display.println(millis()/1000/30);
   
   display.display();

   // desliga a saida para a carga se a tensao cair ao nivel minimo
   if (voltagemPack <= tensaoMinima) {
      if (correnteCarga_mA < 150) {
        digitalWrite(releSaidaBateria,HIGH); 
      }
      // Se estiver carregando bastante entao nao desliga o arduino
      // Se nao estiver carregando ou carregando bem pouco, entao desliga tudo
      if (voltagemPack <= tensaoDesliga & correnteCarga_mA < 300) {
        digitalWrite(releArduino,LOW); // desliga a fonte do arduino e só liga via botão.
      }
   }

   //Religa se a tensão voltar a subir mais do que os 0.20V que normalmente sobe ao desligar a carga.
   if (voltagemPack >= tensaoMinima+0.50) {
        digitalWrite(releSaidaBateria,LOW);
      }

}

float leVoltagem(int banco) {
  //int rele = 0;
 
  if (banco <1 or banco > 3) return 0;

  //if      (banco == 1) rele = releVoltagem1;
  //else if (banco == 2) rele = releVoltagem2;
  //else if (banco == 3) rele = releVoltagem3;
 
  //digitalWrite(rele, LOW);
  
  delay(100);
  float leitura = 0.0;
  for (int x = 0; x < 150; x++){
    if      (banco == 1) leitura += analogRead(voltageSensorBanco1);
    else if (banco == 2) leitura += analogRead(voltageSensorBanco2);
    else if (banco == 3) leitura += analogRead(voltageSensorBanco3);
    //Serial.println(value);
    delay(3);
  }
  leitura = leitura/150.0;

  Serial.print("leitura: ");
  Serial.println(leitura);
  
  //digitalWrite(rele, HIGH);
  delay (10); // let ADC settle before next sample 3ms

  float vOUT = leitura * aref_voltage / 1024.0;
  vIN = vOUT / (R2/(R1+R2));

  if      (banco == 1) vIN += delta2e3; //-0.08; // 2e3
  else if (banco == 2) vIN += deltaPack; //-0.08; //pack
  else if (banco == 3) vIN += delta3; //0.08;
  
  return vIN;

}
