/*
    Projeto Hortamática
    Irrigação controlada por Arduino e monitorada por ESP8266
    Desenvolvido por Artur V. Cordeiro
    Maio de 2020

*/

//  ***** Código usado no Arduino Uno *****

//-------------------------
//  Biblioteca Software Serial para comunicação do Arduino com o Esp8266.
#include<SoftwareSerial.h>
SoftwareSerial ARD_ESP(2, 3);
//  Cria o objeto "ARD_ESP" de comunicação serial.
//  O pino 2 é Receiver (RX) e o pino 3 é o Transmiter (TX).

//-------------------------
//  Biblioteca do sensor de pressão atmosférica BMP180.
#include <Adafruit_BMP085.h>
Adafruit_BMP085 sensorBMP;
//  Cria o objeto "sensorBMP"
//  O sensor BMP180 usa a interface I2C, e no Arduino Uno
//  o conector SDA (Serial Data Line) é ligado no pino analógico A4
//  e o conector SCL (Serial Clock Line) é ligado no pino analógico A5.
//  Observação: apesar do nome da biblioteca ser BMP085, ela funciona com o sensor BMP180.

float bmpTemperatura; //  Variável da temperatura, em graus Celcius.
int bmpPressao;       //  Variável da pressão, em Pascals (Pa).
float bmpAltitude;    //  Variável da altitude, em metros.

//--------------------------------------------

int sensorUmidade1A = A0; //  Pino do Sensor de Umidade 1, saída analógica.
int sensorUmidade2A = A1; //  Pino do Sensor de Umidade 2, saída analógica.
int releSolenoide1 = 6;   //  Pino do Relé 1.

int valorUmidade1A;       //  Variável do valor de leitura analógica do Sensor de Umidade 1 (de 0 a 1023).
int valorUmidade2A;       //  Variável do valor de leitura analógica do Sensor de Umidade 2 (de 0 a 1023).
//  Quanto maior o valor da leitura analógica, mas seco está o solo.

int mediaUmidade;         // Variável para calcular a média dos valores de umidade
int valorAtivarUmidade = 325; //  Valor limite para ativar os relés.

int rele1Estado = 1;      //  Variável do estado lógico do Relé 1 (1 = desligado, 0 = ligado).

int sensorLuminosidade = A2;  // Pino do Sensor de Luminosidade.
int valorLuminosidade;        // Variável do valor de luminosidade (de 0 a 1023).


void setup()
{
  Serial.begin(115200);   //  Inicia a comunicação serial padrão, na velocidade de 115200.
  ARD_ESP.begin(115200);  //  Inicia a comunicação do Software Serial ARD_ESP, na velocidade de 115200.

  //  Inicia o sensor BMP180.
  //  Se tiver problema, imprime mensagem de erro no Monitor Serial.
  if (!sensorBMP.begin()) {
    Serial.println("Erro! O sensor de pressão BMP180 não foi encontrado.");
    while (1) {}
  }

  //  Configura os pinos como saída (OUTPUT) e entrada (INPUT).
  pinMode(sensorLuminosidade, INPUT);
  pinMode(sensorUmidade1A, INPUT);
  pinMode(sensorUmidade2A, INPUT);
  pinMode(releSolenoide1, OUTPUT);

  //  Inicia o código definindo os relés como desligados,
  // (os valores do Rele1Estado e Rele2Estado foram inicializado com 1, portanto desligados).
  digitalWrite(releSolenoide1, rele1Estado);
  //  Observação: a função digitalWrite pode ser usada com LOW e HIGH, bem como 0 e 1.

  //  Pisca 5 vezes o led interno (no pino 13).
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}

void loop()
{
  //  Atualiza o valor das variáveis dos Sensores de Umidade com a leitura analógica.
  valorUmidade1A = analogRead(sensorUmidade1A);
  valorUmidade2A = analogRead(sensorUmidade2A);

  //  Calcula a média das leituras analógicas dos Sensores de Umidade.
  mediaUmidade = (valorUmidade1A + valorUmidade2A) / 2;


  // atualiza o valor das variáveis dos dados do sensor BMP180;
  bmpTemperatura = sensorBMP.readTemperature(); //  Temperatura em Celcius.
  bmpPressao = sensorBMP.readPressure();        //  Pressão em Pascals (Pa).
  bmpAltitude = sensorBMP.readAltitude();       //  Altitude em metro.

  //  Atualiza o valor da variável do Sensor de Luminosidade.
  valorLuminosidade = analogRead(sensorLuminosidade);

  //--------------------------------------------
  //  Condição lógica para ativar os relés, de acordo o valor da média dos Sensores de Umidade.
  //  Se a média for superior ao valor de ativação, os relés serão ligados (valor é 0).
  //  Caso contrário, os relés serão desligados (valor é 1)
  if (mediaUmidade > valorAtivarUmidade) {
    rele1Estado = 0;
  }
  else {
    rele1Estado = 1;
  }

  //--------------------------------------------
  //  Exibe no Monitor Serial os valores lidos pelos sensores.
  Serial.print("valorUmidade1A: ");
  Serial.println(valorUmidade1A);
  Serial.print("valorUmidade2A: ");
  Serial.println(valorUmidade2A);
  Serial.print(" * * * mediaUmidade: ");
  Serial.println(mediaUmidade);
  Serial.print("rele1Estado: ");
  Serial.println(rele1Estado);


  Serial.print("Temperatura: ");
  Serial.println(bmpTemperatura);
  Serial.print("Pressao PA: ");
  Serial.println(bmpPressao);
  Serial.print("Altitude: ");
  Serial.println(bmpAltitude);

  Serial.print("Luminosidade: ");
  Serial.println(valorLuminosidade);

  //--------------------------------------------
  //  Envio dos dados pelo Software Serial para o ESP8266.
  //  As letras que antecedem as variáveis são delimitadores
  //  para facilitar a aquisição dos dados no código do ESP8266.
  //  As dez variáveis são enviadas como uma "string" única de dados,
  //  as quais serão separadas em respectivas variáveis no código do ESP8266.
  ARD_ESP.print('A');
  ARD_ESP.print(valorUmidade1A);
  ARD_ESP.print('B');
  ARD_ESP.print(valorUmidade2A);
  ARD_ESP.print('C');
  ARD_ESP.print(rele1Estado);
  ARD_ESP.print('D');
  ARD_ESP.print(bmpTemperatura);
  ARD_ESP.print('E');
  ARD_ESP.print(bmpPressao);
  ARD_ESP.print('F');
  ARD_ESP.print(bmpAltitude);
  ARD_ESP.print('G');
  ARD_ESP.print(valorLuminosidade);
  ARD_ESP.print('H');
  ARD_ESP.println();

  //--------------------------------------------

  //  Atualiza o estado dos relés, de acordo com os valores definidos anteriormente na condição lógica.
  digitalWrite(releSolenoide1, rele1Estado);

  delay(30000); // Aguarda 30 segundos para fazer a próxima leitura e envio dos dados.
}
