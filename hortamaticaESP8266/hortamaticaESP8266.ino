/*
    Projeto Hortamática
    Irrigação controlada por Arduino e monitorada por ESP8266
    Desenvolvido por Artur V. Cordeiro
    Maio de 2020

*/

//  ***** Código usado no Esp8266 NodeMcu*****


//-------------------------
// Este código usa a biblioteca MQTT da Adafruit.
/***************************************************
  Adafruit MQTT Library ESP8266 Example

  Must use ESP8266 Arduino from:
    https://github.com/esp8266/Arduino

  Works great with Adafruit's Huzzah ESP board & Feather
  ----> https://www.adafruit.com/product/2471
  ----> https://www.adafruit.com/products/2821

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Tony DiCola for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/
#include <ESP8266WiFi.h>          //  Inclui a biblioteca ESP8266WiFi.
#include "Adafruit_MQTT.h"        //  Inclui a biblioteca Adafruit_MQTT.
#include "Adafruit_MQTT_Client.h" //  Inclui a biblioteca Adafruit_MQTT_Client.


//-------------------------
//  Biblioteca Software Serial para comunicação do Esp8266 com o Arduino.
#include<SoftwareSerial.h>
SoftwareSerial ARD_ESP(4, 5); //SRX=Dpin-D2; STX-DPin-D1
//  Cria o objeto "ARD_ESP" de comunicação serial.
//  O pino 4 (pino D2) é Receiver (RX) e o pino 5 (pino D1) é o Transmiter (TX).
//  Observação: tem uma confusão na pinagem da placa ESP8266,
//  tem os números chamados de GPIO e tem os pinos que estão impressos na placa,
//  são números diferentes. Por isso o GPIO4 corresponde ao pino D2,
//  e o GPIO5 corresponde ao pino D1.


//-------------------------
//------------------------- INFORMAÇÕES CONFIDENCIAIS ------------------------
//  Antes de compartilhar o código, apagar as informações de login e senha.

/************************* Ponto de acesso de WiFi *********************************/

#define WLAN_SSID       "nome da rede"    //  Nome da rede de WiFi.
#define WLAN_PASS       "senha"           //  Senha da rede de WiFi.

/************************* Setup de acesso do Adafruit.io *********************************/

#define AIO_SERVER      "io.adafruit.com" //  Endereço do servidor de MQTT.
#define AIO_SERVERPORT  1883              //  Número da porta do servidor, usar 8883 para SSL.
#define AIO_USERNAME    "usuario"         //  Nome de usuário no sistema da Adafruit.io.
#define AIO_KEY         "chave"           //  Chave de acesso no sistema da Adafruit.io.
//  Observação: AIO se refere a Adafruit.io.

//-------------------------
//  Configurações globais de acesso, não precisa alterar nada aqui.

// Cria uma classe do ESP8266 WiFiClient para conectar com o servidor MQTT
WiFiClient client;
//  Ou então usar WiFiFlientSecure para conexão SSL.
//WiFiClientSecure client;

//  Configura a classe de cliente MQTT passando os nomes e chaves de acesso, definidos anteriormente.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

//-------------------------
//  Configura os feeds para publicação (publishing).
// O caminho (path) do MQTT para o Adafruit.io segue o padrão: <nomeDoUsuário>/feeds/<nomeDoFeed>

Adafruit_MQTT_Publish pubUmidade1A = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/umidade1A");
Adafruit_MQTT_Publish pubUmidade2A = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/umidade2A");
Adafruit_MQTT_Publish pubUmidade1D = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/umidade1D");
Adafruit_MQTT_Publish pubUmidade2D = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/umidade2D");

Adafruit_MQTT_Publish pubRele1 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/rele1");
Adafruit_MQTT_Publish pubRele2 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/rele2");

Adafruit_MQTT_Publish pubTemperatura = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/temperatura");
Adafruit_MQTT_Publish pubPressao = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressao");
Adafruit_MQTT_Publish pubAltitude = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/altitude");

Adafruit_MQTT_Publish pubLuminosidade = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/luminosidade");

/*
  Esses 10 feeds correspondem aos 10 dados de sensores e atuadores enviados pelo Arduino Uno.
  O objeto "pubUmidade1A" se refere ao feed "umidade1A" exibido no servidor Adafruit.io,
  O objeto "pubUmidade2A" se refere ao feed "umidade2A" exibido no servidor Adafruit.io,
  e etc.

*/
//-------------------------

int valorUmidade1A;     //  Variável do valor do Sensor de Umidade 1 (de 0 a 1023).
int valorUmidade2A;     //  Variável do valor do Sensor de Umidade 2 (de 0 a 1023).
int valorUmidade1D;     //  Variável do valor do Sensor de Umidade 1 (de 0 a 1).
int valorUmidade2D;     //  Variável do valor do Sensor de Umidade 2 (de 0 a 1).
int rele1Estado;        //  Variável do estado lógico do Relé 1 (1 = desligado, 0 = ligado).
int rele2Estado;        //  Variável do estado lógico do Relé 2 (1 = desligado, 0 = ligado).
float bmpTemperatura;   //  Variável da temperatura, em graus Celcius.
int bmpPressao;         //  Variável da pressão, em Pascals (Pa).
float bmpAltitude;      //  Variável da altitude, em metros.
int valorLuminosidade;  // Variável do valor de luminosidade (de 0 a 1023).

String valores = "";    // String de texto que vai receber os valores na comunicação Software Serial.

int indiceInicial;      // Variável para guardar temporariamente o índice inicial da String "valores".
int indiceFinal;        // Variável para guardar temporariamente o índice final da String "valores".
// Os valores dos sensores foram enviados pela comunicação Software Serial usando letras como delimitadores,
// portanto serão usados os índices da String "valores" para identificar as delimitações.

//   Variável do tipo boolean (verdadeiro/falso) usada para receber os valores na comunicação Software Serial.
boolean atualizado = false;


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT); // Configura o pino do led interno (no Esp8266 é o pino D0) como saída.
  piscarLed(5); //  Executa a função piscarLed() por 5 vezes (que está neste sketch, abaixo do void loop()).

  Serial.begin(115200);   //  Inicia a comunicação serial padrão, na velocidade de 115200.
  ARD_ESP.begin(115200);  //  Inicia a comunicação do Software Serial ARD_ESP, na velocidade de 115200.

  //-------------------------
  Serial.println(F("Sistema usa o Adafruit MQTT")); // Imprime no Monitor Serial.
  Serial.println();
  Serial.println();

  //-------------------------
  // Conectar na rede de WiFi usando o nome "WLAN_SSID" e senha "WLAN_PASS" definidos anteriormente.
  Serial.print("Conectando na rede ");
  Serial.println(WLAN_SSID);          // Imprime no Monitor Serial o nome da rede de WiFi.
  WiFi.begin(WLAN_SSID, WLAN_PASS);   // Conecta na rede de WiFi.

  // Aguarda a conexão WiFi ser realizada.
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Imprime no Monitor Serial a confirmação da conexão e o número IP do Esp8266.
  Serial.println();
  Serial.println("WiFi conectado!");
  Serial.println("endereco IP: ");
  Serial.println(WiFi.localIP());
}

void loop()
{

  //  Estabelece e garante que a conexão com o servidor MQTT esteja "viva",
  //  fazendo a primeira conexão e reconectando automaticamente se for desconectado.
  //  Veja a definição da função MQTT_connect() abaixo do void loop()
  MQTT_connect();

  //-------------------------
  //  Se se houver dados pra receber na comunicação Software Serial ARD_ESP,
  //  e (&&) "atualizado" = false
  //  executa os comandos que estão entre as {}.
  if (ARD_ESP.available() && !atualizado)
  {
    //  Ler a string de dados da comunicação Software Serial até o fim (representado pelo "\r\n"),
    //  e salva na variável "valores".
    valores = ARD_ESP.readStringUntil('\r\n');
    // Atualiza como verdadeiro o estado da variável "atualizado".
    atualizado = true;
  }

  //-------------------------
  //  Se "atualizado" for verdadeiro,
  //  os dados dos sensores em "valores" serão salvos nas suas respectivas variáveis.
  if (atualizado) {
    //  Muda o estado de "atualizado" para falso,
    //  para que possa receber novos valores da comunicação Software Serial.
    atualizado = false;

    /*  Para separar os valores do sensores é usada a função "substring".
        Para entender a substring é necessário entender
        que uma string é um array de caracteres e que cada caractere tem um índice,
        que começa em 0. Por exemplo, "bom dia" é uma string de 7 caracteres,
        no qual o índice de b = 0,
        de o = 1,
        m = 2,
        ' ' = 3,
        d = 4,
        i = 5,
        a = 6.
        A substring pega um trecho da string a partir de um índice inicial e de um indice final,
        sendo o índice inicial inclusivo (o caractere correspondente é incluído),
        e o índice final é exclusivo (o caractere correspondente não é incluído).
        No caso da string "valores", que é enviada assim (os números mudam de acordo com a leitura dos sensores),
        A1023B1023C1D1E1F1G33.80H31356I375.88J1023K,
        portanto, o valorUmidade1A está entre A e B,
        o valorUmidade2A está entre B e C,
        e etc, até o valorLuminosidade que está entre J e K.
    */

    //  Como o índice inicial é inclusivo, e não queremos a letra A,
    //  é adicionado 1 para começar no caractere seguinte a A.
    indiceInicial = valores.indexOf('A') + 1; //  Define o índice inicial.
    indiceFinal = valores.indexOf('B');       //  Define o índice final.
    valorUmidade1A = valores.substring(indiceInicial, indiceFinal).toInt();
    //  Salva na variável valorUmidade1A o valor de leitura analógica pelo sensor de umidade 1.
    //  O "toInt() no final da expressão é necessário para converter a string (texto) em número inteiro.

    indiceInicial = valores.indexOf('B') + 1;
    indiceFinal = valores.indexOf('C');
    valorUmidade2A = valores.substring(indiceInicial, indiceFinal).toInt();

    indiceInicial = valores.indexOf('C') + 1;
    indiceFinal = valores.indexOf('D');
    valorUmidade1D = valores.substring(indiceInicial, indiceFinal).toInt();

    indiceInicial = valores.indexOf('D') + 1;
    indiceFinal = valores.indexOf('E');
    valorUmidade2D = valores.substring(indiceInicial, indiceFinal).toInt();

    indiceInicial = valores.indexOf('E') + 1;
    indiceFinal = valores.indexOf('F');
    rele1Estado = valores.substring(indiceInicial, indiceFinal).toInt();

    indiceInicial = valores.indexOf('F') + 1;
    indiceFinal = valores.indexOf('G');
    rele2Estado = valores.substring(indiceInicial, indiceFinal).toInt();

    indiceInicial = valores.indexOf('G') + 1;
    indiceFinal = valores.indexOf('H');
    bmpTemperatura = valores.substring(indiceInicial, indiceFinal).toFloat();
    // De modo semelhante, o "toFloat() no final da expressão converte a string em número decimal.

    indiceInicial = valores.indexOf('H') + 1;
    indiceFinal = valores.indexOf('I');
    bmpPressao = valores.substring(indiceInicial, indiceFinal).toInt();

    indiceInicial = valores.indexOf('I') + 1;
    indiceFinal = valores.indexOf('J');
    bmpAltitude = valores.substring(indiceInicial, indiceFinal).toFloat();

    indiceInicial = valores.indexOf('J') + 1;
    indiceFinal = valores.indexOf('K');
    valorLuminosidade = valores.substring(indiceInicial, indiceFinal).toFloat();


    //  Exibe no Monitor Serial os valores enviados pela comunicação Software Serial,
    //  salvos nas respectivas variáveis com a função substring.
    Serial.print("valorUmidade1A: ");
    Serial.println(valorUmidade1A);
    Serial.print("valorUmidade2A: ");
    Serial.println(valorUmidade2A);
    Serial.print("valorUmidade1D: ");
    Serial.println(valorUmidade1D);
    Serial.print("valorUmidade2D: ");
    Serial.println(valorUmidade2D);
    Serial.print("rele1Estado: ");
    Serial.println(rele1Estado);
    Serial.print("rele2Estado: ");
    Serial.println(rele2Estado);
    Serial.print("Temperatura: ");
    Serial.println(bmpTemperatura);
    Serial.print("Pressao PA: ");
    Serial.println(bmpPressao);
    Serial.print("Altitude: ");
    Serial.println(bmpAltitude);
    Serial.print("Luminosidade: ");
    Serial.println(valorLuminosidade);

    delay(10);

    //-------------------------
    // Em seguida são publicados os dados nos respectivos feeds do servidor Adafruit.io

    // *************************************** Umidade 1 Analogica
    //  Imprime no Monitor Serial o nome e valor de valorUmidade1A.
    Serial.print(F("\nSending valorUmidade1A "));
    Serial.print(valorUmidade1A);
    Serial.print("... ");

    //  Agora será de fato publicado no feed.
    //  Se tiver erro ou se o envio for bem sucedido,
    //  uma mensagem é impressa no Monitor Serial avisando.
    if (! pubUmidade1A.publish(valorUmidade1A)) {
      Serial.println(F("Erro no envio.."));
    } else {
      Serial.println(F("Ok, enviado!"));
      piscarLed(3); //  Executa a função piscarLed() piscando o led 3 vezes.
    }

    // Repete o mesmo procedimento de publicação dos feeds com os outros valores.

    // *************************************** Umidade 2 Analogica
    Serial.print(F("\nSending valorUmidade2A "));
    Serial.print(valorUmidade2A);
    Serial.print("... ");
    if (! pubUmidade2A.publish(valorUmidade2A)) {
      Serial.println(F("Erro no envio.."));
    } else {
      Serial.println(F("Ok, enviado!"));
      piscarLed(3);
    }
    // *************************************** Umidade 1 Digital
    Serial.print(F("\nSending valorUmidade1D "));
    Serial.print(valorUmidade1D);
    Serial.print("... ");
    if (! pubUmidade1D.publish(valorUmidade1D)) {
      Serial.println(F("Erro no envio.."));
    } else {
      Serial.println(F("Ok, enviado!"));
      piscarLed(3);
    }
    // *************************************** Umidade 2 Digital
    Serial.print(F("\nSending valorUmidade2D "));
    Serial.print(valorUmidade2D);
    Serial.print("... ");
    if (! pubUmidade2D.publish(valorUmidade2D)) {
      Serial.println(F("Erro no envio.."));
    } else {
      Serial.println(F("Ok, enviado!"));
      piscarLed(3);
    }
    // *************************************** Relé 1
    Serial.print(F("\nSending rele1Estado "));
    Serial.print(rele1Estado);
    Serial.print("... ");
    if (! pubRele1.publish(rele1Estado)) {
      Serial.println(F("Erro no envio.."));
    } else {
      Serial.println(F("Ok, enviado!"));
      piscarLed(3);
    }
    // *************************************** Relé 2
    Serial.print(F("\nSending rele1Estado "));
    Serial.print(rele2Estado);
    Serial.print("... ");
    if (! pubRele2.publish(rele2Estado)) {
      Serial.println(F("Erro no envio.."));
    } else {
      Serial.println(F("Ok, enviado!"));
      piscarLed(3);
    }
    // *************************************** Temperatura
    Serial.print(F("\nSending bmpTemperatura "));
    Serial.print(bmpTemperatura);
    Serial.print("... ");
    if (! pubTemperatura.publish(bmpTemperatura)) {
      Serial.println(F("Erro no envio.."));
    } else {
      Serial.println(F("Ok, enviado!"));
      piscarLed(3);
    }
    // *************************************** Pressão
    Serial.print(F("\nSending bmpPressao "));
    Serial.print(bmpPressao);
    Serial.print("... ");
    if (! pubPressao.publish(bmpPressao)) {
      Serial.println(F("Erro no envio.."));
    } else {
      Serial.println(F("Ok, enviado!"));
      piscarLed(3);
    }
    // *************************************** Altitude
    Serial.print(F("\nSending bmpAltitude "));
    Serial.print(bmpAltitude);
    Serial.print("... ");
    if (! pubAltitude.publish(bmpAltitude)) {
      Serial.println(F("Erro no envio.."));
    } else {
      Serial.println(F("Ok, enviado!"));
      piscarLed(3);
    }
    // *************************************** Luminosidade
    Serial.print(F("\nSending valorLuminosidade "));
    Serial.print(valorLuminosidade);
    Serial.print("... ");
    if (! pubLuminosidade.publish(valorLuminosidade)) {
      Serial.println(F("Erro no envio.."));
    } else {
      Serial.println(F("Ok, enviado!"));
      piscarLed(3);
    }
  }

  delay(200);
}

//-------------------------
//  Função para piscar led.

/*    É usado o laço for() para definir a quantidade de piscadas.
      A quantidade de piscada é passada entre parênteses,
      por exemplo, para piscar 3 vezes usar piscarLed(3).
      Em cada ciclo é executado um digitalWrite LOW e HIGH
      com intervalo de 100 milisegundos entre ligado e desligado.
*/

void piscarLed(int piscadas) {
  for (int i = 0; i < piscadas; i++) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
  delay(50);
}


//-------------------------
// Função da biblioteca MQTT para conectar e reconectar quando necessário no servidor da Adafruit.io.

void MQTT_connect() {
  int8_t ret;

  // Para se já estiver conectado.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Conectando ao MQTT... ");

  uint8_t retries = 3;
  
  // A conexão retornará 0 para conectado.
  while ((ret = mqtt.connect()) != 0) {
    Serial.println(mqtt.connectErrorString(ret));
    Serial.println("Tentando a conexão MQTT em 5 segundos...");
    mqtt.disconnect();
    delay(5000);  // Espera 5 segundos.
    retries--;
    if (retries == 0) {
      // basically die and wait for WDT to reset me
      while (1);
    }
  }
  Serial.println("MQTT Conectado!");
  piscarLed(10);  //  Quando MQTT conectar, a função piscarLed() é executada 10 vezes.
}
