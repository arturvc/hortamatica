# hortamatica
Projeto de irrigação controlada por Arduino e monitorada por Esp8266 NodeMcu

O projeto tem dois arquivos de código, o do Arduino e o do Esp8266.
Os sensores estão conectados no Arduino Uno, e envia os dados por Software Serial para o Esp8266, o qual publica os dados no servidor Adafruit.io usando a biblioteca de MQTT.

## Sensores
No projeto são usados:
- 2 sensores de umidade do solo (com saídas digital e analógica);
- 2 relés;
- 1 sensor BMP180 de pressão atmosférica e temperatura do ar;
- 1 sensor LDR de luminosidade.

No total são publicados 10 dados no Adafruit.io.
